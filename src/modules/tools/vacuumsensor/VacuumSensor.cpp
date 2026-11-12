/*
      This file is part of Smoothie (http://smoothieware.org/).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "VacuumSensor.h"

#include "libs/Kernel.h"
#include "Gcode.h"
#include "Config.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "Conveyor.h"
#include "StreamOutputPool.h"

#include "mbed.h"

#include <stdio.h>
#include <string>
#include <stdlib.h>

#define vacuum_sensor_checksum   CHECKSUM("vacuum_sensor")
#define enable_checksum          CHECKSUM("enable")
#define sda_pin_checksum         CHECKSUM("sda_pin")
#define scl_pin_checksum         CHECKSUM("scl_pin")
#define i2c_address_checksum     CHECKSUM("i2c_address")
#define i2c_frequency_checksum   CHECKSUM("i2c_frequency")
#define k_factor_checksum        CHECKSUM("k_factor")

#define VACUUM_MCODE 813

// XGZP6857D registers
#define REG_DATA_MSB  0x06   // 0x06..0x08 pressure, 24 bit signed
#define REG_CMD       0x30
#define CMD_COMBINED  0x0A   // Sco=1, measurement_ctrl=010 (one shot temperature + pressure)
#define CMD_SCO_BIT   0x08

// Only the hardware I2C pin pairs of the LPC17xx are accepted.
struct I2CPinPair { const char *sda; const char *scl; PinName sda_pin; PinName scl_pin; };
static const I2CPinPair i2c_pairs[] = {
    { "0.0",  "0.1",  P0_0,  P0_1  },   // I2C1
    { "0.19", "0.20", P0_19, P0_20 },   // I2C1
    { "0.10", "0.11", P0_10, P0_11 },   // I2C2
    { "0.27", "0.28", P0_27, P0_28 },   // I2C0 (on Smoothieboard used by digipot current control!)
};

VacuumSensor::VacuumSensor() : i2c(nullptr), address(0x6D), k_factor(64)
{
}

VacuumSensor::~VacuumSensor()
{
    delete i2c;
}

void VacuumSensor::on_module_loaded()
{
    if(!THEKERNEL->config->value(vacuum_sensor_checksum, enable_checksum)->by_default(false)->as_bool()) {
        delete this;
        return;
    }

    std::string sda = THEKERNEL->config->value(vacuum_sensor_checksum, sda_pin_checksum)->by_default("0.0")->as_string();
    std::string scl = THEKERNEL->config->value(vacuum_sensor_checksum, scl_pin_checksum)->by_default("0.1")->as_string();

    const I2CPinPair *pair = nullptr;
    for(const auto &p : i2c_pairs) {
        if(sda == p.sda && scl == p.scl) { pair = &p; break; }
    }
    if(pair == nullptr) {
        THEKERNEL->streams->printf("VacuumSensor: invalid I2C pins sda=%s scl=%s, module disabled\n", sda.c_str(), scl.c_str());
        delete this;
        return;
    }

    std::string addr = THEKERNEL->config->value(vacuum_sensor_checksum, i2c_address_checksum)->by_default("0x6D")->as_string();
    this->address = (uint8_t)strtol(addr.c_str(), nullptr, 0);

    int freq = THEKERNEL->config->value(vacuum_sensor_checksum, i2c_frequency_checksum)->by_default(100000)->as_int();
    this->k_factor = THEKERNEL->config->value(vacuum_sensor_checksum, k_factor_checksum)->by_default(64)->as_int();
    if(this->k_factor <= 0) this->k_factor = 64;

    this->i2c = new I2C(pair->sda_pin, pair->scl_pin);
    this->i2c->frequency(freq);

    register_for_event(ON_GCODE_RECEIVED);
}

bool VacuumSensor::write_reg(uint8_t reg, uint8_t value)
{
    char buf[2] = { (char)reg, (char)value };
    return i2c->write(address << 1, buf, 2) == 0;
}

bool VacuumSensor::read_regs(uint8_t reg, uint8_t *buf, int len)
{
    char r = (char)reg;
    if(i2c->write(address << 1, &r, 1, true) != 0) return false;   // repeated start
    return i2c->read((address << 1) | 1, (char *)buf, len) == 0;
}

bool VacuumSensor::measure(int32_t &raw)
{
    if(!write_reg(REG_CMD, CMD_COMBINED)) return false;

    // wait until Sco bit is cleared by the sensor (conversion done), max ~50 ms
    uint8_t cmd = CMD_SCO_BIT;
    for(int i = 0; i < 100; i++) {
        wait_us(500);
        if(!read_regs(REG_CMD, &cmd, 1)) return false;
        if((cmd & CMD_SCO_BIT) == 0) break;
    }
    if(cmd & CMD_SCO_BIT) return false;

    uint8_t d[3];
    if(!read_regs(REG_DATA_MSB, d, 3)) return false;

    raw = ((int32_t)d[0] << 16) | ((int32_t)d[1] << 8) | (int32_t)d[2];
    if(raw & 0x00800000) raw |= 0xFF000000; // sign extend 24 -> 32 bit
    return true;
}

void VacuumSensor::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if(!gcode->has_m || gcode->m != VACUUM_MCODE) return;

    // read after all queued moves are finished (same as M400), so the value belongs to the current position
    THEKERNEL->conveyor->wait_for_idle();
    if(THEKERNEL->is_halted()) return;

    int32_t raw;
    if(!measure(raw)) {
        gcode->txt_after_ok = "VAC:ERR no response from sensor";
        return;
    }

    // pressure [Pa] = raw / K, [mbar] = raw / (K * 100), rounded to nearest
    int32_t div  = k_factor * 100;
    int32_t mbar = (raw >= 0) ? (raw + div / 2) / div : (raw - div / 2) / div;

    char buf[48];
    snprintf(buf, sizeof(buf), "VAC:%ld RAW:%ld", (long)mbar, (long)raw);
    gcode->txt_after_ok = buf;
}
