/*
      This file is part of Smoothie (http://smoothieware.org/).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

/*
    I2C vacuum sensor for pick and place (e.g. CFSensor XGZP6857D).

    Reads the sensor on M813 and reports:
        ok VAC:<mbar> RAW:<counts>
    or, if the sensor does not answer:
        ok VAC:ERR <reason>
    (non-numeric on purpose, so an OpenPnP read regex does not match a bogus value)

    config:
        vacuum_sensor.enable          true      # module is only loaded if true
        vacuum_sensor.sda_pin         0.0       # 0.0/0.1 (I2C1), 0.19/0.20 (I2C1), 0.10/0.11 (I2C2), 0.27/0.28 (I2C0)
        vacuum_sensor.scl_pin         0.1
        vacuum_sensor.i2c_address     0x6D      # 7-bit address
        vacuum_sensor.i2c_frequency   100000    # Hz
        vacuum_sensor.k_factor        64        # datasheet K for the pressure range (64 for 65..131 kPa)
*/

#pragma once

#include "libs/Module.h"
#include <stdint.h>

namespace mbed { class I2C; }

class VacuumSensor : public Module
{
public:
    VacuumSensor();
    ~VacuumSensor();

    void on_module_loaded();
    void on_gcode_received(void *argument);

private:
    bool write_reg(uint8_t reg, uint8_t value);
    bool read_regs(uint8_t reg, uint8_t *buf, int len);
    bool measure(int32_t &raw);

    mbed::I2C *i2c;
    uint8_t address;      // 7-bit
    int32_t k_factor;
};
