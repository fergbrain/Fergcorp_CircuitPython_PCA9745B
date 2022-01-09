# SPDX-FileCopyrightText: 2022 Andrew Ferguson for Fergcorp, LLC
#
# SPDX-License-Identifier: MIT

"""
`pca9745b_spi`
================================================================================
SPI driven CircuitPython driver for NXP PCA9745B constant current LED driver.
* Author(s): Andrew Ferguson

Based in part on: https://github.com/sensorberg/PCA9745B/
`This is a driver class written in (micro)python, to control the PCA9745B or similars LED driver connected via SPI. It currently provides support for PWM controlling LED matrixes of various configurations.`
Copyright (c) 2020 Mirko Vogt, Sensorberg GmbH (mirko.vogt@sensorberg.com)

Coding and formatting follows conventions established by Adafruit Industries;
neopixel_spi.py was used at the exemplar and is Copyright (c) 2019 Carter Nelson for Adafruit Industries

Implementation Notes
--------------------
**Hardware:**
* Hardware SPI port required on host platform.
**Software and Dependencies:**
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

