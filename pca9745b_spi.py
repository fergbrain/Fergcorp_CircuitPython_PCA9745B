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

# pylint: disable=ungrouped-imports
from adafruit_bus_device.spi_device import SPIDevice
from digitalio import DigitalInOut
from busio import SPI

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/fergbrain/CircuitPython_PCA9745B_SPI"


class PCA9745B:
    """
        A sequence of NXP PCA9745B.
        :param ~busio.SPI spi: The SPI bus to output PCA9745B data on.
        :param ~digitalio.DigitalInOut cs: The CS for the PCA9745B
        :param int bpp: Bytes per pixel. 3 for RGB and 4 for RGBW pixels.
        :param bool auto_write: True if the neopixels should immediately change when set. If False,
          ``show`` must be called explicitly.
        :param tuple pixel_order: Set the pixel color channel order. GRBW is set by default.
        :param int frequency: SPI bus frequency.
        :param bool debug: Set debug print messages
        Example:
        .. code-block:: python
            import board
            import pca9745b_spi
            pixels = pca9745b_spi.PCA9745B(board.SPI(), digitalio.DigitalInOut())
        """

    _REG_MODE1 = 0x00       # Mode register 1
    _REG_MODE2 = 0x01       # Mode register 1

    _REG_LEDOUT0 = 0x02     # LED output state 0
    _REG_LEDOUT1 = 0x03     # LED output state 1
    _REG_LEDOUT2 = 0x04     # LED output state 2
    _REG_LEDOUT3 = 0x05     # LED output state 3

    _REG_GRPPWM = 0x06      # group duty cycle control
    _REG_GRPFREQ = 0x07     # group frequency

    _REG_PWM0 = 0x08        # brightness control LED0
    _REG_PWM1 = 0x09        # brightness control LED1
    _REG_PWM2 = 0x0A        # brightness control LED2
    _REG_PWM3 = 0x0B        # brightness control LED3
    _REG_PWM4 = 0x0C        # brightness control LED4
    _REG_PWM5 = 0x0D        # brightness control LED5
    _REG_PWM6 = 0x0E        # brightness control LED6
    _REG_PWM7 = 0x0F        # brightness control LED7
    _REG_PWM8 = 0x10        # brightness control LED8
    _REG_PWM9 = 0x11        # brightness control LED9
    _REG_PWM10 = 0x12        # brightness control LED10
    _REG_PWM11 = 0x13        # brightness control LED11
    _REG_PWM12 = 0x14        # brightness control LED12
    _REG_PWM13 = 0x15        # brightness control LED13
    _REG_PWM14 = 0x16        # brightness control LED14
    _REG_PWM15 = 0x17        # brightness control LED15

    _REG_IREF0 = 0x18        # LED0 output current setting
    _REG_IREF1 = 0x19        # LED1 output current setting
    _REG_IREF2 = 0x1A        # LED2 output current setting
    _REG_IREF3 = 0x1B        # LED3 output current setting
    _REG_IREF4 = 0x1C        # LED4 output current setting
    _REG_IREF5 = 0x1D        # LED5 output current setting
    _REG_IREF6 = 0x1E        # LED6 output current setting
    _REG_IREF7 = 0x1F        # LED7 output current setting
    _REG_IREF8 = 0x20        # LED8 output current setting
    _REG_IREF9 = 0x21        # LED9 output current setting
    _REG_IREF10 = 0x22        # LED10 output current setting
    _REG_IREF11 = 0x23        # LED11 output current setting
    _REG_IREF12 = 0x24        # LED12 output current setting
    _REG_IREF13 = 0x25        # LED13 output current setting
    _REG_IREF14 = 0x26        # LED14 output current setting
    _REG_IREF15 = 0x27        # LED15 output current setting

    _REG_RAMP_RATE_GRP0 = 0x28  # ramp enable and rate control for group 0
    _REG_RAMP_RATE_GRP1 = 0x2C  # ramp enable and rate control for group 1
    _REG_RAMP_RATE_GRP2 = 0x30  # ramp enable and rate control for group 2
    _REG_RAMP_RATE_GRP3 = 0x34  # ramp enable and rate control for group 3

    _REG_STEP_TIME_GRP0 = 0x29  # step time control for group 0
    _REG_STEP_TIME_GRP1 = 0x2D  # step time control for group 1
    _REG_STEP_TIME_GRP2 = 0x31  # step time control for group 2
    _REG_STEP_TIME_GRP3 = 0x35  # step time control for group 3

    _REG_HOLD_CNTL_GRP0 = 0x2A  # hold ON/OFF time control for group 0
    _REG_HOLD_CNTL_GRP1 = 0x2E  # hold ON/OFF time control for group 1
    _REG_HOLD_CNTL_GRP2 = 0x32  # hold ON/OFF time control for group 2
    _REG_HOLD_CNTL_GRP3 = 0x36  # hold ON/OFF time control for group 3

    _REG_IREF_GRP0 = 0x2B       # output gain control for group 0
    _REG_IREF_GRP1 = 0x2F       # output gain control for group 1
    _REG_IREF_GRP2 = 0x33       # output gain control for group 2
    _REG_IREF_GRP3 = 0x37       # output gain control for group 3


    _REG_GRAD_MODE_SEL0 = 0x38  # gradation mode select register for channel 7 to channel 0
    _REG_GRAD_MODE_SEL1 = 0x39  # gradation mode select register for channel 15 to channel 8

    _REG_GRAD_GRP_SEL0 = 0x3A   # gradation group select for channel 3 to channel 0
    _REG_GRAD_GRP_SEL1 = 0x3B   # gradation group select for channel 7 to channel 4
    _REG_GRAD_GRP_SEL2 = 0x3C   # gradation group select for channel 11 to channel 8
    _REG_GRAD_GRP_SEL3 = 0x3D   # gradation group select for channel 15 to channel 12

    _REG_GRAD_CNTL = 0x3E       # gradation control register for all four groups


    _REG_OFFSET = 0x3F          # Offset/delay on LEDn outputs
    _REG_PWMALL = 0x40          # brightness control for all LEDn
    _REG_IREFALL = 0x41         # output gain control for all registers IREF0 to IREF15


    _REG_EFLAG0 = 0x42          # output error flag 0
    _REG_EFLAG1 = 0x43          # output error flag 1
    _REG_EFLAG2 = 0x44          # output error flag 2
    _REG_EFLAG3 = 0x45          # output error flag 3

    _CFG_LEDOUT_IND = 0b10  # not used
    _CFG_LEDOUT_GROUP = 0b11  # required for blinking, used by default
    _CFG_MODE2_BLINK = 0b00100000
    _CFG_MODE2_NORM = 0b00000000
    _CFG_IREFALL_OFF = 0x00
    _CFG_IREFALL_ON = 0xFF

    def __init__(
            self,
            spi: SPI,
            cs: DigitalInOut,
            bpp: int = 3,
            auto_write: bool = True,
            pixel_order: tuple = None,
            frequency: int = 500000,
            debug: bool = False) -> None:
        self.spi_driver = SPIDevice(spi, cs, baudrate=frequency, phase=0, polarity=0)
        self.debug = debug

    def _spi_write(self, cmd: hex, val: hex) -> None:
        if cmd >= 0x7F:
            raise Exception("Registries are are 7 bits and thus limited to a maximum position of 0x7F. You attempted to write to: %s" % cmd)
        if cmd >= 0x46:
            raise Exception("Registries 0x46 to 0x7F are reserved and should not be written to. You attempted to write to: %s" % cmd)
        if cmd >= 0x42:
            raise Exception("Registries 0x42 to 0x45 are read-only. You attempted to write to: %s" % cmd)

        with self.spi_driver as spi:
            spi.write(bytes([cmd << 1, val]))  # Bits 15-9 are for the register. Bit 8 (LSB) is: 1 = read, 0 = write. Bits 7-0 are for data.

    def _spi_read(self, cmd: hex) -> bytes:
        if cmd == (0x40 or 0x41):
            raise Exception("Registry 0x40 and 0x41 are write-only. You attempted to write to: %s" % cmd)
        
        with self.spi_driver as spi:
            spi.write(bytes([cmd << 1 | 0x1, 0xFF]))

        with self.spi_driver as spi:
            result = bytearray(2)
            spi.write_readinto(bytes([0xFF, 0xFF]), result)

        if self.debug:
            print("".join("{:02x}".format(x) for x in result))

        return result

    def clear(self):
        self._spi_write(self._REG_MODE2, 0x11)  # Clear errors

    def reset(self):
        self._spi_write(self._REG_MODE2, 0x00)  # Reset

        for LEDGRP in [self._REG_LEDOUT0, self._REG_LEDOUT1, self._REG_LEDOUT2, self._REG_LEDOUT3]:
            self._spi_write(LEDGRP, 0xAA)  # Default value

        self._spi_write(self._REG_GRPPWM, 0xFF)  # Default value
        self._spi_write(self._REG_GRPFREQ, 0x00)  # Default value

        for LEDPWM in [self._REG_PWM0,
                       self._REG_PWM1,
                       self._REG_PWM2,
                       self._REG_PWM3,
                       self._REG_PWM4,
                       self._REG_PWM5,
                       self._REG_PWM6,
                       self._REG_PWM7,
                       self._REG_PWM8,
                       self._REG_PWM9,
                       self._REG_PWM10,
                       self._REG_PWM11,
                       self._REG_PWM12,
                       self._REG_PWM13,
                       self._REG_PWM14,
                       self._REG_PWM15]:
            self._spi_write(LEDPWM, 0x00)  # Default value

        for IREF in [self._REG_IREF0,
                       self._REG_IREF1,
                       self._REG_IREF2,
                       self._REG_IREF3,
                       self._REG_IREF4,
                       self._REG_IREF5,
                       self._REG_IREF6,
                       self._REG_IREF7,
                       self._REG_IREF8,
                       self._REG_IREF9,
                       self._REG_IREF10,
                       self._REG_IREF11,
                       self._REG_IREF12,
                       self._REG_IREF13,
                       self._REG_IREF14,
                       self._REG_IREF15]:
            self._spi_write(IREF, 0x00)  # Default value

        if self.debug:
            print("Reset")

    def error_flag_exist(self) -> bool:
        results = self._spi_read(self._REG_MODE2)  # Verify errors cleared

    def set_led(self, led_num, pwm: hex = None, iref: hex = None):
        self._spi_write(self._REG_PWM0 + led_num, pwm)
        self._spi_write(self._REG_IREF0 + led_num, iref)

    def set_led_mode_by_group(self, group: int = None, mode: hex = 0xAA):
        if group not in [0, 1, 2, 3]:
            raise ValueError("You must select group 0, 1, 2, or 3. You selected: %s" % group)

        self._spi_write(self._REG_LEDOUT0 + group, mode)



    def set_gain_all(self, val):
        self._spi_write(self._REG_IREFALL, val)

    def set_pwm_all(self, val):
        self._spi_write(self._REG_PWMALL, val)


    def set_blink(self, freq: hex = None, duty: hex = None):
        """
            Blink LEDS

            :param int freq: Blinking period is controlled through 256 linear steps from 00h (67 ms, frequency 15 Hz) to FFh (16.8 s).
            :param int duty: General brightness for the 16 outputs is controlled through 255 linear steps from 00h (0 % duty cycle = LED output off) to FFh (99.6 % duty cycle = maximum brightness).

        """

        if not freq or not duty: # if either arg evals to False, deactivate blinking
            self._spi_write(self._REG_MODE2, self._CFG_MODE2_NORM)
            return

        if freq < 0 or freq > 255 or duty < 0 or duty > 255:
            raise ValueError("Value exceeds limits")
        else:
            self._spi_write(self._REG_MODE2, self._CFG_MODE2_BLINK) # TODO: unnecessary to set for every blink call
            self._spi_write(self._REG_GRPFREQ, freq)
            self._spi_write(self._REG_GRPPWM, duty)


    #TODO: Still working on this. Hard coded values for now for testing.
    def set_gradation_by_group(self, group: int = None,
                               ramp_up_enable: bool = True,
                               ramp_down_enable: bool = True,
                               ramp_rate: hex = 0x00,
                               cycle_time: int = 512,
                               hold_on_enable: bool = True,
                               hold_on_time: float = 1,
                               hold_off_enable: bool = True,
                               hold_off_time: float = 1
                               ):
        ramp_rate = 0xC5
        step_time = 0x3F
        hold_cntl = 0xE4
        iref = 0x58

        self._spi_write(self._REG_RAMP_RATE_GRP0 + group, ramp_rate)
        self._spi_write(self._REG_STEP_TIME_GRP0 + group, step_time)
        self._spi_write(self._REG_HOLD_CNTL_GRP0 + group, hold_cntl)
        self._spi_write(self._REG_IREF_GRP0 + group, iref)


        self._spi_write(self._REG_GRAD_MODE_SEL0, 0x01)
        self._spi_write(self._REG_GRAD_CNTL, 0x01)





