import smbus2 as smbus
class LiDAR_Lite():

    def __init__(self, addr=0x62):
        """Creates the LiDAR_Lite object

        Args:
            addr (int, optional): the I2C address of the LiDAR-Lite
        """
        self.reg = {
            "ACQ_COMMAND":      0x00, # Device command
            "STATUS":           0x01, # System status
            "SIG_COUNT_VAL":    0x02, # Maximum acquisition count
            "ACQ_CONFIG_REG":   0x04, # Acquisition mode control
            "VELOCITY":         0x09, # Velocity measurement output
            "PEAK_CORR":        0x0c, # Peak value in correlation record
            "NOISE_PEAK":       0x0d, # Correlation record noise floor
            "SIGNAL_STRENGTH":  0x0e, # Received signal strength
            "FULL_DELAY_HIGH":  0x0f, # Distance measurement high byte
            "FULL_DELAY_LOW":   0x10, # Distance measurement low byte
            "OUTER_LOOP_COUNT": 0x11, # Burst measurement count control
            "REF_COUNT_VAL":    0x12, # Reference acquisition count
            "LAST_DELAY_HIGH":  0x14, # Previous distance measurement high byte
            "LAST_DELAY_LOW":   0x15, # Previous distance measurement low byte
            "UNIT_ID_HIGH":     0x16, # Serial number high byte
            "UNIT_ID_LOW":      0x17, # Serial number low byte
            "I2C_ID_HIGH":      0x18, # Write serial number high byte for I2C address unlock
            "I2C_ID_LOW":       0x19, # Write serial number low byte for I2C address unlock
            "I2C_SEC_ADDR":     0x1a, # Write new I2C address after unlock
            "THRESHOLD_BYPASS": 0x1c, # Peak detection threshold bypass
            "I2C_CONFIG":       0x1e, # Default address response control
            "COMMAND":          0x40, # State command
            "MEASURE_DELAY":    0x45, # Delay between automatic measurements
            "PEAK_BCK":         0x4c, # Second largest peak value in correlation record
            "CORR_DATA":        0x52, # Correlation record data low byte
            "CORR_DATA_SIGN":   0x53, # Correlation record data high byte
            "ACQ_SETTINGS":     0x5d, # Correlation record memory bank select
            "POWER_CONTROL":    0x65, # Power state control

            "FULL_DELAY":       0x0f,
            "LAST_DELAY":       0x14,
            "UNIT_ID":          0x16,
            "I2C_ID":           0x18,
        }
        self.addr = addr
        self.count = 0

    def connect(self, bus):
        """Connects the internal SMBus instance to an I2C bus

        Args:
            bus (int): I2C bus number (i.e. 1 corresponds to /dev/i2c-1)
        """
        self.bus = smbus.SMBus(bus)

    def wait_until_not_busy(self):
        """Waits until the LiDAR-Lite is ready for a new command
        """
        status = 1
        while status != 0:
            status = self.read_reg("STATUS") & 0b1

    def get_distance(self):
        """Gets the current LiDAR distance

        Returns:
            int: distance (in cm)
        """
        self.write_reg("ACQ_COMMAND", 0x04)
        self.wait_until_not_busy()
        return self.read_reg2("FULL_DELAY")

    def set_maximum_acquisition_count(self, mac=0x80):
        """The maximum acquisition count limits the number of times the device
        will integrate acquisitions to find a correlation record peak (from a
        returned signal), which occurs at long range or with low target
        reflectivity. This controls the minimum measurement rate and maximum
        range. The unit-less relationship is roughly as follows: rate = 1/n and
        range = n^(1/4), where n is the number of acquisitions.

        Args:
            mac (int, optional): maximum acquisition count
        """
        self.write_reg("SIG_COUNT_VAL", mac)

    def set_measurement_quick_termination_detection(self, mqtd=False):
        """If set, the device will terminate a distance measurement early if it
        anticipates that the signal peak in the correlation record will reach
        maximum value. This allows for faster and slightly less accurate
        operation at strong signal strengths without sacrificing long range
        performance.

        Args:
            mqtd (bool, optional): measurement quick termination detection
        """
        self.set_ACR_bit(3, not mqtd)

    def set_detection_sensitivity(self, ds=0x00):
        """The default valid measurement detection algorithm is based on the
        peak value, signal strength, and noise in the correlation record. This
        can be overridden to become a simple threshold criterion by setting a
        non-zero value. Recommended non-default values are 0x20 for higher
        sensitivity with more frequent erroneous measurements, and 0x60 for
        reduced sensitivity and fewer erroneous measurements.

        Args:
            ds (int, optional): detection sensitivity
        """
        self.write_reg("THRESHOLD_BYPASS", ds)

    def set_bm_repetition_count(self, rc=0x00):
        """This controls the number of times the device will retrigger itself.
        Values 0x00 or 0x01 result in the default one measurement per command.
        Values 0x02 to 0xfe directly set the repetition count. Value 0xff will
        enable free running mode after the host device sends an initial
        measurement command.

        Args:
            rc (int, optional): repetition count

        Raises:
            ValueError: if the repetition count is out of range
        """
        if rc < 0x00 or rc > 0xff:
            raise ValueError("Repetition count out of range 0x00 - 0xff")
        self.write_reg("OUTER_LOOP_COUNT", rc)

    def set_bm_delay(self, delay=0x14):
        """This sets the default delay between automatic measurements. The
        default delay (0xc8) corresponds to a 10 Hz repetition rate. A delay
        value of 0x14 roughly corresponds to 100Hz.

        Args:
            delay (int, optional): delay between automatic measurements
        """
        self.set_ACR_bit(5, True)
        self.write_reg("MEASURE_DELAY", delay)

    def reset_bm_delay(self):
        """This resets the delay between automatic measurements to the default
        delay of 10 Hz (0xc8).
        """
        self.set_bm_delay()
        self.set_ACR_bit(5, False)  

    def get_velocity(self):
        """The velocity measurement is the difference between the current
        measurement and the previous one, resulting in a signed (2s complement)
        8-bit number in cm. Positive velocity is away from the device. This can
        be combined with free running mode for a constant measurement frequency.
        The default free running frequency of 10 Hz therefore results in a
        velocity measurement in .1 m/s.

        Returns:
            int: velocity
        """
        vel = self.read_reg("VELOCITY")
        return signed(vel)

    def change_I2C_address(self, addr=0x62):
        """The I2C address can be changed from its default value. Available
        addresses are 7-bit values with a 0 in the least significant bit (even
        hexadecimal numbers).

        Args:
            addr (int, optional): new I2C address

        Raises:
            ValueError: if I2C address is invalid
        """
        if addr & 0b1 != 0:
            raise ValueError("Least significant bit is not 0")
        if addr >> 7 != 0:
            raise ValueError("Address is greater than 7 bits")
        sn = self.read_reg2("UNIT_ID")
        self.write_reg2("I2C_ID", sn)
        self.write_reg("I2C_SEC_ADDR", addr)
        self.addr = addr
        self.write_reg("I2C_CONFIG", 0x08)

    def set_power_control(self, disable_rc=False, device_sleep=False):
        """Disabling the receiver circuit saves roughly 40mA. After being
        re-enabled, the receiver circuit stabilizes by the time a measurement
        can be performed. Putting the device in sleep mode until the next I2C
        transaction saves 20mA. Wake-up time is only around 2 m/s shorter than
        the full power-on time. Both will reset all registers.

        Args:
            disable_rc (bool, optional): disable receiver circuit
            device_sleep (bool, optional): put the device in sleep mode
        """
        pc = 0
        if disable_rc:
            pc |= 0b001
        if device_sleep:
            pc |= 0b100
        self.write_reg("POWER_CONTROL", pc)

    def set_ACR_bit(self, bit, val=True):
        """Sets the specified bit in ACQ_CONFIG_REG to the specified value.

        Args:
            bit (int): the bit to set
            val (bool, optional): the value to set the bit to
        """
        acr = self.read_reg("ACQ_CONFIG_REG")
        if val:
            acr |= 1 << bit
        else:
            acr &= 0b1111111 - (1 << bit)
        self.write_reg("ACQ_CONFIG_REG", acr)

    def read_reg(self, reg):
        """Reads a specified register from the LiDAR

        Args:
            reg (string): the name of the register (contained in self.reg)

        Returns:
            byte: the value of the register (8 bits)
        """
        return self.bus.read_byte_data(self.addr, self.reg[reg])

    def read_reg2(self, reg):
        """Reads 2 registers from the LiDAR

        Args:
            reg (string): the name of the first register (contained in self.reg)

        Returns:
            byte: the value of both registers (16 bits)
        """
        high_byte = self.bus.read_byte_data(self.addr, self.reg[reg])
        low_byte = self.bus.read_byte_data(self.addr, self.reg[reg] + 1)
        return (high_byte << 8) + low_byte

    def write_reg(self, reg, val):
        """Writes a specified value into a specified register

        Args:
            reg (string): the name of the register (contained in self.reg)
            val (byte): the value to write to the register (8 bits)
        """
        self.bus.write_byte_data(self.addr, self.reg[reg], val)

    def write_reg2(self, reg, val):
        """Writes a specified value into 2 registers

        Args:
            reg (string): the name of the first register (contained in self.reg)
            val (byte): the value to write into both registers (16 bits)
        """
        self.bus.write_byte_data(self.addr, self.reg[reg], val >> 8)
        self.bus.write_byte_data(self.addr, self.reg[reg] + 1, val & 0b11111111)

    def signed(self, val):
        """Converts a signed 8 bit value into a signed number

        Args:
            val (byte): 8-bit signed value

        Returns:
            int: signed number
        """
        if val > 0b01111111:
            return (0b100000000 - val) * (-1)
        else:
            return val
