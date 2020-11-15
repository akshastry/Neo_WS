import smbus2 as smbus
class LiDAR_Lite_v4():

    def __init__(self, addr=0x62):
        """Creates the LiDAR_Lite object

        Args:
            addr (int, optional): the I2C address of the LiDAR-Lite
        """
        self.reg = {
            "ACQ_COMMAND":          0x00, # Device command
            "STATUS":               0x01, # System status
            "ACQUISITION_COUNT":    0x05, # Maximum acquisition count
            "FULL_DELAY_LOW":       0x10, # Distance measurement low byte
            "FULL_DELAY_HIGH":      0x11, # Distance measurement high byte
            "I2C_SEC_ADDR":         0x1a, # Write new I2C address after unlock
            "I2C_CONFIG":           0x1b, # Default address response control
            "DETECTION_SENSITIVITY":0x1c, # Peak detection threshold bypass
            "LIB_VERSION":          0x30, # Read Garmin software library version string
            "CORR_DATA":            0x52, # Correlation record data control
            "CP_VER_LO":            0x72, # Coprocessor firmware version low byte
            "CP_VER_HI":            0x73, # Coprocessor firmware version high byte
            "BOARD_TEMPERATURE":    0xe0, # Board temperature
            "HARDWARE_VERSION":     0xe1, # Board hardware version
            "POWER_MODE":           0xe2, # Power state control
            "MEASUREMENT_INTERVAL": 0xe3, # Delay between automatic measurements
            "FACTORY_RESET":        0xe4, # Reset default settings
            "QUICK_TERMINATION":    0xe5, # Quick acquisition termination
            "START_BOOTLOADER":     0xe6, # Start secure Bluetooth LE Bootloader 
            "ENABLE_FLASH_STORAGE": 0xea, # Store register settings
            "HIGH_ACCURACY_MODE":   0xeb, # Improved accuracy setting
            "SOC_TEMPERATURE":      0xec, # SoC temperature
            "ENABLE_ANT_RADIO":     0xf0, # Enable ANT wireless communication
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
        return self.read_reg2("FULL_DELAY_LOW")

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
        self.write_reg("ACQUISITION_COUNT", mac)


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
        self.write_reg("DETECTION_SENSITIVITY", ds)


    def set_bm_delay(self, delay=0xFF):
        """This sets the default delay between automatic measurements. The
        default delay (0xc8) corresponds to a 10 Hz repetition rate. A delay
        value of 0x14 roughly corresponds to 100Hz.

        Args:
            delay (int, optional): delay between automatic measurements
        """
        self.write_reg("MEASUREMENT_INTERVAL", delay)

    def set_high_acc(self, delay=0x14):
        """This sets the default delay between automatic measurements. The
        default delay (0xc8) corresponds to a 10 Hz repetition rate. A delay
        value of 0x14 roughly corresponds to 100Hz.

        Args:
            delay (int, optional): delay between automatic measurements
        """
        self.write_reg("HIGH_ACCURACY_MODE", delay)


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
        low_byte = self.bus.read_byte_data(self.addr, self.reg[reg])
        high_byte = self.bus.read_byte_data(self.addr, self.reg[reg] + 1)
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
