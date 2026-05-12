# ================================================================
# sensor_interface.py
# MicroPython -- Raspberry Pi Pico
#
# AS5600 magnetic angle sensor  -- inner loop (shaft velocity)
#   I2C address : 0x36 (fixed, cannot change)
#   Measures shaft rotation -> linear piston position and velocity
#
# MS5837-30BA pressure sensor   -- outer loop (water depth)
#   I2C address : 0x76 (fixed, cannot change)
#   Measures absolute pressure -> depth below surface
#
# Wiring (both sensors share I2C bus 0):
#   SDA -> GP4 (pin 6)
#   SCL -> GP5 (pin 7)
#   VCC -> 3.3V (pin 36)  -- do NOT connect to 5V
#   GND -> GND (pin 38)
#   AS5600 PS pin -> 3.3V  -- selects I2C mode (CRITICAL -- float = broken)
# ================================================================
 
from machine import I2C, Pin
import time
import math
 
# ================================================================
# AS5600 -- 12-bit magnetic angle sensor
# ================================================================
AS5600_ADDR      = 0x36
AS5600_ANGLE_REG = 0x0C   # filtered angle output (12-bit, two bytes)
AS5600_STATUS    = 0x0B   # magnet detection status
 
class AS5600:
    def __init__(self, i2c, Ks, alpha_vel, counts_per_rev=4096):
        """
        i2c           : machine.I2C object
        Ks            : screw gain [m/rad] from parameters.py
        alpha_vel     : velocity low-pass filter coefficient from parameters.py
        counts_per_rev: 4096 (12-bit = 0 to 4095 counts per revolution)
 
        Coordinate convention:
            shaft_pos = 0.000m -> piston fully extended (surface, max buoyancy)
            shaft_pos = 0.162m -> piston fully retracted (max depth, min buoyancy)
            Positive shaft velocity -> retracting -> vehicle sinking
            Negative shaft velocity -> extending  -> vehicle rising
        """
        self.i2c   = i2c
        self.Ks    = Ks
        self.CPR   = counts_per_rev
        self.ALPHA = alpha_vel      # 0.9048 at 200Hz for 20 rad/s cutoff
 
        # Multi-turn tracking (AS5600 only outputs 0-4095 per revolution)
        self._count_prev  = None    # last raw count reading
        self._turn_offset = 0       # cumulative full-turn correction in counts
        self._zero_count  = 0       # raw count at calibration zero
 
        # Velocity estimation state
        self._pos_prev = 0.0        # position from previous loop iteration [m]
        self._vel_f    = 0.0        # filtered velocity output [m/s]
 
    def check_magnet(self):
        """
        Verify magnet is correctly positioned over the AS5600.
        Call once at startup before any other AS5600 methods.
 
        Status register bits:
            MD (bit 5) = 1 -> magnet detected correctly
            ML (bit 4) = 1 -> magnet too weak  -> move magnet closer
            MH (bit 3) = 1 -> magnet too strong -> move magnet further away
 
        Raises RuntimeError if no magnet is detected.
        """
        status = self.i2c.readfrom_mem(AS5600_ADDR, AS5600_STATUS, 1)[0]
        md = (status >> 5) & 1
        ml = (status >> 4) & 1
        mh = (status >> 3) & 1
        if not md:
            raise RuntimeError("AS5600: no magnet detected -- check placement and wiring")
        if ml:
            print("AS5600 WARNING: magnet too weak -- move magnet closer to sensor")
        if mh:
            print("AS5600 WARNING: magnet too strong -- move magnet further from sensor")
        print("AS5600: magnet detected OK")
        return md
 
    def _read_raw_count(self):
        """
        Read 12-bit angle from registers 0x0C (high byte) and 0x0D (low byte).
        Upper 4 bits of the high byte are unused and masked to zero.
        Returns integer 0-4095 representing current shaft angle.
        """
        data = self.i2c.readfrom_mem(AS5600_ADDR, AS5600_ANGLE_REG, 2)
        return ((data[0] & 0x0F) << 8) | data[1]
 
    def calibrate(self):
        """
        Set the position zero reference.
        Call with piston FULLY EXTENDED (maximum buoyancy, vehicle at surface).
        This maps shaft_pos = 0.000m to that physical position.
 
        Must be called before read_position() or read_velocity().
        Resets all internal state to zero.
        """
        self._zero_count  = self._read_raw_count()
        self._count_prev  = self._zero_count
        self._turn_offset = 0
        self._pos_prev    = 0.0
        self._vel_f       = 0.0
        print(f"AS5600: zeroed at raw count {self._zero_count}")
 
    def read_position(self):
        """
        Returns cumulative linear piston position [m] from the zero reference.
        Handles multi-turn shaft rollover automatically.
 
        Rollover detection logic:
            AS5600 wraps from 4095->0 (forward) or 0->4095 (reverse).
            If two consecutive readings differ by more than 2048 counts
            (half a revolution), a rollover has occurred.
            Direction of the jump determines whether to add or subtract
            one full revolution worth of counts.
 
        Returns:
            0.000m = piston fully extended  (surface, max buoyancy)
            0.162m = piston fully retracted (maximum depth, min buoyancy)
        """
        raw = self._read_raw_count()
 
        if self._count_prev is not None:
            delta = raw - self._count_prev
            if delta >  self.CPR // 2:
                # Jumped backwards past zero count -- rolled one revolution backward
                self._turn_offset -= self.CPR
            elif delta < -self.CPR // 2:
                # Jumped forwards past 4095 -- rolled one revolution forward
                self._turn_offset += self.CPR
 
        self._count_prev = raw
 
        # Total counts accumulated since calibration zero
        cumulative = (raw + self._turn_offset) - self._zero_count
 
        # Convert: counts -> radians -> linear metres
        angle_rad = (cumulative / self.CPR) * 2 * math.pi
        return angle_rad * self.Ks
 
    def read_velocity(self, Ts):
        """
        Returns filtered linear piston velocity [m/s].
        Differentiates consecutive position readings, then applies
        an exponential low-pass filter to reduce encoder noise.
 
        Must be called exactly once per control loop at interval Ts.
        Calling more or less frequently will give incorrect velocity.
 
        Positive = retracting = sinking
        Negative = extending  = rising
 
        Args:
            Ts : float -- control loop sample period [s] from parameters.py
        """
        pos     = self.read_position()
        vel_raw = (pos - self._pos_prev) / Ts
 
        # Exponential low-pass: alpha=0.9048 at 200Hz gives 20 rad/s cutoff
        # Suppresses high-frequency encoder quantisation noise
        self._vel_f    = self.ALPHA * self._vel_f + (1.0 - self.ALPHA) * vel_raw
        self._pos_prev = pos
        return self._vel_f
 
 
# ================================================================
# MS5837-30BA -- pressure and temperature sensor
# Rated to 30 bar (300m depth) -- well beyond 2m operating range
# ================================================================
MS5837_ADDR       = 0x76
MS5837_RESET      = 0x1E   # software reset command byte
MS5837_PROM_BASE  = 0xA2   # PROM start address for C1 (6 coefficients)
MS5837_CONVERT_D1 = 0x44   # start pressure ADC conversion (OSR=1024)
MS5837_CONVERT_D2 = 0x54   # start temperature ADC conversion (OSR=1024)
MS5837_READ_ADC   = 0x00   # read ADC result
 
class MS5837:
    def __init__(self, i2c, fluid_density=1025.0):
        """
        i2c           : machine.I2C object (shared bus with AS5600)
        fluid_density : kg/m^3
                        1025.0 for seawater (default)
                        1000.0 for fresh water
 
        Depth formula: depth = delta_pressure / (density x g)
        """
        self.i2c  = i2c
        self.rho  = fluid_density
        self.g    = 9.80665
 
        self.C    = [0] * 8    # calibration coefficients C1-C6 (index 1-6)
        self._p0  = None       # surface pressure reference [Pa]
 
        self._reset_and_load()
 
    def _reset_and_load(self):
        """
        Send reset and read factory calibration coefficients from PROM.
        C1-C6 are stored at PROM addresses 0xA2 through 0xAC (2 bytes each).
        These are unique per sensor and used in the compensation formula.
        """
        self.i2c.writeto(MS5837_ADDR, bytes([MS5837_RESET]))
        time.sleep_ms(10)
 
        for i in range(6):
            data = self.i2c.readfrom_mem(
                MS5837_ADDR, MS5837_PROM_BASE + i * 2, 2)
            self.C[i + 1] = (data[0] << 8) | data[1]
 
        print("MS5837: calibration coefficients loaded OK")
 
    def _read_adc(self, cmd):
        """
        Send a conversion command byte, wait for ADC completion,
        then read and return the 24-bit result as an integer.
        OSR=1024 conversion takes 2.5ms -- we wait 3ms for margin.
        """
        self.i2c.writeto(MS5837_ADDR, bytes([cmd]))
        time.sleep_ms(3)
        data = self.i2c.readfrom_mem(MS5837_ADDR, MS5837_READ_ADC, 3)
        return (data[0] << 16) | (data[1] << 8) | data[2]
 
    def _read_pressure_pa(self):
        """
        Execute full MS5837 measurement and compensation sequence.
        Implements the manufacturer datasheet formula including
        second-order low-temperature correction.
 
        Returns:
            pressure_pa : float -- absolute pressure [Pa]
            temp_c      : float -- compensated temperature [degrees C]
        """
        D1 = self._read_adc(MS5837_CONVERT_D1)   # raw pressure ADC
        D2 = self._read_adc(MS5837_CONVERT_D2)   # raw temperature ADC
        C  = self.C
 
        # First-order compensation
        dT   = D2 - C[5] * 2**8
        TEMP = 2000 + dT * C[6] / 2**23          # hundredths of degrees C
 
        OFF  = C[2] * 2**17 + (C[4] * dT) / 2**6
        SENS = C[1] * 2**16 + (C[3] * dT) / 2**7
 
        # Second-order low-temperature correction
        Ti = OFF2 = SENS2 = 0
        if TEMP < 2000:
            Ti    = 3 * dT**2 / 2**33
            OFF2  = 3 * (TEMP - 2000)**2 / 2
            SENS2 = 5 * (TEMP - 2000)**2 / 8
            if TEMP < -1500:
                OFF2  += 7 * (TEMP + 1500)**2
                SENS2 += 4 * (TEMP + 1500)**2
 
        TEMP -= Ti
        OFF  -= OFF2
        SENS -= SENS2
 
        # Compensated pressure: hundredths of mbar x 10 = Pa
        P = (D1 * SENS / 2**21 - OFF) / 2**15
        return P * 10.0, TEMP / 100.0
 
    def calibrate(self, n_samples=10):
        """
        Record the surface atmospheric pressure as the depth zero reference.
 
        MUST be called with the sensor in air before the vehicle enters water.
        If this is skipped, depth readings will be wrong by approximately 10m
        (the equivalent depth of atmospheric pressure ~101325 Pa).
 
        Averages n_samples to reduce noise in the reference value.
        """
        print(f"MS5837: recording surface reference ({n_samples} samples)...")
        readings = []
        for _ in range(n_samples):
            p, _ = self._read_pressure_pa()
            readings.append(p)
            time.sleep_ms(50)
 
        self._p0 = sum(readings) / len(readings)
        print(f"MS5837: surface pressure = {self._p0:.1f} Pa")
 
    def read_depth(self):
        """
        Returns current depth below surface [m].
 
        depth = (P_now - P_surface) / (rho x g)
 
        Clamped to 0.0 -- returns zero if measured above the surface
        reference (can happen due to atmospheric pressure variation).
 
        Requires calibrate() to have been called first.
        """
        if self._p0 is None:
            raise RuntimeError("MS5837: call calibrate() before read_depth()")
        p, _ = self._read_pressure_pa()
        depth = (p - self._p0) / (self.rho * self.g)
        return max(0.0, depth)
 
    def read_temperature(self):
        """Returns water temperature [degrees C] -- useful for mission logging."""
        _, t = self._read_pressure_pa()
        return t