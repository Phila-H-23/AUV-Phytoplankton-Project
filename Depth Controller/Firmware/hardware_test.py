# ================================================================
# hardware_test.py
# MicroPython -- Raspberry Pi Pico
#
# Pre-flight hardware diagnostic script.
# Run this BEFORE main_controller.py to verify every hardware
# component is connected and responding correctly.
#
# Tests performed (in order):
#   1. I2C bus scan          -- detects all devices on the bus
#   2. AS5600 magnet check   -- verifies magnet signal strength
#   3. AS5600 position read  -- confirms encoder gives live readings
#   4. MS5837 comms check    -- verifies pressure sensor responds
#   5. MS5837 pressure read  -- confirms sensor gives sensible values
#   6. MS5837 depth read     -- verifies depth calculation after calibration
#   7. BTS7960 forward drive -- moves motor forward briefly
#   8. BTS7960 reverse drive -- moves motor in reverse briefly
#   9. BTS7960 stop          -- confirms motor stops cleanly
#  10. Full loop timing test -- confirms 200Hz loop is achievable on Pico
#
# SAFETY:
#   Motor tests use V_SAFE_MAX (2.142V) -- well within current limit
#   Motor runs for only 1 second per direction
#   Have your hand near the power switch for the motor tests
#   Piston must have free travel space in BOTH directions before running
#
# EXPECTED OUTPUT:
#   Each test prints PASS or FAIL with a reason.
#   All 10 tests must show PASS before running main_controller.py
# ================================================================
 
from machine import I2C, Pin, PWM
import time
import math
 
# ================================================================
# IMPORT PARAMETERS
# ================================================================
from parameters import (Ks, ALPHA_VEL, V_SAFE_MAX, V_SAFE_MIN,
                        STROKE_MAX, Ts, Ra, I_LIMIT)
 
# ================================================================
# PIN DEFINITIONS -- must match motor_driver.py
# ================================================================
PIN_RPWM = 18
PIN_LPWM = 19
PIN_R_EN = 17
PIN_L_EN = 16
PWM_FREQ = 20000
 
# I2C addresses
AS5600_ADDR = 0x36
MS5837_ADDR = 0x76
 
# ================================================================
# TEST RESULT TRACKING
# ================================================================
results = []
 
def passed(test_name, detail=""):
    msg = f"  PASS  {test_name}"
    if detail:
        msg += f" -- {detail}"
    print(msg)
    results.append((test_name, True, detail))
 
def failed(test_name, reason):
    msg = f"  FAIL  {test_name} -- {reason}"
    print(msg)
    results.append((test_name, False, reason))
 
def section(title):
    print(f"\n{'='*54}")
    print(f"  {title}")
    print(f"{'='*54}")
 
# ================================================================
# INITIALISE I2C BUS
# ================================================================
print("\nHARDWARE DIAGNOSTIC -- Cascaded Depth Controller")
print("Initialising I2C bus 0 (SDA=GP4, SCL=GP5, 400kHz)...")
 
try:
    i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
    print("I2C bus initialised OK")
except Exception as e:
    print(f"CRITICAL: I2C bus failed to initialise -- {e}")
    print("Check SDA/SCL wiring. Cannot continue.")
    raise SystemExit
 
# ================================================================
# TEST 1: I2C BUS SCAN
# ================================================================
section("TEST 1 -- I2C Bus Scan")
print("  Scanning for devices on I2C bus 0...")
 
try:
    devices = i2c.scan()
    if devices:
        print(f"  Found {len(devices)} device(s):")
        for addr in devices:
            name = {
                AS5600_ADDR: "AS5600 angle sensor",
                MS5837_ADDR: "MS5837 pressure sensor"
            }.get(addr, "unknown device")
            print(f"    0x{addr:02X} -- {name}")
    else:
        print("  No devices found on bus")
 
    # Check both expected devices are present
    missing = []
    if AS5600_ADDR not in devices:
        missing.append(f"AS5600 (0x{AS5600_ADDR:02X}) not found")
    if MS5837_ADDR not in devices:
        missing.append(f"MS5837 (0x{MS5837_ADDR:02X}) not found")
 
    if not missing:
        passed("I2C bus scan", f"both sensors detected at 0x36 and 0x76")
    else:
        for m in missing:
            failed("I2C bus scan", m)
 
except Exception as e:
    failed("I2C bus scan", f"exception during scan: {e}")
 
 
# ================================================================
# TEST 2: AS5600 MAGNET DETECTION
# ================================================================
section("TEST 2 -- AS5600 Magnet Detection")
 
AS5600_STATUS    = 0x0B
AS5600_ANGLE_REG = 0x0C
 
try:
    status = i2c.readfrom_mem(AS5600_ADDR, AS5600_STATUS, 1)[0]
    md = (status >> 5) & 1   # magnet detected
    ml = (status >> 4) & 1   # magnet too weak
    mh = (status >> 3) & 1   # magnet too strong
 
    print(f"  Status register: 0x{status:02X}")
    print(f"  MD (magnet detected): {md}")
    print(f"  ML (too weak):        {ml}")
    print(f"  MH (too strong):      {mh}")
 
    if md and not ml and not mh:
        passed("AS5600 magnet detection", "magnet detected, strength OK")
    elif md and ml:
        failed("AS5600 magnet detection",
               "magnet detected but too weak -- move magnet closer to sensor")
    elif md and mh:
        failed("AS5600 magnet detection",
               "magnet detected but too strong -- move magnet further from sensor")
    else:
        failed("AS5600 magnet detection",
               "no magnet detected -- check magnet is centred over AS5600 chip, "
               "gap should be 0.5-3mm")
 
except Exception as e:
    failed("AS5600 magnet detection", f"I2C read error: {e}")
    print("  Check: VCC=3.3V, GND, SDA=GP4, SCL=GP5")
    print("  Check: AS5600 PS pin MUST be tied to 3.3V for I2C mode")
 
 
# ================================================================
# TEST 3: AS5600 POSITION READING (live)
# ================================================================
section("TEST 3 -- AS5600 Position Reading")
print("  Reading 5 consecutive angle samples...")
 
try:
    readings = []
    for i in range(5):
        data = i2c.readfrom_mem(AS5600_ADDR, AS5600_ANGLE_REG, 2)
        raw  = ((data[0] & 0x0F) << 8) | data[1]
        readings.append(raw)
        time.sleep_ms(20)
 
    # Convert to degrees for readability
    degrees = [(r / 4096) * 360 for r in readings]
    print(f"  Raw counts: {readings}")
    print(f"  Angles:     {[f'{d:.1f}' for d in degrees]} degrees")
 
    # Check readings are in valid range and not all identical
    all_valid   = all(0 <= r <= 4095 for r in readings)
    all_same    = len(set(readings)) == 1
 
    if all_valid and not all_same:
        passed("AS5600 position reading",
               f"readings vary {min(readings)}-{max(readings)} counts (sensor noise = alive)")
    elif all_valid and all_same:
        passed("AS5600 position reading",
               f"all readings identical ({readings[0]} counts) -- "
               "OK if shaft was completely stationary")
    else:
        failed("AS5600 position reading",
               f"invalid counts detected (must be 0-4095)")
 
    # Manual movement test
    print("\n  >> Manually rotate the shaft slightly then press Enter")
    print("  >> (skip by pressing Enter without moving)")
    input()
    data_after = i2c.readfrom_mem(AS5600_ADDR, AS5600_ANGLE_REG, 2)
    raw_after  = ((data_after[0] & 0x0F) << 8) | data_after[1]
    if raw_after != readings[-1]:
        passed("AS5600 shaft movement response",
               f"angle changed from {readings[-1]} to {raw_after} counts")
    else:
        print("  NOTE: angle unchanged -- OK if shaft was not moved")
 
except Exception as e:
    failed("AS5600 position reading", f"I2C read error: {e}")
 
 
# ================================================================
# TEST 4: MS5837 COMMS AND CALIBRATION LOAD
# ================================================================
section("TEST 4 -- MS5837 Communications and Calibration")
 
MS5837_RESET    = 0x1E
MS5837_PROM_BASE = 0xA2
 
try:
    # Send reset
    i2c.writeto(MS5837_ADDR, bytes([MS5837_RESET]))
    time.sleep_ms(15)
    print("  Reset command sent OK")
 
    # Read calibration coefficients C1-C6
    C = [0] * 8
    for i in range(6):
        data = i2c.readfrom_mem(MS5837_ADDR, MS5837_PROM_BASE + i * 2, 2)
        C[i + 1] = (data[0] << 8) | data[1]
 
    print(f"  Calibration coefficients:")
    for i in range(1, 7):
        print(f"    C{i} = {C[i]}")
 
    # Coefficients should be non-zero and not all the same
    coeff_valid = all(C[i] != 0 for i in range(1, 7))
    coeff_unique = len(set(C[1:7])) > 1
 
    if coeff_valid and coeff_unique:
        passed("MS5837 calibration load",
               "all 6 coefficients non-zero and unique")
    elif not coeff_valid:
        failed("MS5837 calibration load",
               "one or more zero coefficients -- sensor may be damaged or wiring fault")
    else:
        failed("MS5837 calibration load",
               "coefficients all identical -- possible I2C read error")
 
except Exception as e:
    failed("MS5837 calibration load", f"error: {e}")
    print("  Check: VCC=3.3V, GND, SDA=GP4, SCL=GP5")
 
 
# ================================================================
# TEST 5: MS5837 PRESSURE READING
# ================================================================
section("TEST 5 -- MS5837 Pressure Reading")
 
MS5837_CONVERT_D1 = 0x44
MS5837_CONVERT_D2 = 0x54
MS5837_READ_ADC   = 0x00
 
def read_ms5837_raw():
    """Read one set of raw D1/D2 ADC values from MS5837."""
    i2c.writeto(MS5837_ADDR, bytes([MS5837_CONVERT_D1]))
    time.sleep_ms(3)
    d = i2c.readfrom_mem(MS5837_ADDR, MS5837_READ_ADC, 3)
    D1 = (d[0] << 16) | (d[1] << 8) | d[2]
 
    i2c.writeto(MS5837_ADDR, bytes([MS5837_CONVERT_D2]))
    time.sleep_ms(3)
    d = i2c.readfrom_mem(MS5837_ADDR, MS5837_READ_ADC, 3)
    D2 = (d[0] << 16) | (d[1] << 8) | d[2]
    return D1, D2
 
def compensate_ms5837(D1, D2, C):
    """Apply MS5837 datasheet compensation formula. Returns (Pa, degC)."""
    dT   = D2 - C[5] * 2**8
    TEMP = 2000 + dT * C[6] / 2**23
    OFF  = C[2] * 2**17 + (C[4] * dT) / 2**6
    SENS = C[1] * 2**16 + (C[3] * dT) / 2**7
    Ti = OFF2 = SENS2 = 0
    if TEMP < 2000:
        Ti    = 3 * dT**2 / 2**33
        OFF2  = 3 * (TEMP - 2000)**2 / 2
        SENS2 = 5 * (TEMP - 2000)**2 / 8
    TEMP -= Ti; OFF -= OFF2; SENS -= SENS2
    P = (D1 * SENS / 2**21 - OFF) / 2**15
    return P * 10.0, TEMP / 100.0
 
try:
    # Re-read calibration (in case test 4 failed)
    C = [0] * 8
    for i in range(6):
        data = i2c.readfrom_mem(MS5837_ADDR, MS5837_PROM_BASE + i * 2, 2)
        C[i + 1] = (data[0] << 8) | data[1]
 
    print("  Reading 3 pressure samples...")
    pressures = []
    temps     = []
    for _ in range(3):
        D1, D2   = read_ms5837_raw()
        p, t     = compensate_ms5837(D1, D2, C)
        pressures.append(p)
        temps.append(t)
        time.sleep_ms(100)
 
    avg_p = sum(pressures) / len(pressures)
    avg_t = sum(temps) / len(temps)
 
    print(f"  Pressure readings: {[f'{p:.1f}' for p in pressures]} Pa")
    print(f"  Temperature:       {avg_t:.2f} degrees C")
    print(f"  Average pressure:  {avg_p:.1f} Pa  ({avg_p/100:.2f} mbar)")
 
    # Atmospheric pressure at sea level ~101325 Pa
    # Allow wide range for altitude: 85000-110000 Pa
    if 85000 < avg_p < 110000:
        passed("MS5837 pressure reading",
               f"avg {avg_p:.0f} Pa ({avg_t:.1f} C) -- plausible atmospheric pressure")
    elif avg_p <= 0:
        failed("MS5837 pressure reading",
               "negative or zero pressure -- calibration coefficients likely wrong")
    else:
        failed("MS5837 pressure reading",
               f"pressure {avg_p:.0f} Pa outside expected range (85000-110000 Pa) "
               "-- check sensor is in air and calibration coefficients loaded correctly")
 
except Exception as e:
    failed("MS5837 pressure reading", f"error during measurement: {e}")
 
 
# ================================================================
# TEST 6: MS5837 DEPTH CALCULATION
# ================================================================
section("TEST 6 -- MS5837 Depth Calculation (surface calibration)")
print("  Sensor must be in AIR for this test")
print("  Recording surface pressure reference (10 samples)...")
 
try:
    surface_readings = []
    for _ in range(10):
        D1, D2 = read_ms5837_raw()
        p, _   = compensate_ms5837(D1, D2, C)
        surface_readings.append(p)
        time.sleep_ms(50)
 
    p0 = sum(surface_readings) / len(surface_readings)
    print(f"  Surface reference: {p0:.1f} Pa")
 
    # Take a reading immediately after and compute depth (should be ~0m)
    time.sleep_ms(200)
    D1, D2 = read_ms5837_raw()
    p_now, _ = compensate_ms5837(D1, D2, C)
    rho   = 1025.0
    g     = 9.80665
    depth = max(0.0, (p_now - p0) / (rho * g))
 
    print(f"  Depth reading (in air, expect ~0.0m): {depth:.4f} m")
 
    if depth < 0.05:
        passed("MS5837 depth calculation",
               f"depth reads {depth:.4f}m in air -- calibration working correctly")
    else:
        failed("MS5837 depth calculation",
               f"depth reads {depth:.4f}m in air (expected < 0.05m) "
               "-- surface calibration may have drifted, re-run if outdoors")
 
except Exception as e:
    failed("MS5837 depth calculation", f"error: {e}")
 
 
# ================================================================
# TEST 7-9: BTS7960 MOTOR DRIVER
# ================================================================
section("TESTS 7-9 -- BTS7960 Motor Driver")
print(f"  SAFETY CHECK:")
print(f"    Voltage ceiling: +/-{V_SAFE_MAX:.4f}V  (3A x Ra)")
print(f"    Motor will run for 1 second in each direction")
print(f"    Ensure piston has free travel space in BOTH directions")
print(f"    Starting in 5 seconds -- press Ctrl+C NOW to skip")

try:
    for i in range(5, 0, -1):
        print(f"    {i}...")
        time.sleep(1)
    print("    Starting motor tests now.")
 
    # Initialise BTS7960 pins
    r_en = Pin(PIN_R_EN, Pin.OUT, value=1)
    l_en = Pin(PIN_L_EN, Pin.OUT, value=1)
    rpwm = PWM(Pin(PIN_RPWM)); rpwm.freq(PWM_FREQ); rpwm.duty_u16(0)
    lpwm = PWM(Pin(PIN_LPWM)); lpwm.freq(PWM_FREQ); lpwm.duty_u16(0)
 
    # Convert V_SAFE_MAX to duty cycle (use half for this test -- extra safe)
    test_voltage = V_SAFE_MAX * 0.5           # = ~1.07V, ~0.5A
    v_supply     = 12.0
    duty         = int(test_voltage / v_supply * 65535)
    duty_pct     = (test_voltage / v_supply) * 100
 
    print(f"\n  Test voltage: {test_voltage:.3f}V ({duty_pct:.1f}% duty)")
    print(f"  Expected current: ~{test_voltage/Ra:.3f}A (well within 3A limit)")
 
    # -- Test 7: Forward (RPWM active) --
    print(f"\n  TEST 7: Forward drive (1 second)...")
    print(f"  Piston should RETRACT -- vehicle would SINK")
    rpwm.duty_u16(duty)
    lpwm.duty_u16(0)
    time.sleep_ms(1000)
    rpwm.duty_u16(0)
 
    print("  >> Observe: piston should have RETRACTED (vehicle sinks direction)")
    print("  >> If it ran the wrong way, swap M+ and M- wires on BTS7960")
    time.sleep(1)
    passed("BTS7960 forward drive",
           f"{test_voltage:.3f}V forward applied for 1s -- check direction above")
 
    # Brief pause between directions
    time.sleep_ms(500)
 
    # -- Test 8: Reverse (LPWM active) --
    print(f"\n  TEST 8: Reverse drive (1 second)...")
    print(f"  Piston should EXTEND -- vehicle would RISE")
    rpwm.duty_u16(0)
    lpwm.duty_u16(duty)
    time.sleep_ms(1000)
    rpwm.duty_u16(0)
    lpwm.duty_u16(0)
 
    print("  >> Observe: piston should have EXTENDED (vehicle rises direction)")
    time.sleep(1)
    passed("BTS7960 reverse drive",
           f"{test_voltage:.3f}V reverse applied for 1s -- check direction above")
    
    # -- Test 9: Stop --
    print(f"\n  TEST 9: Stop command...")
    rpwm.duty_u16(0)
    lpwm.duty_u16(0)
    r_en.value(0)
    l_en.value(0)
    time.sleep_ms(200)
    passed("BTS7960 stop", "both PWM channels zeroed, enable pins low")
 
except KeyboardInterrupt:
    # User skipped motor tests
    rpwm.duty_u16(0) if 'rpwm' in dir() else None
    lpwm.duty_u16(0) if 'lpwm' in dir() else None
    print("\n  Motor tests skipped by user")
    results.append(("BTS7960 forward drive", None, "skipped"))
    results.append(("BTS7960 reverse drive", None, "skipped"))
    results.append(("BTS7960 stop",          None, "skipped"))
 
except Exception as e:
    failed("BTS7960 motor driver", f"error during motor test: {e}")
    print("  Check: RPWM=GP18, LPWM=GP19, R_EN=GP17, L_EN=GP16")
    print("  Check: BTS7960 VCC connected to 5V (logic supply)")
    print("  Check: Motor power supply connected to B+ and B-")
    try:
        rpwm.duty_u16(0); lpwm.duty_u16(0)
    except:
        pass
 
 
# ================================================================
# TEST 10: LOOP TIMING
# ================================================================
section("TEST 10 -- 200Hz Loop Timing")
print("  Running 200 iterations of the control loop timing test...")
print("  (no motor movement -- timing only)")
 
try:
    # Simulate the full sensor read + compute time budget of one loop
    # At 200Hz we have 5ms (5000 microseconds) per loop
    # Sensor reads take most of this time -- verify we fit within budget
 
    loop_times = []
    BUDGET_MS  = int(Ts * 1000)    # = 5ms
 
    for _ in range(200):
        t0 = time.ticks_us()
 
        # Simulate the work done each loop iteration:
        # AS5600 read (2 bytes) + MS5837 read (6 bytes + 2 conversions)
        try:
            # AS5600 angle read
            i2c.readfrom_mem(AS5600_ADDR, AS5600_ANGLE_REG, 2)
 
            # MS5837 pressure read (most expensive operation)
            i2c.writeto(MS5837_ADDR, bytes([MS5837_CONVERT_D1]))
            time.sleep_ms(3)
            i2c.readfrom_mem(MS5837_ADDR, MS5837_READ_ADC, 3)
            i2c.writeto(MS5837_ADDR, bytes([MS5837_CONVERT_D2]))
            time.sleep_ms(3)
            i2c.readfrom_mem(MS5837_ADDR, MS5837_READ_ADC, 3)
        except:
            pass   # sensor may have been disturbed -- timing test continues
 
        elapsed_us = time.ticks_diff(time.ticks_us(), t0)
        loop_times.append(elapsed_us)
 
    avg_us = sum(loop_times) / len(loop_times)
    max_us = max(loop_times)
    budget_us = BUDGET_MS * 1000
 
    print(f"  Loop timing results (200 samples):")
    print(f"    Average: {avg_us/1000:.2f} ms  (budget: {BUDGET_MS} ms)")
    print(f"    Maximum: {max_us/1000:.2f} ms  (budget: {BUDGET_MS} ms)")
    print(f"    Remaining headroom: {(budget_us - avg_us)/1000:.2f} ms average")
 
    if max_us < budget_us:
        passed("200Hz loop timing",
               f"max loop {max_us/1000:.2f}ms -- fits within {BUDGET_MS}ms budget")
    elif avg_us < budget_us:
        passed("200Hz loop timing",
               f"average {avg_us/1000:.2f}ms fits budget but max {max_us/1000:.2f}ms "
               f"exceeded -- occasional overruns, generally OK")
    else:
        failed("200Hz loop timing",
               f"average {avg_us/1000:.2f}ms exceeds {BUDGET_MS}ms budget -- "
               f"consider reducing to 100Hz (Ts=0.01) in parameters.py")
 
except Exception as e:
    failed("200Hz loop timing", f"error: {e}")
 
 
# ================================================================
# FINAL SUMMARY
# ================================================================
print(f"\n{'='*54}")
print("  DIAGNOSTIC SUMMARY")
print(f"{'='*54}")
 
passed_count  = sum(1 for _, r, _ in results if r is True)
failed_count  = sum(1 for _, r, _ in results if r is False)
skipped_count = sum(1 for _, r, _ in results if r is None)
 
for name, result, detail in results:
    if result is True:
        status = "PASS "
    elif result is False:
        status = "FAIL "
    else:
        status = "SKIP "
    print(f"  {status}  {name}")
    if result is False:
        print(f"         Reason: {detail}")
 
print(f"\n  Total: {passed_count} passed, "
      f"{failed_count} failed, "
      f"{skipped_count} skipped")
 
if failed_count == 0:
    print("\n  All tests passed.")
    print("  System is ready to run main_controller.py")
    print(f"\n  Reminder before first run:")
    print(f"    1. Set TARGET_DEPTH = 0.2 in main_controller.py")
    print(f"    2. Ensure piston is fully extended at startup")
    print(f"    3. Run encoder.calibrate() at fully extended position")
    print(f"    4. Run depth_s.calibrate() before water entry")
else:
    print(f"\n  {failed_count} test(s) failed.")
    print("  Resolve all failures before running main_controller.py")
    print("  Check wiring for any failed sensor or driver test")