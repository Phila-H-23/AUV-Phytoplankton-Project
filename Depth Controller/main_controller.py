# ================================================================
# main_controller.py
# MicroPython -- Raspberry Pi Pico
# Cascaded depth controller for buoyancy-driven UUV
#
# Architecture:
#   Outer loop: MS5837 depth feedback -> shaft velocity reference
#   Inner loop: AS5600 shaft velocity feedback -> motor voltage
#
# Physical behaviour:
#   Piston retracts (shaft_pos increases) -> water fills chamber
#                                         -> vehicle gets heavier
#                                         -> vehicle sinks (depth increases)
#   Piston extends  (shaft_pos decreases) -> water leaves chamber
#                                         -> vehicle gets lighter
#                                         -> vehicle rises  (depth decreases)
#
# Constraints enforced:
#   1. Stroke limits   -- piston cannot travel beyond 0-16.2cm
#   2. Velocity limit  -- shaft velocity capped at 14.73 mm/s (physical max)
#   3. Current limit   -- voltage clamped to +/-2.142V (3A x Ra) at two points
#   4. Depth fault     -- latching stop if depth exceeds DEPTH_MAX_OP
#   5. Stroke fault    -- latching stop if encoder exceeds stroke limit
#
# Startup sequence (order is critical):
#   1. Ensure piston is fully extended (maximum buoyancy, vehicle at surface)
#   2. Run check_magnet()  -- confirm AS5600 has valid magnet signal
#   3. Run encoder.calibrate() -- set position zero at fully extended position
#   4. Run depth_s.calibrate() -- record atmospheric pressure before water entry
#   5. Set TARGET_DEPTH and run
# ================================================================
 
from machine import I2C, Pin
import time
 
from sensor_interface import AS5600, MS5837
from motor_driver import BTS7960
from parameters import (Ks, Ts, V_SAFE_MAX, V_SAFE_MIN,
                        NUM_INNER, DEN_INNER, K_OUTER,
                        ALPHA_VEL, STROKE_MAX, STROKE_MIN,
                        MAX_STROKE_VELOCITY, STROKE_EQUILIBRIUM,
                        DEPTH_MIN_OP, DEPTH_MAX_OP)
 
# ================================================================
# HARDWARE INITIALISATION
# I2C bus 0: SDA=GP4 (pin 6), SCL=GP5 (pin 7), 400kHz
# Both AS5600 (0x36) and MS5837 (0x76) share this bus
# ================================================================
i2c     = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
encoder = AS5600(i2c, Ks, ALPHA_VEL)
depth_s = MS5837(i2c, fluid_density=1025.0)    # seawater: 1025, fresh: 1000
motor   = BTS7960(v_supply=12.0)
 
# ================================================================
# STARTUP CALIBRATION SEQUENCE
# ================================================================
print("\n--- Startup sequence ---")
print("Ensure piston is FULLY EXTENDED before continuing")
print("(maximum buoyancy -- vehicle should float at surface)")
 
encoder.check_magnet()     # verify AS5600 magnet signal is valid
encoder.calibrate()        # zero shaft position at fully extended
depth_s.calibrate()        # record surface pressure before water entry
 
print(f"\nEquilibrium stroke position: {STROKE_EQUILIBRIUM*100:.1f} cm")
print(f"Control authority: +/-{STROKE_EQUILIBRIUM*100:.1f} cm from equilibrium")
print("Calibration complete\n")
 
# ================================================================
# MISSION SETPOINT
# Change TARGET_DEPTH to set the desired operating depth [m]
# For first hardware test use a small value (e.g. 0.2m)
# ================================================================
TARGET_DEPTH = 0.2          # m -- START SMALL for first test
 
# ================================================================
# CONTROLLER STATE
# Two samples of memory needed for the second-order difference equation
# Must be reset to zero at startup (or on mission restart)
# ================================================================
e_prev1 = 0.0   # inner loop error: e[k-1]
e_prev2 = 0.0   # inner loop error: e[k-2]
u_prev1 = 0.0   # inner loop output: u[k-1]
u_prev2 = 0.0   # inner loop output: u[k-2]
 
# ================================================================
# FAULT FLAG
# Latches True on any safety limit violation
# Motor stops and stays stopped until power cycle
# ================================================================
FAULT = False
 
# ================================================================
# CONTROL STEP FUNCTION
# Implements one iteration of the cascaded controller
# Called every Ts=0.005s (200Hz) from the main loop
# ================================================================
def control_step(target_depth, meas_depth, shaft_pos, shaft_vel):
    """
    One 200Hz iteration of the cascaded depth controller.
 
    Outer loop (slow, 1.0 rad/s bandwidth):
        Computes how fast the piston should move based on depth error.
        Positive depth error (too shallow) -> positive shaft_vel_ref (retract)
        Negative depth error (too deep)    -> negative shaft_vel_ref (extend)
 
    Inner loop (fast, 21.5 rad/s bandwidth):
        Tracks the velocity reference from the outer loop.
        Implements the discrete lead-integrator compensator.
        Outputs a voltage command to the motor driver.
 
    Constraints applied (in order):
        1. Stroke limits  -- zero velocity reference at physical endpoints
        2. Velocity limit -- cap shaft velocity to physical maximum
        3. Current limit  -- clamp voltage to +/-V_SAFE_MAX
 
    Args:
        target_depth : float -- desired depth setpoint [m]
        meas_depth   : float -- current depth from MS5837 [m]
        shaft_pos    : float -- current piston position from AS5600 [m]
        shaft_vel    : float -- current piston velocity from AS5600 [m/s]
 
    Returns:
        voltage      : float -- motor voltage command [V]
        shaft_vel_ref: float -- velocity reference sent to inner loop [m/s]
    """
    global e_prev1, e_prev2, u_prev1, u_prev2
 
    # ---- Outer loop: depth error -> shaft velocity reference ----
    # Positive error = too shallow = need to retract = positive shaft vel
    depth_error   = target_depth - meas_depth
    shaft_vel_ref = K_OUTER * depth_error
 
    # ---- Constraint 1: Stroke limits ----------------------------
    # At fully retracted end: cannot sink further
    if shaft_pos >= STROKE_MAX and shaft_vel_ref > 0:
        shaft_vel_ref = 0.0
    # At fully extended end: cannot rise further
    if shaft_pos <= STROKE_MIN and shaft_vel_ref < 0:
        shaft_vel_ref = 0.0
 
    # ---- Constraint 2: Physical velocity ceiling ----------------
    # Motor cannot physically move piston faster than MAX_STROKE_VELOCITY
    # Clamping prevents integrator windup from demanding impossible speeds
    shaft_vel_ref = max(-MAX_STROKE_VELOCITY,
                        min( MAX_STROKE_VELOCITY, shaft_vel_ref))
 
    # ---- Inner loop: difference equation ------------------------
    # u[k] = DEN[0]*u[k-1] + DEN[1]*u[k-2]
    #       + NUM[0]*e[k] + NUM[1]*e[k-1] + NUM[2]*e[k-2]
    e_k = shaft_vel_ref - shaft_vel
    u_k = (DEN_INNER[0] * u_prev1
         + DEN_INNER[1] * u_prev2
         + NUM_INNER[0] * e_k
         + NUM_INNER[1] * e_prev1
         + NUM_INNER[2] * e_prev2)
 
    # ---- Constraint 3: Current limit (voltage ceiling) ----------
    # V_SAFE_MAX = I_LIMIT x Ra = 3A x 0.714 = 2.142V
    # Any voltage above this would cause motor to exceed 3A
    u_k = max(V_SAFE_MIN, min(V_SAFE_MAX, u_k))
 
    # ---- Shift controller memory --------------------------------
    # Must happen AFTER using the previous values above
    # Order: always update older slot first to avoid overwriting needed value
    e_prev2 = e_prev1;  e_prev1 = e_k
    u_prev2 = u_prev1;  u_prev1 = u_k
 
    return u_k, shaft_vel_ref
 
# ================================================================
# MAIN CONTROL LOOP
# ================================================================
print(f"Controller running")
print(f"  Target depth:      {TARGET_DEPTH} m")
print(f"  Max piston vel:    {MAX_STROKE_VELOCITY*1000:.2f} mm/s")
print(f"  Voltage ceiling:   +/-{V_SAFE_MAX:.4f} V")
print(f"  Sample rate:       {1/Ts:.0f} Hz")
print(f"  Depth fault limit: {DEPTH_MAX_OP} m")
print()
 
loop_count = 0
 
try:
    while not FAULT:
        t0 = time.ticks_ms()
 
        # ---- Read sensors -----------------------------------
        meas_depth = depth_s.read_depth()       # [m] from MS5837
        shaft_pos  = encoder.read_position()    # [m] from AS5600
        shaft_vel  = encoder.read_velocity(Ts)  # [m/s] differentiated + filtered
 
        # ---- Safety fault checks (latching) -----------------
        # Depth fault: vehicle has gone too deep
        if meas_depth > DEPTH_MAX_OP:
            print(f"FAULT: depth {meas_depth:.3f}m exceeds limit {DEPTH_MAX_OP}m")
            FAULT = True
            break
 
        # Stroke fault: encoder reading beyond physical travel
        # 5% tolerance accounts for sensor noise near the endpoint
        if shaft_pos > STROKE_MAX * 1.05:
            print(f"FAULT: shaft {shaft_pos*100:.2f}cm exceeds stroke {STROKE_MAX*100:.1f}cm")
            FAULT = True
            break
 
        # ---- Run cascaded controller ------------------------
        voltage, shaft_vel_ref = control_step(
            TARGET_DEPTH, meas_depth, shaft_pos, shaft_vel)
 
        # ---- Drive motor ------------------------------------
        motor.set_voltage(voltage)
 
        # ---- Log every 10 loops (20Hz printout) -------------
        if loop_count % 10 == 0:
            depth_err  = TARGET_DEPTH - meas_depth
            stroke_pct = (shaft_pos / STROKE_MAX) * 100
 
            # Show direction the controller is commanding
            if shaft_vel_ref > 0.0001:
                cmd_dir = "SINK "
            elif shaft_vel_ref < -0.0001:
                cmd_dir = "RISE "
            else:
                cmd_dir = "HOLD "
 
            print(f"depth:{meas_depth:.3f}m  "
                  f"err:{depth_err:+.3f}m  "
                  f"stroke:{stroke_pct:.1f}%  "
                  f"cmd:{cmd_dir}  "
                  f"vel:{shaft_vel*1000:.3f}mm/s  "
                  f"u:{voltage:.4f}V")
 
        loop_count += 1
 
        # ---- Timing: enforce exactly 200Hz ------------------
        # ticks_diff handles the millisecond counter rollover correctly
        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        delay   = int(Ts * 1000) - elapsed      # remaining ms in this period
        if delay > 0:
            time.sleep_ms(delay)
 
except KeyboardInterrupt:
    print("\nStopped by user (Ctrl+C)")
 
finally:
    # Always executed -- ensures motor stops even if an exception occurs
    motor.close()
    if FAULT:
        print("System halted -- FAULT latched")
        print("Resolve the fault condition then power cycle to restart")
 