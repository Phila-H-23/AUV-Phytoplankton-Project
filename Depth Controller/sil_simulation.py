"""
sil_simulation.py — Software-in-the-Loop Simulation
Runs on laptop (requires numpy, scipy, matplotlib)
Do NOT upload to the Pico — uses libraries unavailable on MicroPython
 
Validates the discrete cascaded controller before hardware connection.
The controller difference equations here are IDENTICAL to main_controller.py.
The only difference is that read_position()/read_depth() are replaced by
a simulated plant model.
 
Tests:
    1. Clean step response     -- verifies tracking, rise time, overshoot
    2. Disturbance rejection   -- sudden load applied mid-stroke
    3. Sensor noise            -- encoder quantisation + pressure noise
    4. Voltage ceiling effect  -- shows real response under 2.142V limit
 
Motor/screw parameters: lead angle 10 deg, BTS7960 3A current limit
Controller:  200Hz, inner PM 58.42 deg, outer PM 88.98 deg
"""
 
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete, tf2ss
import math
 
# ================================================================
# PARAMETERS -- must match parameters.py exactly
# ================================================================
p_rated     = 19.68
max_torque  = 0.0105
omega_rated = 22356 * (2 * math.pi / 60)
I_rated     = 5.25
 
Ra     = p_rated / I_rated**2
Kt     = max_torque / I_rated
Ke     = Kt
J      = 5e-6
b_visc = max_torque / omega_rated
 
# Screw geometry -- lead angle 10 degrees
LEAD_ANGLE_DEG = 10.0
SCREW_DIAMETER = 0.025
Lead = math.pi * SCREW_DIAMETER * math.tan(math.radians(LEAD_ANGLE_DEG))
Ks   = Lead / (2 * math.pi)
 
# Physical constraints
STROKE_MAX          = 0.162
STROKE_MIN          = 0.0
STROKE_EQUILIBRIUM  = STROKE_MAX / 2      # = 0.081m
DEPTH_MAX_OP        = 2.0
MAX_STROKE_VELOCITY = STROKE_MAX / 11.0   # = 0.01473 m/s
 
# BTS7960 voltage ceiling from 3A current limit
I_LIMIT    = 3.0
V_SAFE_MAX =  I_LIMIT * Ra                # = 2.142V
V_SAFE_MIN = -V_SAFE_MAX
 
# Timing
Ts = 0.005          # 200Hz -- matches main_controller.py
 
# Discrete coefficients -- from MATLAB c2d at 200Hz, lead angle 10 deg
NUM_INNER = [ 2.104167,  0.041667, -2.062500]
DEN_INNER = [ 1.777778, -0.777778]
K_OUTER   = 1.0
 
# Velocity filter
ALPHA_VEL = math.exp(-20.0 * Ts)         # = 0.9048
 
# ================================================================
# DISCRETE PLANT MODEL
# Simplified 2nd order motor + screw (drop electrical time constant)
# G(s) = Kt*Ks / (Ra*J*s^2 + (Ra*b_visc + Kt*Ke)*s)
# voltage [V] -> position [m]
# ================================================================
num_ct = [Kt * Ks]
den_ct = [Ra * J,  Ra * b_visc + Kt * Ke,  0.0]
 
A_c, B_c, C_c, D_c = tf2ss(num_ct, den_ct)
A_d, B_d, C_d, D_d, _ = cont2discrete(
    (A_c, B_c, C_c, D_c), Ts, method='zoh')
 
# ================================================================
# SIMULATION FUNCTION
# Implements the SAME controller logic as main_controller.py
# ================================================================
def simulate(t_end, target_depth,
             pos_noise_std=0.0,
             depth_noise_std=0.0,
             disturbance_time=None,
             disturbance_magnitude=0.0,
             label=""):
    """
    Run one SIL scenario.
 
    The controller code (outer loop, inner loop, difference equation,
    voltage clamp, stroke limits, velocity limit) is identical to
    main_controller.py control_step().
 
    The plant (A_d, B_d, C_d matrices) replaces read_position()
    and read_depth() with a simulated motor response.
 
    Args:
        t_end                : float -- simulation duration [s]
        target_depth         : float -- depth setpoint [m]
        pos_noise_std        : float -- encoder noise std dev [m]
        depth_noise_std      : float -- pressure sensor noise std dev [m]
        disturbance_time     : float -- time to apply load disturbance [s]
        disturbance_magnitude: float -- disturbance as equivalent voltage [V]
        label                : str   -- name for printout
 
    Returns:
        dict with keys: t, shaft_pos, depth_est, shaft_vel, voltage, vel_ref
    """
    N = int(t_end / Ts)
    t = np.arange(N) * Ts
 
    # Plant state (starts at rest, piston fully extended)
    x = np.zeros((A_d.shape[0], 1))
 
    # Controller memory -- same initial conditions as main_controller.py
    e_prev1 = e_prev2 = 0.0
    u_prev1 = u_prev2 = 0.0
 
    # Velocity filter state
    vel_f    = 0.0
    pos_prev = 0.0
 
    # Logs
    pos_log  = np.zeros(N)
    vel_log  = np.zeros(N)
    u_log    = np.zeros(N)
    vref_log = np.zeros(N)
 
    for k in range(N):
 
        # ---- Plant output: true piston position [m] ---------
        shaft_pos_true = float((C_d @ x).flat[0])
 
        # ---- Simulated sensor readings with noise -----------
        shaft_pos_meas = shaft_pos_true + np.random.normal(0.0, pos_noise_std)
 
        # Depth estimate from shaft position (for outer loop feedback)
        # In the real system this comes from MS5837 directly
        # Here we use shaft position as a proxy to drive the plant
        # A separate depth_noise channel models MS5837 noise
        depth_meas = shaft_pos_meas + np.random.normal(0.0, depth_noise_std)
 
        # ---- Velocity estimate: differentiate + filter ------
        vel_raw = (shaft_pos_meas - pos_prev) / Ts
        vel_f   = ALPHA_VEL * vel_f + (1.0 - ALPHA_VEL) * vel_raw
        pos_prev = shaft_pos_meas
 
        # ================================================================
        # CONTROLLER -- identical to main_controller.py control_step()
        # ================================================================
 
        # Outer loop: depth error -> shaft velocity reference
        depth_error   = target_depth - depth_meas
        shaft_vel_ref = K_OUTER * depth_error
 
        # Stroke limits
        if shaft_pos_meas >= STROKE_MAX and shaft_vel_ref > 0:
            shaft_vel_ref = 0.0
        if shaft_pos_meas <= STROKE_MIN and shaft_vel_ref < 0:
            shaft_vel_ref = 0.0
 
        # Velocity ceiling
        shaft_vel_ref = max(-MAX_STROKE_VELOCITY,
                            min( MAX_STROKE_VELOCITY, shaft_vel_ref))
 
        # Inner loop difference equation
        e_k = shaft_vel_ref - vel_f
        u_k = (DEN_INNER[0] * u_prev1
             + DEN_INNER[1] * u_prev2
             + NUM_INNER[0] * e_k
             + NUM_INNER[1] * e_prev1
             + NUM_INNER[2] * e_prev2)
 
        # Voltage ceiling (current limit enforcement)
        u_k = float(np.clip(u_k, V_SAFE_MIN, V_SAFE_MAX))
 
        # Shift controller memory
        e_prev2 = e_prev1;  e_prev1 = e_k
        u_prev2 = u_prev1;  u_prev1 = u_k
 
        # ================================================================
 
        # ---- Load disturbance (equivalent voltage offset) ---
        d = 0.0
        if disturbance_time is not None and t[k] >= disturbance_time:
            d = disturbance_magnitude
 
        # ---- Step plant forward one timestep ----------------
        x = A_d @ x + B_d * (u_k + d)
 
        # ---- Log --------------------------------------------
        pos_log[k]  = shaft_pos_meas
        vel_log[k]  = vel_f
        u_log[k]    = u_k
        vref_log[k] = shaft_vel_ref
 
    return {'t': t, 'shaft_pos': pos_log, 'shaft_vel': vel_log,
            'voltage': u_log, 'vel_ref': vref_log}
 
 
# ================================================================
# METRICS FUNCTION
# ================================================================
def metrics(r, target, label):
    """Print key performance metrics for a simulation result."""
    pos  = r['shaft_pos']
    t    = r['t']
    ref  = target
 
    # Rise time: 10% to 90% of target
    try:
        idx_10 = np.where(pos >= 0.10 * ref)[0][0]
        idx_90 = np.where(pos >= 0.90 * ref)[0][0]
        rise   = t[idx_90] - t[idx_10]
    except IndexError:
        rise = float('nan')
 
    # Overshoot
    overshoot = max(0.0, (np.max(pos) - ref) / ref * 100) if ref > 0 else 0.0
 
    # Settling time (within 2% band)
    within = np.where(np.abs(pos - ref) <= 0.02 * ref)[0]
    settle = t[within[0]] if len(within) > 0 else float('nan')
 
    # Steady-state error (mean of last 10% of simulation)
    ss_error = abs(np.mean(pos[int(0.9 * len(pos)):]) - ref) * 1000  # mm
 
    print(f"\n{label}")
    print(f"  Rise time:       {rise:.2f} s")
    print(f"  Overshoot:       {overshoot:.2f}%")
    print(f"  Settling time:   {settle:.2f} s")
    print(f"  SS error:        {ss_error:.4f} mm  (want < 0.1 mm)")
    print(f"  Peak voltage:    {np.max(np.abs(r['voltage'])):.4f} V  "
          f"(ceiling {V_SAFE_MAX:.4f} V)")
    print(f"  Max velocity:    {np.max(np.abs(r['shaft_vel']))*1000:.4f} mm/s  "
          f"(limit {MAX_STROKE_VELOCITY*1000:.2f} mm/s)")
 
 
# ================================================================
# RUN ALL FOUR TEST SCENARIOS
# ================================================================
# Target: piston moves from 0 (surface) to equilibrium (0.081m)
# In the real system this corresponds to sinking to target depth
target = STROKE_EQUILIBRIUM    # 0.081m = half stroke = assumed neutral buoyancy
 
print("=" * 54)
print("SIL SIMULATION -- CASCADED DEPTH CONTROLLER")
print("=" * 54)
print(f"Target stroke position: {target*100:.1f} cm")
print(f"Voltage ceiling:        +/-{V_SAFE_MAX:.4f} V  (3A x Ra)")
print(f"Max piston velocity:    {MAX_STROKE_VELOCITY*1000:.2f} mm/s")
print(f"Sample rate:            {1/Ts:.0f} Hz")
print()
print("Running scenarios...")
 
# Test 1: Clean step response
r1 = simulate(120.0, target,
              label="1. Clean step response")
print("  [1/4] done")
 
# Test 2: Disturbance rejection
# At t=60s a sustained load disturbance hits (water pressure equivalent)
r2 = simulate(120.0, target,
              disturbance_time=60.0,
              disturbance_magnitude=-0.3,
              label="2. Disturbance rejection")
print("  [2/4] done")
 
# Test 3: Sensor noise
# 0.1mm encoder noise + 0.005m pressure sensor noise (realistic values)
r3 = simulate(120.0, target,
              pos_noise_std=0.0001,
              depth_noise_std=0.005,
              label="3. Sensor noise")
print("  [3/4] done")
 
# Test 4: Combined noise + disturbance (worst case)
r4 = simulate(120.0, target,
              pos_noise_std=0.0001,
              depth_noise_std=0.005,
              disturbance_time=60.0,
              disturbance_magnitude=-0.3,
              label="4. Noise + disturbance combined")
print("  [4/4] done")
 
# ================================================================
# PRINT METRICS
# ================================================================
print("\n" + "=" * 54)
print("RESULTS")
print("=" * 54)
metrics(r1, target, "1. Clean step response")
metrics(r2, target, "2. Disturbance rejection")
metrics(r3, target, "3. Sensor noise")
metrics(r4, target, "4. Noise + disturbance combined")
 
# ================================================================
# PLOTS
# ================================================================
fig, axes = plt.subplots(2, 2, figsize=(14, 9))
fig.suptitle(
    f"SIL Simulation -- Cascaded Depth Controller\n"
    f"BTS7960 3A limit | Voltage ceiling +/-{V_SAFE_MAX:.3f}V | "
    f"200Hz | Lead angle 10 deg",
    fontsize=11)
 
colours = {
    'ref':  '#888888',
    'pos':  '#378ADD',
    'vel':  '#1D9E75',
    'u':    '#EF9F27',
    'dist': '#E24B4A',
    'noise':'#7F77DD'
}
 
def add_voltage_axis(ax, r, colour):
    """Add voltage on a secondary y-axis."""
    ax2 = ax.twinx()
    ax2.plot(r['t'], r['voltage'], color=colour, lw=0.6, alpha=0.4,
             label='Voltage')
    ax2.axhline( V_SAFE_MAX, color=colour, lw=0.5, ls=':', alpha=0.6)
    ax2.axhline(-V_SAFE_MAX, color=colour, lw=0.5, ls=':', alpha=0.6)
    ax2.set_ylabel('Voltage (V)', color=colour, fontsize=9)
    ax2.tick_params(axis='y', labelcolor=colour, labelsize=8)
    ax2.set_ylim(-V_SAFE_MAX * 2, V_SAFE_MAX * 2)
    return ax2
 
# -- Plot 1: Clean step response --
ax = axes[0, 0]
ax.plot(r1['t'], np.ones_like(r1['t']) * target * 1000,
        '--', color=colours['ref'], lw=1, label='Target')
ax.plot(r1['t'], r1['shaft_pos'] * 1000,
        color=colours['pos'], lw=1.5, label='Shaft position')
ax.axhline(target * 1000 * 1.02, color='gray', lw=0.4, ls=':')
ax.axhline(target * 1000 * 0.98, color='gray', lw=0.4, ls=':')
ax.set_title('1 — Clean step response', fontsize=10)
ax.set_xlabel('Time (s)'); ax.set_ylabel('Shaft position (mm)')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
add_voltage_axis(ax, r1, colours['u'])
 
# -- Plot 2: Disturbance rejection --
ax = axes[0, 1]
ax.plot(r2['t'], np.ones_like(r2['t']) * target * 1000,
        '--', color=colours['ref'], lw=1, label='Target')
ax.plot(r2['t'], r2['shaft_pos'] * 1000,
        color=colours['pos'], lw=1.5, label='Shaft position')
ax.axvline(60.0, color=colours['dist'], lw=1.2, ls='--',
           label='Disturbance on')
ax.set_title('2 — Disturbance rejection (load at t=60s)', fontsize=10)
ax.set_xlabel('Time (s)'); ax.set_ylabel('Shaft position (mm)')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
 
# -- Plot 3: Sensor noise --
ax = axes[1, 0]
ax.plot(r3['t'], np.ones_like(r3['t']) * target * 1000,
        '--', color=colours['ref'], lw=1, label='Target')
ax.plot(r3['t'], r3['shaft_pos'] * 1000,
        color=colours['noise'], lw=0.8, alpha=0.8, label='Shaft position (noisy)')
ax.set_title('3 — Sensor noise (encoder 0.1mm, depth 5mm sigma)', fontsize=10)
ax.set_xlabel('Time (s)'); ax.set_ylabel('Shaft position (mm)')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
 
# -- Plot 4: Combined worst case --
ax = axes[1, 1]
ax.plot(r4['t'], np.ones_like(r4['t']) * target * 1000,
        '--', color=colours['ref'], lw=1, label='Target')
ax.plot(r4['t'], r4['shaft_pos'] * 1000,
        color=colours['dist'], lw=1, alpha=0.9, label='Shaft position')
ax.axvline(60.0, color='gray', lw=1, ls='--', label='Disturbance on')
ax.set_title('4 — Noise + disturbance combined (worst case)', fontsize=10)
ax.set_xlabel('Time (s)'); ax.set_ylabel('Shaft position (mm)')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
 
plt.tight_layout()
plt.savefig('sil_results.png', dpi=150, bbox_inches='tight')
plt.show()
print("\nPlot saved to sil_results.png")