"""
sil_simulation.py -- Software-in-the-Loop Simulation
Runs on laptop. 
 
Validates outer loop (depth control) behaviour.
Inner loop verified analytically in MATLAB -- discrete coefficients used directly.
 
Motor model: first-order velocity model saturating at MAX_STROKE_VELOCITY.
This correctly captures the slow shaft velocity (limited by water pressure at
operating depth) without requiring a load model we can't parameterise.
 
Buoyancy physics:
    depth_acc = BUOY*(shaft - EQ) - DRAG*depth_rate
    BUOY=1.5 m/s^2/m, DRAG=5.0 /s (3kg vehicle, rho=1025, strongly overdamped)
 
Depth rate filtering:
    Raw depth_rate from differencing MS5837 readings is noisy at 200Hz.
    A low-pass filter (cutoff 2 rad/s) is applied before the K_D term.
    This same filter is implemented in main_controller.py.
"""
 
import numpy as np
import matplotlib.pyplot as plt
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
 
LEAD_ANGLE_DEG = 10.0
SCREW_DIAMETER = 0.025
Lead = math.pi * SCREW_DIAMETER * math.tan(math.radians(LEAD_ANGLE_DEG))
Ks   = Lead / (2 * math.pi)
 
STROKE_MAX          = 0.162
STROKE_MIN          = 0.0
STROKE_EQUILIBRIUM  = STROKE_MAX / 2       # 0.081m neutral buoyancy
DEPTH_MAX_OP        = 2.0
MAX_STROKE_VELOCITY = STROKE_MAX / 11.0    # 0.01473 m/s (measured at 3m)
 
I_LIMIT    = 3.0
V_SAFE_MAX =  I_LIMIT * Ra                 # 2.142V
V_SAFE_MIN = -V_SAFE_MAX
 
Ts = 0.005    # 200Hz
 
NUM_INNER = [ 2.104167,  0.041667, -2.062500]
DEN_INNER = [ 1.777778, -0.777778]
 
K_OUTER = 0.1
K_D     = 4.3
 
ALPHA_VEL = math.exp(-20.0 * Ts)          # shaft velocity filter (20 rad/s)
ALPHA_DR  = math.exp(-2.0  * Ts)          # depth rate filter     (2 rad/s)
                                            # = 0.9900 at 200Hz
                                            # Filters out MS5837 differentiation noise
                                            # while still tracking real depth rate changes
 
TAU_SHAFT = 1.5    # shaft first-order time constant [s]
BUOY      = 1.5    # buoyancy gain [m/s^2 per m shaft deviation]
DRAG      = 5.0    # water drag [/s]
 
 
# ================================================================
# SIMULATION FUNCTION
# ================================================================
def simulate(t_end, target_depth,
             encoder_noise_std=0.0,
             depth_noise_std=0.0,
             disturbance_time=None,
             disturbance_fraction=0.0):
    """
    Args:
        t_end               : float -- duration [s]
        target_depth        : float -- depth setpoint [m]
        encoder_noise_std   : float -- AS5600 std dev [m]  realistic: 0.0001m
        depth_noise_std     : float -- MS5837 std dev [m]  realistic: 0.001m
        disturbance_time    : float -- time to inject load disturbance
        disturbance_fraction: float -- disturbance as fraction of V_SAFE_MAX
    """
    N = int(t_end / Ts)
    t = np.arange(N) * Ts
 
    shaft_pos  = STROKE_EQUILIBRIUM
    shaft_vel  = 0.0
    depth      = 0.0
    depth_rate = 0.0
 
    e_prev1 = e_prev2 = 0.0
    u_prev1 = u_prev2 = 0.0
 
    depth_prev      = 0.0
    depth_rate_filt = 0.0    # filtered depth rate for K_D term
 
    vel_f    = 0.0
    pos_prev = STROKE_EQUILIBRIUM
 
    depth_log      = np.zeros(N)    # noisy MS5837 measurement
    true_depth_log = np.zeros(N)    # true underlying depth (smooth)
    shaft_log      = np.zeros(N)
    vel_log        = np.zeros(N)
    voltage_log    = np.zeros(N)
    velref_log     = np.zeros(N)
 
    for k in range(N):
 
        # ---- AS5600: shaft position measurement -------------------------
        shaft_pos_meas = shaft_pos + np.random.normal(0.0, encoder_noise_std)
        shaft_pos_meas = max(STROKE_MIN, min(STROKE_MAX, shaft_pos_meas))
 
        # ---- Shaft velocity: differentiate + 20 rad/s filter ------------
        vel_raw = (shaft_pos_meas - pos_prev) / Ts
        vel_f   = ALPHA_VEL * vel_f + (1.0 - ALPHA_VEL) * vel_raw
        pos_prev = shaft_pos_meas
 
        # ---- Buoyancy physics -------------------------------------------
        depth_acc   = BUOY * (shaft_pos - STROKE_EQUILIBRIUM) - DRAG * depth_rate
        depth_rate += depth_acc * Ts
        depth_rate  = max(-0.05, min(0.05, depth_rate))
        depth       = max(0.0, min(DEPTH_MAX_OP, depth + depth_rate * Ts))
 
        # ---- MS5837: depth measurement ----------------------------------
        depth_meas = max(0.0, depth + np.random.normal(0.0, depth_noise_std))
 
        # ================================================================
        # CONTROLLER -- identical to main_controller.py control_step()
        # ================================================================
 
        # Depth rate: differentiate MS5837 readings + 2 rad/s low-pass filter
        # The low-pass filter is essential -- raw differentiation at 200Hz
        # amplifies sensor noise to unusable levels without it.
        depth_rate_raw  = (depth_meas - depth_prev) / Ts
        depth_rate_filt = ALPHA_DR * depth_rate_filt \
                        + (1.0 - ALPHA_DR) * depth_rate_raw
        depth_prev      = depth_meas
 
        # Outer loop PD
        depth_error   = target_depth - depth_meas
        shaft_vel_ref = K_OUTER * depth_error - K_D * depth_rate_filt
 
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
        u_k = float(np.clip(u_k, V_SAFE_MIN, V_SAFE_MAX))
 
        e_prev2 = e_prev1;  e_prev1 = e_k
        u_prev2 = u_prev1;  u_prev1 = u_k
 
        # ================================================================
 
        # Disturbance
        d = 0.0
        if disturbance_time is not None and t[k] >= disturbance_time:
            d = -disturbance_fraction * V_SAFE_MAX
 
        # First-order shaft velocity model (water-pressure limited)
        net_v = u_k + d
        v_cmd = (net_v / V_SAFE_MAX) * MAX_STROKE_VELOCITY
        v_cmd = max(-MAX_STROKE_VELOCITY, min(MAX_STROKE_VELOCITY, v_cmd))
        shaft_vel += (Ts / TAU_SHAFT) * (v_cmd - shaft_vel)
        shaft_vel  = max(-MAX_STROKE_VELOCITY, min(MAX_STROKE_VELOCITY, shaft_vel))
        shaft_pos  = max(STROKE_MIN, min(STROKE_MAX, shaft_pos + shaft_vel * Ts))
 
        depth_log[k]        = depth_meas    # noisy -- what controller sees
        true_depth_log[k]   = depth         # true -- smooth underlying depth
        shaft_log[k]        = shaft_pos * 1000
        vel_log[k]          = vel_f
        voltage_log[k]      = u_k
        velref_log[k]       = shaft_vel_ref
 
    return {'t': t, 'depth': true_depth_log, 'depth_meas': depth_log,
            'shaft_mm': shaft_log, 'shaft_vel': vel_log,
            'voltage': voltage_log, 'vel_ref': velref_log}
 
 
# ================================================================
# METRICS
# ================================================================
def metrics(r, target_depth, label):
    dep = r['depth']     # true depth -- smooth, used for metrics
    t   = r['t']
    try:
        i10  = np.where(dep >= 0.10 * target_depth)[0][0]
        i90  = np.where(dep >= 0.90 * target_depth)[0][0]
        rise = t[i90] - t[i10]
    except IndexError:
        rise = float('nan')
 
    overshoot = max(0.0, (np.max(dep) - target_depth) / target_depth * 100)
    within    = np.where(np.abs(dep - target_depth) <= 0.02 * target_depth)[0]
    settle    = t[within[0]] if len(within) > 0 else float('nan')
    ss_err    = abs(np.mean(dep[int(0.9*len(dep)):]) - target_depth)
 
    print(f"\n{label}")
    print(f"  Rise time:     {rise:.1f} s")
    print(f"  Overshoot:     {overshoot:.2f}%  (want 0%)")
    print(f"  Settling time: {settle:.1f} s")
    print(f"  SS error:      {ss_err*1000:.1f} mm  (want < 20 mm)")
    print(f"  Peak voltage:  {np.max(np.abs(r['voltage'])):.4f} V"
          f"  (ceiling {V_SAFE_MAX:.4f} V)")
    print(f"  Max shaft vel: {np.max(np.abs(r['shaft_vel']))*1000:.2f} mm/s"
          f"  (limit {MAX_STROKE_VELOCITY*1000:.2f} mm/s)")
 
 
# ================================================================
# RUN
# ================================================================
TARGET = 1.0   # metres
 
print("=" * 54)
print("SIL SIMULATION -- CASCADED DEPTH CONTROLLER")
print("=" * 54)
print(f"Target:       {TARGET} m")
print(f"K_OUTER:      {K_OUTER}   K_D: {K_D}")
print(f"Depth rate filter: {ALPHA_DR:.4f} (2 rad/s cutoff)")
print()
 
r1 = simulate(300.0, TARGET)
print("[1/4] clean step done")
r2 = simulate(300.0, TARGET, disturbance_time=150.0, disturbance_fraction=0.3)
print("[2/4] disturbance done")
r3 = simulate(300.0, TARGET, encoder_noise_std=0.0001, depth_noise_std=0.001)
print("[3/4] sensor noise done  (encoder 0.1mm, depth 1mm -- realistic MS5837)")
r4 = simulate(300.0, TARGET, encoder_noise_std=0.0001, depth_noise_std=0.001,
              disturbance_time=150.0, disturbance_fraction=0.3)
print("[4/4] combined done")
 
print("\n" + "=" * 54)
print("RESULTS")
print("=" * 54)
metrics(r1, TARGET, "1. Clean step response")
metrics(r2, TARGET, "2. Disturbance rejection")
metrics(r3, TARGET, "3. Sensor noise")
metrics(r4, TARGET, "4. Noise + disturbance combined")
 
# Plots
fig, axes = plt.subplots(2, 2, figsize=(14, 9))
fig.suptitle(
    f"SIL Simulation -- Cascaded Depth Controller\n"
    f"K_outer={K_OUTER}  K_D={K_D}  Depth rate filter: 2 rad/s  "
    f"BTS7960 3A  +/-{V_SAFE_MAX:.3f}V  200Hz",
    fontsize=10)
 
for ax, r, col, ttl, dt in zip(
    axes.flat,
    [r1, r2, r3, r4],
    ['#378ADD','#378ADD','#7F77DD','#D85A30'],
    ['1 -- Clean step response',
     '2 -- Disturbance rejection (t=150s)',
     '3 -- Sensor noise (encoder 0.1mm, depth 1mm)',
     '4 -- Noise + disturbance combined'],
    [None, 150.0, None, 150.0]
):
    t = r['t']
    # Noisy measurement as faint background (what the controller sees)
    ax.plot(t, r['depth_meas'], color=col, lw=0.6, alpha=0.25,
            label='Measured (noisy)')
    # True depth as bold main line
    ax.plot(t, r['depth'], color=col, lw=2.0, label='True depth')
    ax.plot(t, np.ones_like(t)*TARGET, '--', color='#888888', lw=1,
            label='Target')
    ax.axhline(TARGET*1.02, color='gray', lw=0.4, ls=':')
    ax.axhline(TARGET*0.98, color='gray', lw=0.4, ls=':')
    if dt:
        ax.axvline(dt, color='#e74c3c', lw=1.2, ls='--', label='Disturbance')
    ax.set_title(ttl, fontsize=10)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Depth (m)')
    ax.set_ylim(-0.05, DEPTH_MAX_OP * 0.75)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
 
    ax2 = ax.twinx()
    ax2.plot(t, r['shaft_mm'], color='gray', lw=0.6, alpha=0.4)
    ax2.axhline(STROKE_EQUILIBRIUM*1000, color='gray', lw=0.5, ls=':',
                alpha=0.5)
    ax2.set_ylabel('Shaft (mm)', color='gray', fontsize=8)
    ax2.tick_params(axis='y', labelcolor='gray', labelsize=7)
    ax2.set_ylim(0, STROKE_MAX*1000*1.3)
 
plt.tight_layout()
plt.savefig('/mnt/user-data/outputs/sil_results.png', dpi=150,
            bbox_inches='tight')
plt.show()
print("\nPlot saved.")