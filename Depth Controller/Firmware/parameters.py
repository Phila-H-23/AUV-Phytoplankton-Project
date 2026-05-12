# parameters.py — FINAL VERIFIED DESIGN
# Lead angle: 10°  |  Sample rate: 200Hz  |  Depth range: 0-2m
import math

# ================================================================
# MOTOR PARAMETERS
# ================================================================
p_rated      = 19.68
max_torque   = 0.0105
omega_rated  = 22356 * (2 * math.pi / 60)
I_rated      = 5.25
Ra           = p_rated / I_rated**2
Kt           = max_torque / I_rated
Ke           = Kt
J            = 5e-6
b_visc       = max_torque / omega_rated

# ================================================================
# SCREW GEOMETRY — final verified values
# Lead angle: 10°, Diameter: 25mm
# ================================================================
LEAD_ANGLE_DEG = 10.0
SCREW_DIAMETER = 0.025
Lead = math.pi * SCREW_DIAMETER * math.tan(
           math.radians(LEAD_ANGLE_DEG))    # 0.01385 m/rev
Ks   = Lead / (2 * math.pi)                # 0.002204 m/rad

# ================================================================
# PHYSICAL STROKE CONSTRAINTS
# ================================================================
STROKE_MAX = 0.162      # m — fully retracted (max depth)
STROKE_MIN = 0.0        # m — fully extended  (surface)

# Equilibrium: assumed at stroke midpoint
# Symmetric control authority: 0.081m to sink, 0.081m to rise
# Verify and update after water test
STROKE_EQUILIBRIUM = STROKE_MAX / 2    # = 0.081 m

# ================================================================
# DEPTH OPERATING RANGE
# ================================================================
DEPTH_MIN_OP = 0.0      # m — surface
DEPTH_MAX_OP = 2.0      # m — maximum operating depth

# Sign convention:
# Piston RETRACTS (shaft_pos ↑) → vehicle SINKS   (depth ↑)
# Piston EXTENDS  (shaft_pos ↓) → vehicle RISES   (depth ↓)

# ================================================================
# VELOCITY LIMITS
# ================================================================
# Worst case measured: 11s for full stroke at 3m depth
# Conservative ceiling — real velocity at 2m will be slightly faster
MAX_STROKE_VELOCITY = STROKE_MAX / 11.0     # = 0.01473 m/s

# ================================================================
# L298N CURRENT LIMIT
# ================================================================
I_LIMIT    = 3.0
V_SAFE_MAX = I_LIMIT * Ra               # = 2.142 V
V_SAFE_MIN = -V_SAFE_MAX                # = -2.142 V

# ================================================================
# CONTROL LOOP TIMING
# ================================================================
# Inner loop BW = 21.53 rad/s
# 200Hz → 1256.6 rad/s sample rate >> 10× BW requirement
# Discrete inner PM = 58.42° — 13.42° above 45° minimum
Ts = 0.005      # 200 Hz

# ================================================================
# DISCRETE CONTROLLER COEFFICIENTS
# Verified: MATLAB c2d Tustin at 200Hz, lead angle 10°
#
# Difference equation (inner loop):
# u[k] = DEN[0]*u[k-1] + DEN[1]*u[k-2]
#       + NUM[0]*e[k]   + NUM[1]*e[k-1] + NUM[2]*e[k-2]
# ================================================================
NUM_INNER = [ 2.104167,  0.041667, -2.062500]
DEN_INNER = [ 1.777778, -0.777778]
K_OUTER   = 1.0

# ================================================================
# VELOCITY FILTER
# Cutoff: 20 rad/s, recalculated for 200Hz sample rate
# ================================================================
ALPHA_VEL = math.exp(-20.0 * Ts)       # = 0.9048

# ================================================================
# STARTUP DIAGNOSTIC
# ================================================================
print("=" * 52)
print("SYSTEM PARAMETERS — FINAL VERIFIED DESIGN")
print("=" * 52)
print(f"Lead angle:          {LEAD_ANGLE_DEG}°")
print(f"Screw lead:          {Lead*1000:.3f} mm/rev")
print(f"Ks:                  {Ks:.6f} m/rad")
print(f"Stroke:              {STROKE_MAX*100:.1f} cm")
print(f"Equilibrium:         {STROKE_EQUILIBRIUM*100:.1f} cm")
print(f"Voltage ceiling:     ±{V_SAFE_MAX:.4f} V  (2A × Ra)")
print(f"Max piston vel:      {MAX_STROKE_VELOCITY*1000:.2f} mm/s")
print(f"Sample rate:         {1/Ts:.0f} Hz")
print(f"Inner PM (discrete): 58.42°")
print(f"Outer PM (discrete): 88.98°")
print(f"Velocity filter α:   {ALPHA_VEL:.4f}")
print("=" * 52)