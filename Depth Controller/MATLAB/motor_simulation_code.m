% Motor parameters 
%given parameters
p_rated = 19.68; %Watts
max_torque = 0.0105; %Nm
omega = 22356*(2*pi/60); %rad/s
I_rated = 5.25; % Amps
weight = 0.069; % kg
length = 0.038; %excl. shaft
diameter = 0.0277; % metres
shaft_diameter = 0.0023; % metres
shaft_length = 0.01715; % metres
lead_angle = 10; % degrees
screw_diameter = 0.025; % meters

% calculations
R = p_rated/(I_rated)^2;        % Ohms
L = 0.001;    % Henry
Kt = max_torque/I_rated;     % Nm/A
Ke = Kt;     % V/(rad/s)
J = 5*10^(-6);     % kg.m^2
b = max_torque/omega;    % viscous friction

% Screw parameters
Lead = pi * screw_diameter * tan(lead_angle * pi/180);
Ks   = Lead / (2*pi);

% current limit constraint
I_limit = 2 % L298N continuous limit [A]
V_limit = I_limit * R % max. safe voltage

% physical stroke
stroke = 0.162; % metres
max_velocity = stroke /11.0 % from 3m depth measurement

% Transfer function variable
s = tf('s');

% Motor transfer function (velocity output)
G_motor = Kt / ((L*s + R)*(J*s + b) + Kt*Ke);

% Convert angular velocity → linear velocity
G_linear = Ks * G_motor;

% Convert velocity → position
G_position = G_linear * (1/s);

% =============================
% SIMULATION
% =============================
%figure;
%step(G_position)
%title('Actuator Position Response')
%grid on;

%figure;
%bode(G_position)
%title('Bode Plot of Actuator')
%grid on;

%% validating plant model

[Gm, Pm, Wcg, Wcp] = margin(G_position);
fprintf('Gain margin: %.2f dB at %.4f rad/s\n', 20*log10(Gm), Wcg);
fprintf('Phase margin: %.2f deg at %.4f rad/s\n', Pm, Wcp);


% check the poles to confirm stability
p = pole(G_position);
disp('Open-loop poles:'); disp(p)

% Check the DC velocity gain (slope of step response)
G_velocity = G_linear;  % without the 1/s
dc_vel = dcgain(G_velocity);
fprintf('Linear velocity per volt: %.4f m/s/V\n', dc_vel);

% Electrical time constant (should be << mechanical)
tau_e = L/R;
fprintf('Electrical time constant: %.4f s\n', tau_e);  % expect ~1.4 ms

% Mechanical time constant
tau_m = J/b;
fprintf('Mechanical time constant: %.4f s\n', tau_m);  % expect ~1.1 s

% Back-EMF check: at rated speed, Ke*omega should ≈ V_supply - I*R
V_bemf = Ke * omega;
V_resistive = I_rated * R;
fprintf('Back-EMF at rated speed: %.2f V\n', V_bemf);
fprintf('Resistive drop: %.2f V\n', V_resistive);
fprintf('Implied supply voltage: %.2f V\n', V_bemf + V_resistive);

%% simplifying model

% Simplified 2nd-order model (drop L*s term)
% Motor velocity: G_motor_simple = Kt / (R*(J*s + b) + Kt*Ke)
G_motor_simple = Kt / (R*(J*s + b) + Kt*Ke);
G_position_simple = G_motor_simple * Ks * (1/s);

% Compare step responses to confirm simplification is valid
%figure;
%step(G_position, G_position_simple);
%legend('Full 3rd order', 'Simplified 2nd order');
%title('Model simplification check');

%% inner velocity loop

% Interactive tuning — used this to find gains visually

% ============================================================
% INNER VELOCITY LOOP — lead compensator (sisotool-matched)
% ============================================================
G_motor_simple = Kt / (R*(J*s + b) + Kt*Ke);
G_inner = G_motor_simple * Ks;
%sisotool(G_inner)


% Compensator exactly as sisotool shows it
z_inner     = 4;
p_inner     = 50;
K_sisotool  = 75;
K_inner     = K_sisotool * (p_inner / z_inner);  % = 937.5

C_inner = K_inner * (s + z_inner) / (s * (s + p_inner));

% Verify — should match sisotool exactly
L_inner = C_inner * G_inner;
[Gm_i, Pm_i, ~, Wcp_i] = margin(L_inner);
T_inner = feedback(L_inner, 1);

fprintf('--- Inner loop verification ---\n')
fprintf('GM:        Inf dB (expected)\n')
fprintf('PM:        %.2f deg  (sisotool: 68.5 deg)\n', Pm_i)
fprintf('Bandwidth: %.4f rad/s (sisotool: 8.81 rad/s)\n', Wcp_i)
fprintf('DC gain:   %.4f (must be 1.0000)\n', dcgain(T_inner))

%% ============================================================
% OUTER POSITION LOOP — plant construction
% ============================================================

% Outer plant = closed inner loop × position integrator
G_outer = T_inner * (1/s);
%sisotool(G_outer)

% Verify outer plant
fprintf('--- Outer plant ---\n')
fprintf('Poles: '); disp(pole(G_outer)')

% DC gain will be Inf (integrator present) — check mid-frequency behaviour
[mag, ~, w] = bode(G_outer, 1.0);   % magnitude at 1 rad/s
fprintf('|G_outer| at 1 rad/s: %.4f\n', mag)

figure; bode(G_outer);
title('Outer loop plant — Bode'); grid on;

% Compensator — pure gain, no poles/zeros to convert
K_outer = 1;
C_outer = tf(K_outer, 1);   % wrap scalar gain as a proper tf objec

% Open loop
L_outer = C_outer * G_outer;

% Verify margins match sisotool
[Gm_o, Pm_o, ~, Wcp_o] = margin(L_outer);
fprintf('--- Outer loop verification ---\n')
fprintf('GM:        %.2f dB  (sisotool: 31.4 dB)\n', 20*log10(Gm_o))
fprintf('PM:        %.2f deg (sisotool: 86.3 deg)\n', Pm_o)
fprintf('Bandwidth: %.4f rad/s (sisotool: 1.01 rad/s)\n', Wcp_o)

% Close the outer loop
T_outer = feedback(L_outer, 1);

% DC gain must be 1.0
fprintf('DC gain:   %.4f (must be 1.0000)\n', dcgain(T_outer))

% Step response — use a realistic position target
figure;
step(T_outer * 0.10);
title('Full cascaded position step response — 10cm stroke test');
ylabel('Position (m)'); grid on;

info_o = stepinfo(T_outer);
fprintf('Rise time:     %.3f s\n', info_o.RiseTime)
fprintf('Overshoot:     %.2f%%\n', info_o.Overshoot)
fprintf('Settling time: %.3f s\n', info_o.SettlingTime)

% Bandwidth separation check
fprintf('\n--- Cascade separation check ---\n')
fprintf('Inner BW: %.4f rad/s\n', Wcp_i)
fprintf('Outer BW: %.4f rad/s\n', Wcp_o)
fprintf('Ratio:    %.1f:1 (must be > 5:1)\n', Wcp_i/Wcp_o)

%% ============================================================
% SENSITIVITY ANALYSIS
% ============================================================

% Sensitivity function S — disturbance rejection
% How well does the controller reject load/friction disturbances?
S_outer = feedback(1, L_outer);

% Complementary sensitivity T — noise bandwidth
% How much sensor noise passes through to the actuator?
T_comp = feedback(L_outer, 1);

% Input sensitivity (effort) — how hard is the controller working?
CS = feedback(C_outer, G_outer);

figure;
bodemag(S_outer, T_comp, {0.001, 100});
legend('S — disturbance rejection', 'T — noise bandwidth');
title('Sensitivity functions — outer loop');
grid on;

% Key metrics
[Ms, w_Ms] = getPeakGain(S_outer);
[Mt, w_Mt] = getPeakGain(T_comp);
fprintf('--- Sensitivity analysis ---\n')
fprintf('Peak |S|: %.2f dB at %.4f rad/s (want < 6 dB)\n', ...
        20*log10(Ms), w_Ms)
fprintf('Peak |T|: %.2f dB at %.4f rad/s\n', ...
        20*log10(Mt), w_Mt)

% Disturbance step test — simulates a sudden load force
% e.g. water weight suddenly applied mid-stroke
figure;
t_dist = linspace(0, 20, 2000);
dist_response = lsim(S_outer, ones(size(t_dist)), t_dist);
plot(t_dist, dist_response); grid on;
title('Output disturbance rejection — step load input');
xlabel('Time (s)'); ylabel('Position error (m)');
% This shows how far position drifts when a constant disturbance hits,
% and how quickly the integrator drives it back to 

%% ============================================================
% DISCRETISATION — convert to discrete time for Raspberry Pi
% ============================================================

% Sample rate selection
% Rule: Ts < 1/(10 * outer_bandwidth) for outer loop
Ts = 0.005;   % 200 Hz — required for 30.29 rad/s inner loop bandwidth
fprintf('\n--- Discretisation at %.0f Hz ---\n', 1/Ts)

% Discretise both compensators using Tustin (bilinear) method
% Tustin preserves frequency-domain behaviour best for PID-type controllers
C_inner_d = c2d(C_inner, Ts, 'tustin');
C_outer_d = c2d(C_outer, Ts, 'tustin');   % pure gain — trivial

% Discretise plant for verification
G_inner_d = c2d(G_inner, Ts, 'zoh');
G_outer_d = c2d(G_outer, Ts, 'zoh');

% Verify discrete margins still hold
L_inner_d = C_inner_d * G_inner_d;
L_outer_d = C_outer_d * G_outer_d;

[~, Pm_id, ~, Wcp_id] = margin(L_inner_d);
[Gm_od, Pm_od, ~, Wcp_od] = margin(L_outer_d);

fprintf('Discrete inner PM: %.2f deg (continuous: 55.08)\n', Pm_id)
fprintf('Discrete outer PM: %.2f deg (continuous: 89.13)\n', Pm_od)
fprintf('Discrete outer GM: %.2f dB  (continuous: 33.13)\n', 20*log10(Gm_od))
fprintf('Discrete outer BW: %.4f rad/s (continuous: 1.0035)\n', Wcp_od)

% Extract difference equation coefficients for Python
[num_ci, den_ci] = tfdata(C_inner_d, 'v');
[num_co, den_co] = tfdata(C_outer_d, 'v');

fprintf('\n--- Inner compensator discrete coefficients ---\n')
fprintf('Numerator:   [%.6f  %.6f  %.6f]\n', num_ci(1), num_ci(2), num_ci(3))
fprintf('Denominator: [%.6f  %.6f  %.6f]\n', den_ci(1), den_ci(2), den_ci(3))

fprintf('\n--- Outer compensator discrete coefficients ---\n')
fprintf('Numerator:   [%.6f]\n', num_co(1))
fprintf('Denominator: [%.6f]\n', den_co(1))

% Compare continuous vs discrete step responses
t_cont = linspace(0, 15, 1000);        % 15 second window
t_disc = 0:Ts:15;                       % discrete time vector at 200 Hz

T_inner_d = feedback(L_inner_d, 1);
T_outer_d = feedback(L_outer_d, 1);

% Continuous response
[y_cont, t_cont_out] = step(T_outer * 0.10, t_cont);

% Discrete response
[y_disc, t_disc_out] = step(T_outer_d * 0.10, t_disc);

figure;
plot(t_cont_out, y_cont, 'b-',  'LineWidth', 1.5); hold on;
stairs(t_disc_out, y_disc, 'r--', 'LineWidth', 1.2);
legend('Continuous', 'Discrete (200 Hz)');
title('Continuous vs discrete — position step response');
xlabel('Time (s)'); ylabel('Position (m)');
grid on;

%%
% ============================================================
% LEAD ANGLE UPDATE CHECK — 10 degrees
% ============================================================
lead_angle  = 10;           % degrees — updated
screw_diameter = 0.025;     % m

Lead_new = pi * screw_diameter * tan(lead_angle * pi/180);
Ks_new   = Lead_new / (2*pi);
fprintf('New Lead: %.5f m/rev (%.3f mm/rev)\n', Lead_new, Lead_new*1000)
fprintf('New Ks:   %.6f m/rad\n', Ks_new)
fprintf('Change:   %.1f%% of old Ks\n', (Ks_new/0.003350)*100)

% Rebuild plant with new Ks
s = tf('s');
G_motor_simple = Kt / (R*(J*s + b) + Kt*Ke);
G_inner_new    = G_motor_simple * Ks_new;

% Check inner loop with existing compensator
C_inner_existing = 937.5 * (s + 4) / (s * (s + 50));
L_inner_new      = C_inner_existing * G_inner_new;
T_inner_new      = feedback(L_inner_new, 1);

[~, Pm_new, ~, Wcp_new] = margin(L_inner_new);
fprintf('\n--- Inner loop with new Ks ---\n')
fprintf('PM:      %.2f deg  (was 55.08 at 15deg)\n', Pm_new)
fprintf('BW:      %.4f rad/s (was 30.29 at 15deg)\n', Wcp_new)
fprintf('DC gain: %.4f (must be 1.0000)\n', dcgain(T_inner_new))

% Check outer loop
G_outer_new = T_inner_new * (1/s);
C_outer     = tf(1, 1);
L_outer_new = C_outer * G_outer_new;
T_outer_new = feedback(L_outer_new, 1);

[Gm_o, Pm_o, ~, Wcp_o] = margin(L_outer_new);
fprintf('\n--- Outer loop with new Ks ---\n')
fprintf('GM: %.2f dB\n', 20*log10(Gm_o))
fprintf('PM: %.2f deg\n', Pm_o)
fprintf('BW: %.4f rad/s\n', Wcp_o)

% Re-discretise at 200Hz
Ts = 0.005;
C_inner_d   = c2d(C_inner_existing, Ts, 'tustin');
G_inner_d   = c2d(G_inner_new, Ts, 'zoh');
L_inner_d   = C_inner_d * G_inner_d;

[~, Pm_disc, ~, Wcp_disc] = margin(L_inner_d);
fprintf('\n--- Discrete inner loop at 200Hz ---\n')
fprintf('PM: %.2f deg (want > 50)\n', Pm_disc)
fprintf('BW: %.4f rad/s\n', Wcp_disc)

[num_ci, den_ci] = tfdata(C_inner_d, 'v');
fprintf('\nNUM_INNER = [%.8f, %.8f, %.8f]\n', ...
        num_ci(1), num_ci(2), num_ci(3))
fprintf('DEN_INNER = [%.8f, %.8f]\n', -den_ci(2), -den_ci(3))