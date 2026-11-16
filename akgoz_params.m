function P = akgoz_params()
% AKGOZ_PARAMS Single source of truth for the LOS stabilization model (codegen-safe).
% Every value carries a source tag in its comment (gimbal-for-fun CLAUDE.md convention):
%   user | datasheet:<url> | calc:<file> | placeholder (NOT a design value)
% Values copied from gimbal-for-fun/params/gimbal.yaml keep that file's source.
% SI units. Axis order everywhere: [az; el].

% ---- Timing ----
fs = 1000;                           % Hz, control + sensor rate; source: user (1 kHz)
P.fs = fs;                           % (codegen: no field reads while the struct is built)
P.Ts = 1/fs;

% ---- Inertia (from the URDF geometry) ----
P.J = akgoz_inertia();               % source: calc:make_akgoz_urdf.py

% ---- Motors: Celera Omni+ OPH-060-013-D, per axis, FOC current loop ----
% DS_MOT = 'https://novanta.com/robotics-automation/wp-content/uploads/sites/3/2026/03/Omni__Motors_Data_Sheet_English.pdf'
P.mot.kt   = [0.168; 0.168];         % N*m/A; source: datasheet:DS_MOT (KtSINE per A peak)
P.mot.imax = [2.65; 2.65];           % A; source: calc: 0.446 N*m cont / 0.168 (datasheet:DS_MOT)
P.mot.cl_bw = 500;                   % Hz, current loop bandwidth (1st order); source: placeholder
P.mot.cog_amp = [0.0045; 0.0045];    % N*m; source: calc: 1 % of 0.446 N*m (datasheet:DS_MOT)
P.mot.cog_per_rev = [84; 84];        % source: placeholder (slot count not published)

% ---- Friction: Stribeck + viscous, smoothed sign ----
P.fr.coulomb = [0.010; 0.008];       % N*m; source: placeholder
P.fr.static  = [0.015; 0.012];       % N*m; source: placeholder
P.fr.v_stribeck = [0.01; 0.01];      % rad/s; source: placeholder
P.fr.viscous = [0.002; 0.002];       % N*m*s/rad; source: placeholder
P.fr.eps = 1e-3;                     % rad/s, tanh smoothing of sign(); source: placeholder (model choice)
P.fr.cable_k_el = 0.005;             % N*m/rad, pitch cable stiffness; source: placeholder

% ---- Gyro: Safran STIM210 on the head (LOS rate feedback) ----
% DS_GYRO = 'https://safran-navigation-timing.com/wp-content/uploads/2022/12/STIM210_Datasheet.pdf'
P.gyro.arw = 0.15*pi/180/60;         % rad/sqrt(s); source: datasheet (Safran brochure 2026) 0.15 deg/sqrt(h)
P.gyro.bias_instab = 0.3*pi/180/3600;% rad/s; source: datasheet (Safran brochure 2026) <= 0.3 deg/h
P.gyro.bias_tc = 100;                % s, Gauss-Markov corr. time; source: placeholder (model choice)
P.gyro.bw = 262;                     % Hz, -3 dB; source: datasheet:DS_GYRO table 5-3
P.gyro.delay = 0.0027;               % s; source: calc: 2.2 ms group + 0.5 ms (datasheet:DS_GYRO note 5)
P.gyro.quant = 0.22*pi/180/3600;     % rad/s; source: datasheet:DS_GYRO resolution 0.22 deg/h
% One fixed example draw of the random errors (deterministic runs):
P.gyro.bias0 = [8; -5; 11]*pi/180/3600;   % rad/s; source: placeholder draw, sigma 10 deg/h (datasheet:DS_GYRO)
P.gyro.sf    = [300; -450; 150]*1e-6;     % source: placeholder draw, sigma 500 ppm (datasheet:DS_GYRO)
P.gyro.mis   = 1e-3*[0 0.6 -0.9; -0.4 0 0.7; 1.1 -0.3 0]; % rad off-diag; source: placeholder draw, sigma 1 mrad
P.gyro.seed_white = [11 12 13];      % Random Number seeds; source: placeholder
P.gyro.seed_bias  = [21 22 23];

% ---- Encoder ----
P.enc.bits = 16;                     % source: placeholder

% ---- Controller (gimbal-for-fun control.py structure + lead) ----
P.ctl.rate_wc_pi   = 2*pi*25;        % rad/s, rate-loop crossover, lead OFF (PM ~43 deg); source: placeholder (gimbal-for-fun)
P.ctl.rate_wc_lead = 2*pi*35;        % rad/s, crossover, lead ON (PM ~61 deg); source: calc:tools/axis_margin_check.py (decision)
P.ctl.rate_ti_ratio = 4;             % integrator corner = wc/ratio; source: placeholder
P.ctl.rate_lpf = 150;                % Hz, torque-command LPF; source: placeholder
P.ctl.lead_phase = 30;               % deg, max phase lead placed at rate_wc; source: placeholder (model choice)
P.ctl.secant_cap = 5;                % caps 1/cos(el); source: placeholder
P.ctl.pos_bw = 3;                    % Hz, outer LOS position loop; source: placeholder
P.ctl.slew_max = 60*pi/180;          % rad/s, rate command cap; source: placeholder
P.ctl.dob_cutoff = 20;               % Hz, DOB Q filter; 40 Hz destabilises with lead at 35-40 Hz crossover;
                                     % source: calc:tools/axis_margin_check.py (decision)

% ---- Base (vehicle) motion: maneuver + vibration, rate amplitudes ----
% Vibration spectrum from gimbal-for-fun vehicle.* (placeholder spectrum), realised
% here as a deterministic multisine instead of filtered noise. Rows: [yaw; pitch].
P.base.man_amp  = [4; 3]*pi/180;     % rad, slow maneuver sine; source: placeholder
P.base.man_freq = [0.20; 0.27];      % Hz; source: placeholder
P.base.vib_sigma = [0.5; 1.0]*pi/180;% rad/s rms broadband (FRD yaw, pitch); source: placeholder
P.base.vib_band  = [2 40];           % Hz; source: placeholder
P.base.vib_n     = 8;                % number of tones in the band (model choice)
P.base.tone_freq = 85;               % Hz, prop harmonic; source: placeholder
P.base.tone_amp  = [0.2; 0.5]*pi/180;% rad/s; source: placeholder

% ---- LOS command scenario (inertial az/el) ----
P.cmd.t  = [2 6];                    % s: el step time, az step time
P.cmd.el = -20*pi/180;               % rad (negative = look down)
P.cmd.az = 15*pi/180;                % rad
P.sim.stop = 12;                     % s
P.sim.rms_from = 8;                  % s, steady-state LOS metric window start
end
