function y = akgoz_base_motion(t)
% Vehicle (base) angles [yaw; pitch] in rad: slow maneuver sine + vibration.
% Vibration: deterministic multisine over P.base.vib_band with the given rms RATE,
% plus a prop-harmonic tone. Placeholder spectrum (gimbal-for-fun vehicle.*).
% A 0.5 s raised-cosine envelope fades the vibration in (consistent initial state).
% Scalar loops only (no parameter-sized arrays) so code generation stays fixed-size.
P = akgoz_params();
n = P.base.vib_n;
f1 = P.base.vib_band(1); f2 = P.base.vib_band(2);
env = 0.5 - 0.5*cos(pi*min(t/0.5, 1));
y = zeros(2,1);
for ax = 1:2
    a = P.base.vib_sigma(ax)*sqrt(2/n);                  % rate amplitude per tone
    vib = 0;
    for kk = 1:n
        f = f1*(f2/f1)^((kk-1)/(n-1));                   % log-spaced tones
        ph = 2*pi*mod(kk*0.6180339887 + 0.37*ax, 1);     % fixed pseudo-random phase
        vib = vib + a/(2*pi*f)*sin(2*pi*f*t + ph);
    end
    tone = P.base.tone_amp(ax)/(2*pi*P.base.tone_freq)*sin(2*pi*P.base.tone_freq*t + ax);
    man = P.base.man_amp(ax)*sin(2*pi*P.base.man_freq(ax)*t);
    y(ax) = man + env*(vib + tone);
end
end
