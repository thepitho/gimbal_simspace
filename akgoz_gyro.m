function g = akgoz_gyro(w, nw, nb)
% STIM210 gyro model at 1 kHz (port of gimbal-for-fun sensors.py Gyro):
% 1st-order bandwidth + pure delay, scale factor, misalignment, turn-on bias,
% Gauss-Markov bias, white noise (ARW), quantization.  g = K*y + b + n.
% w: true head rate (camera frame) [rad/s]; nw, nb: unit white normals (3x1 each).
% Inserted into a MATLAB Function block by build_gimbal.m (sample time 1/P.fs).
persistent y gm buf
P = akgoz_params();
fs = P.fs;
if isempty(y)
    y = zeros(3,1); gm = zeros(3,1); buf = zeros(3,8);
end
a = 1 - exp(-2*pi*P.gyro.bw/fs);
phi = exp(-1/(fs*P.gyro.bias_tc));
nd = max(round((P.gyro.delay - 1/(2*pi*P.gyro.bw))*fs), 0);  % datasheet delay minus the lag's own
nd = min(nd, 7);
K = diag(1 + P.gyro.sf)*(eye(3) + P.gyro.mis);
y = y + a*(w - y);
gm = phi*gm + P.gyro.bias_instab*sqrt(1 - phi^2)*nb;
m = K*y + P.gyro.bias0 + gm + P.gyro.arw*sqrt(fs)*nw;
m = round(m/P.gyro.quant)*P.gyro.quant;
buf = [m, buf(:,1:end-1)];
g = buf(:, nd+1);
end
