function [err, el_enc] = akgoz_los_estimator(q, cmd)
% LOS error estimate for the outer loop: base attitude (ideal INS) x 16-bit encoders.
% q: true joint angles [base_yaw base_pitch az el]; cmd: inertial LOS command [az; el].
% err: [az; el] rotation-vector LOS error in camera frame (see akgoz_los_kinematics);
% el_enc: quantized elevation angle (secant / J_az scheduling in the controller).
% Inserted into a MATLAB Function block by build_gimbal.m (sample time 1/P.fs).
P = akgoz_params();
lsb = 2*pi/2^P.enc.bits;
qe = q;
qe(3:4) = round(q(3:4)/lsb)*lsb;
[~, err] = akgoz_los_kinematics(qe, zeros(4,1), cmd);
el_enc = qe(4);
end
