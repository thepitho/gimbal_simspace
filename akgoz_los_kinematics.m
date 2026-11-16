function [wc, err, R] = akgoz_los_kinematics(q, w, cmd)
% Head (camera) attitude, inertial angular rate and LOS pointing error.
% q, w : [base_yaw; base_pitch; az; el] joint angles [rad] / rates [rad/s]
% cmd  : [az; el] desired inertial LOS direction [rad] (world X fwd, Y left, Z up)
% wc   : inertial angular velocity of the head, camera frame (X LOS, Y left, Z up) = ideal gyro
% err  : [az; el] rotation-vector error taking the LOS onto cmd, in camera frame:
%        err(1) about camera +Z (cross-elevation), err(2) about camera -Y (elevation, + = nose up)
% R    : world <- camera rotation, R = Rz(yaw)*Ry(pitch)*Rz(az)*Ry(-el)
Rz = @(t) [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
Ry = @(t) [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
R  = Rz(q(1))*Ry(q(2))*Rz(q(3))*Ry(-q(4));
wb = Ry(q(2)).'*[0;0;w(1)] + [0;w(2);0];   % base, base frame
wa = Rz(q(3)).'*wb + [0;0;w(3)];           % azimuth frame
wc = Ry(-q(4)).'*wa + [0;-w(4);0];         % head, camera frame
l = R(:,1);                                % LOS (optical axis) in world
d = [cos(cmd(2))*cos(cmd(1)); cos(cmd(2))*sin(cmd(1)); sin(cmd(2))];
c = cross(l, d);
s = norm(c);
if s < 1e-12
    e = zeros(3,1);
else
    e = c/s*atan2(s, l.'*d);               % rotation vector l -> d, world frame
end
eC = R.'*e;                                % camera frame
err = [eC(3); -eC(2)];
end
