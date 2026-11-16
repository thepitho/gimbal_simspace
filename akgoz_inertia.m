function J = akgoz_inertia()
% make_akgoz_urdf.py tarafindan uretildi; elle duzenlemeyin. [kg*m^2]
% source: cad:gimbal-for-fun cad/turret.py (cad_ref/massprops.json)
% Tensors in FLU (x fwd, y left, z up) about O (axis intersection).
J.el      = 0.003897156679;  % head (pitch body), elevation axis
J.head_xx = 0.004850308606;  % head tensor, camera frame
J.head_zz = 0.005709128402;
J.head_xz = -1.098945819e-11;
J.yoke_zz = 0.004191323028;  % yoke (yaw body), azimuth axis
J.m_head  = 1.512957393;  % head mass [kg]
% J_az(el) = yoke_zz + head_xx*sin(el)^2 + head_zz*cos(el)^2 + 2*head_xz*sin(el)*cos(el)
end
