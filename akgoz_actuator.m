function [tau_az, tau_el] = akgoz_actuator(i, q, w)
% Motor torque minus bearing friction, cogging and cable torque (continuous time).
% i: motor current after the current loop [A] ([az; el]); q, w: 4 joint angles/rates.
% Friction: Stribeck + viscous with tanh-smoothed sign (gimbal-for-fun friction.*;
% no LuGre bristle state here, so true stiction is not modelled).
P = akgoz_params();
qa = q(3:4); wa = w(3:4);
tau_m = P.mot.kt.*i;
fr = (P.fr.coulomb + (P.fr.static - P.fr.coulomb).*exp(-(wa./P.fr.v_stribeck).^2)) ...
    .*tanh(wa/P.fr.eps) + P.fr.viscous.*wa;
cog = P.mot.cog_amp.*sin(P.mot.cog_per_rev.*qa);
cable = [0; P.fr.cable_k_el*qa(2)];
tau = tau_m - fr - cog - cable;
tau_az = tau(1); tau_el = tau(2);
end
