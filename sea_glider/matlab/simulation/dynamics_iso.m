
syms gravity mass moment rotational_damping body_area wing_area water_density...
    cop_length ballast_mass com_length drag_coefficient lift_coefficient specific_gravity

syms x vx y vy theta omega

v = sqrt(vx*vx + vy*vy);
theta_v = atan2(vy, vx);
phi = theta - theta_v;

n_R_s = [cos(theta) -sin(theta); sin(theta) cos(theta)];
n_R_w = [cos(theta_v) -sin(theta_v); sin(theta_v) cos(theta_v)];

w_drag_force = [-0.5*water_density*body_area*v^2*drag_coefficient; 0];
s_lift_force = [0; 0.5*water_density*wing_area*v^2*lift_coefficient];
n_bouy_force = [0; mass*gravity*(specific_gravity - 1)];

n_drag_force = n_R_w * w_drag_force;
n_lift_force = n_R_s * s_lift_force;

s_com = [com_length; 0];
s_cop = [cop_length; 0];

n_com = n_R_s * s_com;
n_cop = n_R_s * s_cop;

pressure_moment = det([n_cop, n_drag_force]);
ballist_moment = det([n_com, [0; ballast_mass*gravity]]);

dq = [...
     vx;
     (n_drag_force(1) + n_lift_force(1))/mass;
     vy;
     (n_drag_force(2) + n_lift_force(2) + n_bouy_force(2))/mass;
     omega;
     (pressure_moment + ballist_moment - rotational_damping*omega) / moment;
    ];