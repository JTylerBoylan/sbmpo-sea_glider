%% Jonathan Boylan
% 23-03-12

clc
clear
close all

%%

inits = [0 0 0 0 0 0];
t_int = [0 10];

[t, q] = ode45(@(t,q) gliderODE(t,q), t_int, inits);

x = q(:,1);
vx = q(:,2);
y = q(:,3);
vy = q(:,4);
theta = q(:,5);
omega = q(:,6);

%%
figure
subplot(3,2,1)
plot(t,x)
ylabel("X")
subplot(3,2,2)
plot(t,vx)
ylabel("Vx")

subplot(3,2,3)
plot(t,y)
ylabel("Y")
subplot(3,2,4)
plot(t,vy)
ylabel("Vy")

subplot(3,2,5)
plot(t,theta)
ylabel("\theta")
xlabel("Time")
subplot(3,2,6)
plot(t,omega)
ylabel("\omega")
xlabel("Time")
sgtitle("States")

figure
plot(x,y)
xlabel("X")
ylabel("Y")
title("Position")

%%
v = sqrt(vx.*vx + vy.*vy);
theta_v = atan2(vy,vx);
phi = theta - theta_v;

figure
subplot(3,1,1)
plot(t,v)
ylabel("V")

subplot(3,1,2)
hold on
plot(t,theta_v)
plot(t,theta)
legend(["\theta_v" "\theta"])

subplot(3,1,3)
plot(t,phi)
ylabel("\phi")
xlabel("Time");

%%

function SG = specific_gravity(t)
    %f = 10;
    %SG = 1 + 0.995*sin(2*pi*t/f);
    
    SG = 1.5;
end

function CD = drag_coefficient(phi)
    CD0 = 0.01;
    KD = 0.05;
    CD = CD0 + abs(KD*sin(phi));
end

function CL = lift_coefficient(phi)
    KL = 0.25;
    CL = KL*sin(2*phi);
end

function dq = gliderODE(t, q)

    gravity = -9.81; % m/s^2
    mass = 10; % kg
    moment = 5; % kg*m^2
    rotational_damping = 10; % kg*m/s
    body_area = 0.25; % m^2
    wing_area = 1.0; % m^2
    water_density = 1000; % kg/m^3
    cop_length = -0.1; % m
    
    % x = q(1);
    vx = q(2);
    % y = q(3);
    vy = q(4);
    theta = q(5);
    omega = q(6);

    v = sqrt(vx*vx + vy*vy);
    theta_v = atan2(vy, vx);
    phi = theta - theta_v;
    
    n_R_s = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    n_R_w = [cos(theta_v) -sin(theta_v); sin(theta_v) cos(theta_v)];

    w_drag_force = [-0.5*water_density*body_area*v^2*drag_coefficient(phi); 0];
    s_lift_force = [0; 0.5*water_density*wing_area*v^2*lift_coefficient(phi)];
    n_bouy_force = [0; mass*gravity*(specific_gravity(t) - 1)];
    
    n_drag_force = n_R_w * w_drag_force;
    n_lift_force = n_R_s * s_lift_force;
    
    s_cop = [cop_length; 0];
    n_cop = n_R_s * s_cop;
    pressure_moment = det([n_cop, n_drag_force]);
    
    dq = [...
         vx;
         (n_drag_force(1) + n_lift_force(1))/mass;
         vy;
         (n_drag_force(2) + n_lift_force(2) + n_bouy_force(2))/mass;
         omega;
         (pressure_moment - rotational_damping*omega) / moment;
        ];

end