%%

clc
clear
close all

it = 'Interpreter';
lx = 'Latex';

%%

goal_time = 45;
max_time = 45;

desired_depth = -8;
max_depth = -10;

m_dive = -1.5;
m_surf = 1.5;

resolution = 1000;
[t,y] = meshgrid(linspace(0,max_time,resolution), linspace(0,max_depth,resolution));

% transform grid
T = goal_time - t;
ys = -desired_depth;
y0 = y - desired_depth;

% cases
test1 = ys - y0 > m_surf*T;
test2 = y0 < 0;
test3 = (ys/m_surf - y0/m_dive) > T;

case1 = ~test1 & ~test2 & ~test3;
case2 = ~test1 & ~test2 & test3;
case3 = ~test1 & test2;
case4 = test1 & ~test2;
case5 = test1 & test2;

% case plot
cases = zeros(size(t));
cases(case1) = 1;
cases(case2) = 2;
cases(case3) = 3;
cases(case4) = nan;
cases(case5) = nan;

map = [0 0 1;
       1 0 0;
       0 1 0];

figure('Color', [1 1 1])
hold on
grid on
contourf(t,y,cases);
colormap(map)
xlabel("Time [s]", it ,lx)
ylabel("Y [m]", it, lx)

% heuristics
h1 = 1/2*(ys^2/m_surf - y0.^2/m_dive);
h2 = 1/2*((m_dive*m_surf*T + 2*m_surf*y0 - 2*m_dive*ys).*T + (y0 - ys).^2)./(m_surf-m_dive);
h3 = 1/2*(ys^2 + y0.^2)/m_surf;
h4 = 1/2*(ys^2 - y0.^2)/m_surf;
h5 = 1/2*(ys^2 + y0.^2)/m_surf;

h = zeros(size(t));
h(case1) = h1(case1);
h(case2) = h2(case2);
h(case3) = h3(case3);
h(case4) = h4(case4);
h(case5) = h5(case5);


% plot
figure
contourf(t, y, h);