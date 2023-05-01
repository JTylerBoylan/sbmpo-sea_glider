% Sea glider results

clc
clear
close all

it = 'Interpreter';
lx = 'Latex';
%%

stats = sbmpo_stats("../csv/stats.csv");
[path, nodes] = sbmpo_results("../csv/nodes.csv");

num_nodes = length(nodes.nodes);
t = zeros(1, num_nodes);
x = zeros(1, num_nodes);
y = zeros(1, num_nodes);
g = zeros(1, num_nodes);
f = zeros(1, num_nodes);

for n = 1:num_nodes
   t(n) = nodes.nodes(n).state(end);
   x(n) = nodes.nodes(n).state(1);
   y(n) = nodes.nodes(n).state(2); 
   g(n) = nodes.nodes(n).g;
   f(n) = nodes.nodes(n).f;
end

num_path = length(path.nodes);
tp = zeros(1, num_path);
xp = zeros(1, num_path);
yp = zeros(1, num_path);
qp = zeros(1, num_path);
vxp = zeros(1, num_path);
vyp = zeros(1, num_path);
sgp = zeros(1, num_path);
Ep = zeros(1, num_path);

for n = 1:num_path
   tp(n) = path.nodes(n).state(end);
   xp(n) = path.nodes(n).state(1);
   yp(n) = path.nodes(n).state(2); 
   qp(n) = path.nodes(n).state(3);
   vxp(n) = path.nodes(n).state(4);
   vyp(n) = path.nodes(n).state(5);
   sgp(n) = path.nodes(n).state(7);
   Ep(n) = path.nodes(n).state(8);
end

yd = -8.0;
ms = 1.5;
md = -1.5;
Tg = 45;

ys = -yd;
y0 = y - yd;
T = Tg - t;

test1 = ys - y0 > ms*T;
test2 = y0 < 0;
test3 = (ys/ms - y0/md) > T;

case1 = ~test1 & ~test2 & ~test3;
case2 = ~test1 & ~test2 & test3;
case3 = ~test1 & test2;
case4 = test1 & ~test2;
case5 = test1 & test2;

figure('Color', [1 1 1])
hold on
grid on
plot(x(case1),y(case1),'xb');
plot(x(case2),y(case2),'xr');
plot(x(case3),y(case3),'xg');
%plot(x(case4),y(case4),'xr');
%plot(x(case5),y(case5),'xg');
plot(xp, yp, '-k', 'LineWidth', 5);
plot([0 max(x)], [yd yd], '--k', 'LineWidth', 2);
ylabel("Y [m]", it, lx)
xlabel("X [m]", it, lx)

figure('Color', [1 1 1])
hold on
grid on
plot(t(case1),y(case1),'xb');
plot(t(case2),y(case2),'xr');
plot(t(case3),y(case3),'xg');
%plot(t(case4),y(case4),'xr');
%plot(t(case5),y(case5),'xg');
plot(tp,yp,'-k','LineWidth',5);
plot([0 45], [yd yd], '--k', 'LineWidth', 2);
ylabel("Y [m]", it, lx)
xlabel("Time [s]", it, lx)
axis([0 45 -10 0])

figure('Color', [1 1 1])
plot(tp, sqrt(vxp.^2 + vyp.^2), 'LineWidth', 5);
ylabel("Velocity [m/s]", it, lx)
xlabel("Time [s]", it, lx)

figure('Color', [1 1 1])
hold on
grid on
plot(tp, Ep, 'LineWidth', 5)
ylabel("Energy Used [J]", it, lx)
xlabel("Time [s]", it, lx)
axis([0 tp(end) -inf inf])