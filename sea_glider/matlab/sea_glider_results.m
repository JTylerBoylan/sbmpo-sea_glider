% Sea glider results

close all

%%

stats = sbmpo_stats("../csv/stats.csv");
[path, nodes] = sbmpo_results("../csv/nodes.csv");

num_nodes = length(nodes.nodes);
t = zeros(1, num_nodes);
x = zeros(1, num_nodes);
y = zeros(1, num_nodes);

for n = 1:num_nodes
   t(n) = nodes.nodes(n).state(end);
   x(n) = nodes.nodes(n).state(1);
   y(n) = nodes.nodes(n).state(2); 
end

num_path = length(path.nodes);
tp = zeros(1, num_path);
xp = zeros(1, num_path);
yp = zeros(1, num_path);

for n = 1:num_path
   tp(n) = path.nodes(n).state(end);
   xp(n) = path.nodes(n).state(1);
   yp(n) = path.nodes(n).state(2); 
end

ms = 0.03;
md = -0.03;
ys = 1.0;
y0 = y + 1.0;
T = 180 - t;

case4 = ys - y0 > ms*T;
case3 = ~case4 & y0 < 0;
case2 = ~case4 & ~case3 & (ys/ms - y0/md) <= T;
case1 = ~case4 & ~case3 & ~case2;

figure
hold on
plot(x(case1),y(case1),'xr');
plot(x(case2),y(case2),'xb');
plot(x(case3),y(case3),'xg');
plot(x(case4),y(case4),'xy');
plot(xp, yp, '-b', 'LineWidth', 5);
title("Y vs X")

figure
hold on
plot(t(case1),y(case1),'xr');
plot(t(case2),y(case2),'xb');
plot(t(case3),y(case3),'xg');
plot(t(case4),y(case4),'xy');
plot(tp,yp,'-b','LineWidth',5);
title("Y vs Time")