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

figure
hold on
plot(x,y,'xr');
plot(xp, yp, '-b');