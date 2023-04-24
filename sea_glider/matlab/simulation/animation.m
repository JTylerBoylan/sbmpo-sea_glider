
L = 3;
glider = [-L/2 L/2; 0 0];

fig = figure;
hold on
grid on
plot(x,y,'--g');
glider_plot = plot(0,0,'-b','LineWidth',10);
axis([min(x)-L, max(x)+L, min(y)-L, max(y)+L]);
axis equal

ti = linspace(0, t(end), 600);
xi = interp1(t,x,ti);
yi = interp1(t,y,ti);
qi = interp1(t,theta,ti);

tic
for k = 1:length(ti)
    
    figure(fig);
    
    cx = xi(k);
    cy = yi(k);
    cq = qi(k);
    
    n_R = [cos(cq) -sin(cq); sin(cq) cos(cq)];
    
    cpos = [cx; cy];
    cglider = n_R*glider + cpos;
    
    set(glider_plot, 'xdata', cglider(1,:));
    set(glider_plot, 'ydata', cglider(2,:));
    
    pause(ti(k) - toc);
end