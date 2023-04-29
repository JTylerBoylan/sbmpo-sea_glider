clear
close all
clc

syms y0 ys md ms T 

%% case 1

t1_1 = -y0/md;
t2_1 = -ys/ms + T;

A1_1 = 1/2*md*(t1_1^2)+y0*t1_1;
A1_1 = simplify(A1_1)

A2_1 = 1/2*ms*(T^2-t2_1^2)-ms*T*(T-t2_1)+ys*(T-t2_1);
A2_1 = simplify(A2_1)

A_1 = simplify(A1_1 + A2_1)

%% case 2

ti = ((ys-y0)-ms*T)/(md-ms);

A1_2 = 1/2*md*(ti^2)+y0*ti;
A1_2 = simplify(A1_2);

A2_2 = 1/2*ms*(T^2-ti^2)-ms*T*(T-ti)+ys*(T-ti);
A2_2 = simplify(A2_2);

A_2 = simplify(A1_2 + A2_2)

%% case 3

t1_3 = -y0/ms;
t2_3 = -ys/ms + T;

A1_3 = -1/2*ms*(t1_3^2)-y0*t1_3;
A1_3 = simplify(A1_3);

A2_3 = 1/2*ms*(T^2-t2_3^2)-ms*T*(T-t2_3)+ys*(T-t2_3);
A2_3 = simplify(A2_3);

A_3 = simplify(A1_3 + A2_3)

%% case 4

ts_4 = (ys-y0)/ms;

A_4 = 1/2*ms*ts_4^2 + y0*ts_4;
A_4 = simplify(A_4)

%% case 5

ti_5 = -y0/ms;
ts_5 = (ys-y0)/ms;

A1_5 = -1/2*ms*ti_5^2 - y0*ti_5;
A1_5 = simplify(A1_5);

A2_5 = 1/2*ms*(ts_5^2-ti_5^2) + y0*(ts_5-ti_5);
A2_5 = simplify(A2_5)

A_5 = simplify(A1_5 + A2_5)

