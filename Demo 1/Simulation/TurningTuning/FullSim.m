clc

data = xlsread('StepResponseData.xlsx'); 
 
% first_column = data(:,1); 
second_column = data(:,4); 


x = linspace(0,2,201);

figure(1)
plot(x, second_column)


Ra=1;
Kt=0.5;
Ke=0.5;
J=.05;
b=.5;

open_system('FullSim01')
out=sim('FullSim01');

hold on
plot(x, out.Velocity.signals.values)
hold off


