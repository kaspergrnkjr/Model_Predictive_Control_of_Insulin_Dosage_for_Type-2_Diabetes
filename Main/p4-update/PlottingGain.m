close all
clc
clear all

filename = 'gain0.1.csv';
gain01 = csvread(filename);
filename = 'gain0.2.csv';
gain02 = csvread(filename);
filename = 'gain0.3.csv';
gain03 = csvread(filename);
filename = 'gain0.4.csv';
gain04 = csvread(filename);
filename = 'gain0.5.csv';
gain05 = csvread(filename);
filename = 'gain0.6.csv';
gain06 = csvread(filename);

t = 0:18;
t = t';
hold on
plot(t,[1.8;gain01(:,2)],"LineWidth",2)
plot(t,[1.8;gain02(:,2)],"LineWidth",2)
plot(t,[1.8;gain03(:,2)],"LineWidth",2)
plot(t,[1.8;gain04(:,2)],"LineWidth",2)
plot(t,[1.8;gain05(:,2)],"LineWidth",2)
plot(t,[1.8;gain06(:,2)],"LineWidth",2)
legend("Gain 0.1", "Gain 0.2","Gain 0.3","Gain 0.4","Gain 0.5","Gain 0.6",'Location','southeast')
title('Insulin sensitivity')
xlabel('Time [days]') 
ylabel('Insulin Sensitivity [U^{-1}]') 
grid on
hold off 

figure(2)
hold on
plot(gain01(:,3),"LineWidth",2)
plot(gain02(:,3),"LineWidth",2)
plot(gain03(:,3),"LineWidth",2)
plot(gain04(:,3),"LineWidth",2)
plot(gain05(:,3),"LineWidth",2)
plot(gain06(:,3),"LineWidth",2)
legend("Gain 0.1", "Gain 0.2","Gain 0.3","Gain 0.4","Gain 0.5","Gain 0.6")
title('Deviation of measurement and model')
xlabel('Time [days]')
ylabel('Deviation') 

grid on
hold off 