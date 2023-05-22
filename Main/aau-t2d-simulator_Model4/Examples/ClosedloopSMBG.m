%For reproducibility:
rng(0,'twister')
clear all 
%This example demonstrate how to use the simulator in a closed loop fashion
%with SMBG measuerements. 

%The controller takes an SMBG measurement at the last day of a week and
%adjust a basal insulin dose accordingly.

%First, we define a patient: 
p = T2Dp; 

%We set the simulation time to be 7 days (The insulin dose will change
%each 7 days).
p.Nd = 7; 

%Initial Glucose: 
p.GBPC0 = 9/0.055; %mmol/dl  

%We set initial meals: 
Mealtimes = [0.5*60 (5.5)*60 (11.5)*60]; %In minutes from 00:00.
MealSizes = [50 80 120]; %In grams.


%We set an inital insulin dose: 
INS = 10; %In units.

UlaTime = 0; %At 00:00
UlaSizes = INS;

[ula, ~]=p.perday(UlaTime,UlaSizes);

p.Ula = ula;
%p.LongActingType('Abasaglar KwikPen');
%Times to SMBG: 
SMBGtime = [7*24*60]; %L
%Number of weeks to simulate: 
W = 20; 
SMBG = zeros(1,W); %SMBG measurements
for i=1:W
%Meals for the week:
[y, yt]= p.perday(Mealtimes,MealSizes);
%Change the meal randomly:
y(yt) = (1.1*y(yt)-0.6*y(yt)).*rand(size(y(yt)))+0.6*y(yt);
p.Meals = y;
%Simulate for one week:
[Xsim,timesim]=p.Simulationfast; %p.Simulationfast is faster than
%p.Simulation but less stable Error in the simulation might occur. To
%perform more robust simulations, use p.Simulation instead.
%Obtain SMBG measurement:
SMBG(i) = p.SMBGmeas(SMBGtime,'simple');

%Calculate insulin dose for the next week:
if (SMBG(i)*0.056)<4
    INS = INS - 2; 
elseif (SMBG(i)*0.056)>5
    INS = INS + 2;
end
[ula, ~]=p.perday(0,INS);
p.Ula = ula;

%Set initial for next week: (p.X0sim is used for closed loop simulations)
%Since by default, the simulator computes the initial conditions based on
%Basal values. However, in closed loop we want to perform multiple
%simulations where the last point of the previous simulation
%is simply the first point of the next simulation. In that case, we use
%X0sim. To go back to the default mode, set X0sim to be empty p.X0sim =[];
p.X0sim = p.Xend;
end

plot(0:(W-1),SMBG*0.056,'x','MarkerSize',5)
grid on 
xlabel('Week')
ylabel('SMBG [mmol/L]')