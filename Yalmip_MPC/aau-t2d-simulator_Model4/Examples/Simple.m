%In this example, we define a patient with the defult parameters and
%simulate the patient with meals only.
clear all 
close all
%% Define a patient object: 
p = T2Dp; 
p.Param.sig_diff = 0;
%Defult initial conditions:

%Glucose concentration in the periphery central compartment:
   %GBPC0=7/0.0555; % in mg/dl
    
   %Insulin concentraion in the in periphery compartment: 
   %IBPF0=1; % in mU/l
   
%Default number of simulation days: Nd = 4 Days 
%Default sampling time for inputs dt = 1 min

p.GBPC0 = 22./0.0555;
%% Define meals: 
p.Nd = 1; 
%p.GBPC0 = 20
%The method (perday) lets you define any input for day and repeated for all
%the simulation days:
%Say the patient eats three meals: 7:00, 12:00, and 18:00 :
Mealtimes = [7*60 12*60 18*60]; %In minutes from 00:00.
MealSizes = [30 50 120]/70; %In grams.

%The (perday) method can now be used to generate meals for 4 days.
[y, yt]=p.perday(Mealtimes,MealSizes);
% figure
% stem(p.T/(60),y);
% xlabel('Time [min]');
% ylabel('Meals [g]');

%Set the meals for the simulation:
%p.Meals = ;

%% Simulation:
%The simulation method returns the time of the simulation (Which is
%different than the time vector T with fixed sampling dt) and the values
%for the states Xsim. It also modifies the propeties X (Which is a structre)
%and time in the object
%p:
[Xsim,timesim]=p.Simulation;
%One can also obtain X and time by:

X = p.X; %This one is a structre.

time = p.time;

%% Plotting:
%figure 
plot(time/(24*60),X.GH*0.056,'LineWidth',5);
xlabel('Time [Day]');
ylabel({'Plasma Glucose [mmol/L]'});
grid on