%For reproducibility:
rng(0,'twister')

%This example demonstrate how to use the simulator in a closed loop fashion
%with SMBG measuerements on 4 randomly drawn patients. 

%The controller takes an SMBG measurement at the last day of a week and
%adjust a basal insulin dose accordingly.

%Load the parameters' set with their names:
load('PatientsSMBGtunned/ThetatunedSMBG.mat')

%Defining the number of patients to simulate: 
Np = 3;
Pcell = {};
%Draw 4 different patients from the parameters set: 
for j = 1:Np
    p = T2Dp(); 
    p.DrawRandomPatient(Thetaname,ThetatunedSMBG);
    %We set the simulation time to be 7 days (The insulin dose will change
    %each 7 days).
    p.Nd = 7; 

    %Initial Glucose: 
    p.GBPC0 = 9/0.056; 

    %We set an inital insulin dose: 
    INS = 5; %In units.

    UlaTime = 0; %At 00:00
    UlaSizes = INS;

    [ula, ~]=p.perday(UlaTime,UlaSizes);

    p.Ula = ula;
    
    Pcell{j} = p;
end

%We set initial meals:
Mealtimes = [0.5*60 (5.5)*60 (11.5)*60]; %In minutes from 00:00.
MealSizes = [50 80 120]; %In grams.
    
%Times to SMBG: 
SMBGtime = [7*24*60]; %Last day
%Number of weeks to simulate: 
W = 20; 
%Number of different patients:
Np = 3;
SMBG = zeros(Np,W); %SMBG measurements

for j=1:Np
    INS = 10;
    for i=1:W
        %Meals for the week:
        [y, yt]= Pcell{j}.perday(Mealtimes,MealSizes);
        %Change the meal randomly:
        y(yt) = (1.1*y(yt)-0.8*y(yt)).*rand(size(y(yt)))+0.8*y(yt);
        Pcell{j}.Meals = y;
        %Simulate for one week:
        [Xsim,timesim]=Pcell{j}.Simulationfast; %p.Simulationfast is faster than
        %p.Simulation but less stable Error in the simulation might occur. To
        %perform more robust simulations, use p.Simulation instead.
        %Obtain SMBG measurement:
        SMBG(j,i) = Pcell{j}.SMBGmeas(SMBGtime,'simple');

        %Calculate insulin dose for the next week:
        if (SMBG(j,i)*0.056)<4
            INS = INS - 2; 
        elseif (SMBG(j,i)*0.056)>5
            INS = INS + 2;
        end
        [ula, ~]=Pcell{j}.perday(0,INS);
        Pcell{j}.Ula = ula;

%Set initial for next week: (p.X0sim is used for closed loop simulations)
%Since by default, the simulator computes the initial conditions based on
%Basal values. However, in closed loop we want to perform multiple
%simulations where the last point of the previous simulation
%is simply the first point of the next simulation. In that case, we use
%X0sim. To go back to the default mode, set X0sim to be empty p.X0sim =[];
        Pcell{j}.X0sim = Pcell{j}.Xend;
    end
end

plot(0:(W),[9*ones(Np,1) SMBG]*0.056,'x','MarkerSize',5)
grid on 
xlabel('Week')
ylabel('SMBG [mmol/L]')
legend('Patient1','Patient2','Patient3')