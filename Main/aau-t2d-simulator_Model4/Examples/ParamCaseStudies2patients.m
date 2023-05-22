% In this example we compare between two patients with different parameters
% for the effect of insulin on glucose uptake. 

%% Define two patients: 

%Define two parameters objects
Param1 = paramClass;
Param2 = paramClass;

%Patient 2 has lower overall effect of insulin on glucose uptake:

%Lower the parameters for the effect of insulin on the hepatic glucose
%uptake and production rates (See paper): 
Param2.c4 = Param2.c4 - 20*Param2.c4;
Param2.c2 = Param2.c2 - 20*Param2.c2;

%The same for the peripheral uptake rate:
Param2.c1 = Param2.c1 - 20*Param2.c1;

%Define the patients:

p1 = T2Dp(Param1,[],[],[],[],[]);
p2 = T2Dp(Param2,[],[],[],[],[]);


%%%%%% Another way of doing it:

%p1 = T2Dp;
%p2 = T2Dp;

%Lower the parameters for the effect of insulin on the hepatic glucose
%uptake rate (See paper): 
%p2.Param.cIHGUinft = p2.Param.cIHGUinft - 0.1*p2.Param.cIHGUinft;
%p2.Param.cIHGPinft = p2.Param.cIHGPinft - 50*p2.Param.cIHGPinft;

%The same for the peripheral uptake rate:
%p2.Param.cIPGU = p2.Param.cIPGU - 0.1*p2.Param.cIPGU;
%%%%%%

%% Define meals: 
%Both patients are assumed to consume the same meals:
%Say the patient eats three meals: 7:00, 12:00, and 18:00 :
Mealtimes = [7*60 12*60 18*60]; %In minutes from 00:00.
MealSizes = [30 50 120]; %In grams.

%The (perday) method can now be used to generate meals for 4 days.
[y, yt]=p1.perday(Mealtimes,MealSizes);


%Set the meals for the simulation:
p1.Meals = y;
p2.Meals = y;

%% Simulation
p1.Simulation;

X1 = p1.X;
time1 = p1.time;

p2.Simulation;

X2 = p2.X;
time2 = p2.time;

%% Plotting
figure 
subplot(2,1,1)
plot(time1/(24*60),X1.GH*0.055,'LineWidth',5);
hold on 
plot(time2/(24*60),X2.GH*0.055,'LineWidth',5);
xlabel('Time [Day]')
ylabel('Plasma Glucose [mmol/L]')
grid on 
legend('p1','p2')
subplot(2,1,2)
plot(time1/(24*60),X1.IH,'LineWidth',5);
hold on 
plot(time2/(24*60),X2.IH,'LineWidth',5);
xlabel('Time [Day]')
ylabel('Plasma Glucose [mU/dL]')
grid on 
legend('p1','p2')