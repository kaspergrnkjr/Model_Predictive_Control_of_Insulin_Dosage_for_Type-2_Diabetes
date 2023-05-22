
clear 
addpath(genpath('aau-simulator'))
%close all 
NUMB = 1;
xinit = 15;
Sig = 1;
Iinit = 2;


%Define the model: 
p = T2Dp;
%These parameters were tuned to find an "average" patient. Do not change
%them for now unless it is necessary 
p.dtdif = 5; %Sampling time of CGM
p.Param.c1 = 1;
p.Param.c2 = 1;
p.Param.d1 = 1;%c1(j);
p.Param.d2 = 1;%c2(j);
p.Param.SPGU = 10;
p.Param.SHGU = 1e-8;
p.Param.SHGP = 1e-8;

% p.Param.ae = 0.1;
% p.Param.alphae = 5;
p.Param.fg = 0.7;

%Final time to simulate: 
T = 5; %In days approximatly 
p.Nd = 1; %Number of days per each loop (1 so you can recommend a different
%insulin dose each day.
dt = 5/(24*60);
time = 0:dt:T;
%Process noise parameter on the glucose (uncomment later if you want).
%p.Param.sig_diff = (Sig(j)/0.056)*sqrt(1/(24*60));

p.GBPC0 = xinit/0.056; %Initial Glucose
p.IBPF0 = Iinit; %Initial insulin 

countd = 0;
Xsim = p.X0;
%Type of insulin (Do not change unless necessaroy) This is the same type of
%Tinna model.
p.LongActingType('Insulin Degludec');
Bg0 = Xsim(35)*0.056;

KP = 50; %P controller gain
count = 0;

for i=1:T
%Stopping time for next day:
%Draw breakfast time 6am is 0:
breakfast_time = ((7-6)*rand+6)*60-6*60;
%You can do it as a normal dist. It is okay.


%Round to 5 minuts:
breakfast_time=round(breakfast_time/5)*5;
%Simulation time: 
Simtime = 1 + (breakfast_time-5)/(60*24);
p.Nd = round(Simtime*24*60)/(24*60); %In minutes
%Meals for the next day:
%Breakfast is 5 mins after starting time
Mealtime = [5 ((8-6)*rand+6)*60 ((14-13)*rand+13)*60]; 
Mealtime = round(Mealtime/5)*5; 
MealSizes =  ([30 50 80]-[10 30 50]).*rand(1,3) + [10 30 50];
[y, yt]= p.perday(Mealtime,MealSizes);
%MealM is for medimum glycemic index
p.MealsM = y;
if i==1
count = count +1;
%Take the SMBG measurement at day 1
ym(count) = Xsim(35)*0.056 + ((1/5)*0.1*log(1+exp(5*(Xsim(35)*0.056-4.2)))+0.415)*randn;
%P-controller :) :
u(1) = round((max(KP*(ym(count)-5),0))/0.5)*0.5;
UlaTime = 0; %At 06:00 (Insulin dose size)
UlaSizes = u(1);
[ula, ~]=p.perday(UlaTime,UlaSizes);
p.Ula = ula;
%Simulate for 1 day until simtime
[~,~,Xsamp,Tsamp] = p.Simulation(5);
Xsim = [Xsim,Xsamp(:,2:end)];
%Glucose value at the end of the simulatiomn:
BG(count) = Xsim(35,end)*0.056;

p.X0sim = p.Xend;
elseif(i>=2)

countd = countd+1; 
count = count+1;
%SMBG measurement at the begining of the day:
ym(count) = BG(count-1) + ((1/5)*0.1*log(1+exp(5*(BG(count-1)-4.2)))+0.415)*randn;
%CGM measurememnts:
ycgm = Xsamp(35,:)*0.056 + 0.42*randn(size(Xsamp(35,:))).*Xsamp(35,:)*0.056;


% You would put your kalman filter + MPC here to calculate the dose for the
% next day. You can either use cgm measureemtns or SMBG measurements for
% the kalman filter.
u(count) = round((max(KP*(ym(count)-5),0))/0.5)*0.5;


UlaTime = 0; %At 06:00
UlaSizes = u(count);
[ula, ~]=p.perday(UlaTime,UlaSizes);
p.Ula = ula;

[~,~,Xsamp,Tsamp] = p.Simulation(5);
Xsim = [Xsim,Xsamp(:,2:end)];
BG(count) = Xsim(35,end)*0.056;
p.X0sim = p.Xend;
end

end
time = 0:1:(length(Xsim(35,:))-1); %We take the time vector depending on size of Xsim
%since the amount of samples we simulate now is random.
plot(time,Xsim(35,:)*0.056)
xlabel('time in dyas')
ylabel('Glucose~[mmol/l]')