clear all
p = T2Dp; 
p.Nd = 1;
p.dtdif = 5 ; 
Mealtimes = [((8-6)*rand+6)*60 ((14-12)*rand+12)*60 ((21-19)*rand+19)*60 ]-6*60; %In minutes from 00:00.
MealSizes =  [45 75 85]+10*randn(1,3); %In grams.

%The (perday) method can now be used to generate meals for 4 days.
[y, yt]=p.perday(Mealtimes,MealSizes);
p.Meals = y;
p.LongActingType('Insulin Degludec')
%We set an inital insulin dose: 
INS = 50; %In units.

UlaTime = 0; %At 00:00
UlaSizes = 20;

[ula, ~]=p.perday(UlaTime,UlaSizes);

p.Ula = ula;
tstart = tic;
for i=1:1
[~,~,Xsamp,Tsamp] = p.Simulation(5);
p.X0sim = p.Xend;
end
toc(tstart);
X = p.X; 
time = p.time;
plot(time,X.GH*0.056)
hold on 
plot(Tsamp,Xsamp(35,:)*0.056)