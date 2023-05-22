function x = RealPatientSim(obj,tspan, initialValues)
%close all 
NUMB = 1;
xinit = 10.5;
Sig = 1;
Iinit = 2;
MealsMTemp = 0;
x_hat = zeros(((24*60)/obj.timeStepInMin)*obj.simDays,obj.numberOfStates);
breakfastStates = zeros(obj.numberOfStates,1);
dayOffset = 0;
numberOfIntervals = (24*60)/obj.timeStepInMin;
loadingMessages = ["Getting a mug" "Pouring coffee" "Sipping.." "Too hot!" "Blowing.." "Sipping..." "Perfect!" "Ahhh..." "Jokke er grem"];

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
p.Param.SHGU = 1e-2;
p.Param.SHGP = 1e-2;

% p.Param.ae = 0.1;
% p.Param.alphae = 5;
p.Param.fg = 0.7;

%Final time to simulate: 
%T = 5; %In days approximatly 
p.Nd = 1; %Number of days per each loop (1 so you can recommend a different
%insulin dose each day.z
%Process noise parameter on the glucose (uncomment later if you want).
%p.Param.sig_diff = (Sig(j)/0.056)*sqrt(1/(24*60));

p.GBPC0 = xinit/0.056; %Initial Glucose
p.IBPF0 = Iinit; %Initial insulin 

count = 0;
countd = 0;
Xsim = p.X0;
%Type of insulin (Do not change unless necessaroy) This is the same type of
%Tinna model.
p.LongActingType('Insulin Degludec');
Bg0 = Xsim(35)*0.056;



for i=1:obj.simDays

    p.Nd = 1;
    %%%% Clean the syringe (set injection to zero so we know we dont injected at wrong times) %%%%   
    u(1) = 0;
    UlaTime = 0; %At 06:00 (Insulin dose size)
    UlaSizes = u(1);
    [ula, ~]=p.perday(UlaTime,UlaSizes);
    p.Ula = ula;
    %Draw breakfast time
    obj.find_carbs(0); % To make sure the food for the day has been decided psedurandomly
    CarbsInMin = (round(obj.actualMealInfo(:,1)/obj.timeStep)*obj.timeStepInMin); %
    %CarbsInMin(1)=420;
    %CarbsInMin = round(CarbsInMin/obj.timeStepInMin)*obj.timeStepInMin; %Make the meals fit the time interval
    [y, yt]= p.perday(CarbsInMin,obj.actualMealInfo(:,2)); % Puts this into a day (Subtracting 360 to syncronize the clocks, real patient starts a 6 o clock, we start at 0 o clock(Max to make sure we eat on this day));
    
    breakfast_time = CarbsInMin(1);
    %Round to 5 minuts:
    %breakfast_time=round(breakfast_time/obj.timeStepInMin)*obj.timeStepInMin; 
    %Simulation time: 

    %SimtimeBreakfast = (breakfast_time-obj.timeStepInMin)/(60*24);
    SimtimeBreakfast = obj.actualMealInfo(1,1)-obj.timeStep;

    SimtimeRemaining = 1- SimtimeBreakfast;
    %p.Nd = round(Simtime*24*60)/(24*60); %In minutes
    %MealM is for medimum glycemic index
    if i==1
        count = count +1;
        
        %%%%%%%%%%%%%%%%%% Simulate until breakfast, and select dose %%%%%%%%%%%%%
        p.Nd = SimtimeBreakfast; %In minutes
        p.MealsM = y(1,[1:breakfast_time-obj.timeStepInMin]);
        
        
        
        %Take the SMBG measurement at day 1
        ym(count) = Xsim(35)*0.056 + ((1/5)*0.1*log(1+exp(5*(Xsim(35)*0.056-4.2)))+0.415)*randn;
        
        %Simulate until breakfast
        [~,~,Xsamp,Tsamp] = p.Simulation(obj.timeStepInMin);
        Xsim = [Xsim,Xsamp(:,2:end)];
        %Glucose value at the end of the simulatiomn:
        BG(count) = Xsim(35,end)*0.056;
        
        p.X0sim = p.Xend; % Update initial condition
        
        MealsMTemp = (length(p.MealsM)/obj.timeStepInMin);
        
        p.Nd = SimtimeRemaining; %In minutes

        if (obj.assumeCGM==1)
            for j = 1:MealsMTemp-1
                x_hat(j,:) = obj.estimate(0, (Xsamp(35,j)*0.056));
                obj.find_carbs(j*obj.timeStep);
            end
        else
            for j = 1:MealsMTemp-1
                x_hat(j,:) = obj.estimate(0, 0);
                obj.find_carbs(j*obj.timeStep);
            end
        end
            breakfastStates = obj.estimate(0, (Xsamp(35,MealsMTemp)*0.056));
            
            u(1) = obj.find_dose(i+obj.actualMealInfo(1,1)-obj.timeStep,breakfastStates);
        
        %%%% Adherence %%%%
        UlaTime = 0; 
        UlaSizes = u(1);
        [ula, ~]=p.perday(UlaTime,UlaSizes);
        p.Ula = ula;

        if(obj.trajectory == 1)
            obj.Linearization((Xsamp(35,MealsMTemp)*0.056));
        end
        
        day = floor(i);
        idx = day;
        if ((day >= obj.simDays) ||(idx > length(loadingMessages)))
            idx = length(loadingMessages);
        end
        day = num2str(day);
        waitbar(i/obj.simDays, obj.progressBar, "Day " + day + " of " + num2str(obj.simDays) + newline + loadingMessages(idx));
        if(getappdata(obj.progressBar, 'canceling'))
            delete(obj.progressBar)
            error("Canceled by user");
        end

        
        x_hat(MealsMTemp,:) = obj.estimate(u(1)/obj.timeStep, (Xsamp(35,MealsMTemp)*0.056)); %
        obj.find_carbs(MealsMTemp*obj.timeStep);

        %%%%%%%%%%%%%%%%%% Simulate rest of the day %%%%%%%%%%%%%
        
        p.MealsM = y(1,[breakfast_time+1-obj.timeStepInMin:end]);
        
        %Simulate rest if the day
        [~,~,Xsamp,Tsamp] = p.Simulation(obj.timeStepInMin);
        Xsim = [Xsim,Xsamp(:,2:end)];
        
        %Glucose value at the end of the simulation:
        BG(count) = Xsim(35,end)*0.056;
        
        p.X0sim = p.Xend;
        
        if (obj.assumeCGM==1)
        for j = MealsMTemp+1:numberOfIntervals
            x_hat(j,:) = obj.estimate(0, (Xsamp(35,j-MealsMTemp)*0.056));
            obj.find_carbs(j*obj.timeStep);
        end
        else
        for j = MealsMTemp+1:numberOfIntervals
            x_hat(j,:) = obj.estimate(0, 0);
            obj.find_carbs(j*obj.timeStep);
        end
        end

        
    elseif(i>=1)
        dayOffset = (numberOfIntervals*(i-1));
        
        
        countd = countd+1; 
        count = count+1;
        %SMBG measurement at the begining of the day:
        ym(count) = BG(count-1) + ((1/5)*0.1*log(1+exp(5*(BG(count-1)-4.2)))+0.415)*randn;

        %%%%%%%%%%%%%%%%%% Simulate until breakfast, and select dose %%%%%%%%%%%%%
        p.Nd = SimtimeBreakfast; %In minutes
        p.MealsM = y(1,[1:breakfast_time-obj.timeStepInMin]);
        dev = y(1,[1:breakfast_time-obj.timeStepInMin]);
        %Take the SMBG measurement
        ym(count) = Xsim(35)*0.056 + ((1/5)*0.1*log(1+exp(5*(Xsim(35)*0.056-4.2)))+0.415)*randn;
        %CGM measurememnts:
        ycgm = Xsamp(35,:)*0.056 + 0.42*randn(size(Xsamp(35,:))).*Xsamp(35,:)*0.056;
        
        %Simulate for 1 day until simtime
        [~,~,Xsamp,Tsamp] = p.Simulation(obj.timeStepInMin);
        Xsim = [Xsim,Xsamp(:,1:end)];
        %Glucose value at the end of the simulatiomn:
        BG(count) = Xsim(35,end)*0.056;
        
        
        p.X0sim = p.Xend;
        
        MealsMTemp = (length(p.MealsM)/obj.timeStepInMin);
        
        p.Nd = SimtimeRemaining; %In minutes
        

        if (obj.assumeCGM==1)
            for j = 1:MealsMTemp-1
                x_hat(dayOffset+j,:) = obj.estimate(0, (Xsamp(35,j)*0.056));
                obj.find_carbs(j*obj.timeStep);
            end
        else
            for j = 1:MealsMTemp-1
                x_hat(dayOffset+j,:) = obj.estimate(0, 0);
                obj.find_carbs(j*obj.timeStep);
            end
        end
        breakfastStates = obj.estimate(0, (Xsamp(35,MealsMTemp)*0.056));
            
        u(1) = obj.find_dose(i+obj.actualMealInfo(1,1)-obj.timeStep,breakfastStates);
        
        
        
        UlaTime = 0; 
        UlaSizes = u(1);
        [ula, ~]=p.perday(UlaTime,UlaSizes);
        p.Ula = ula;

        if(obj.trajectory == 1)
            obj.Linearization((Xsamp(35,MealsMTemp)*0.056));
        end
        
        x_hat(dayOffset + MealsMTemp,:) = obj.estimate(u(1)/obj.timeStep, (Xsamp(35,MealsMTemp)*0.056)); 
        obj.find_carbs(MealsMTemp*obj.timeStep);

        day = floor(i);
        idx = day;
        if ((day >= obj.simDays) ||(idx > length(loadingMessages)))
            idx = length(loadingMessages);
        end
        day = num2str(day);
        waitbar(i/obj.simDays, obj.progressBar, "Day " + day + " of " + num2str(obj.simDays) + newline + loadingMessages(idx));
        if(getappdata(obj.progressBar, 'canceling'))
            delete(obj.progressBar)
            error("Canceled by user");
        end

        %%%%%%%%%%%%%%%%%% Simulate rest of the day %%%%%%%%%%%%%
        
        

        dev = y(1,[breakfast_time+1-obj.timeStepInMin:end]);
        p.MealsM = y(1,[breakfast_time+1-obj.timeStepInMin:end]);
        
        %Simulate for 1 day until simtime
        [~,~,Xsamp,Tsamp] = p.Simulation(obj.timeStepInMin);
        Xsim = [Xsim,Xsamp(:,2:end)];
        
        
        %Glucose value at the end of the simulatiomn:
        BG(count) = Xsim(35,end)*0.056;
        
        p.X0sim = p.Xend;
        
        
        if (obj.assumeCGM==1)
        for j = MealsMTemp+1:numberOfIntervals
            x_hat(dayOffset+j,:) = obj.estimate(0, (Xsamp(35,j-MealsMTemp)*0.056));
            obj.find_carbs(j*obj.timeStep);
        end
        else
        for j = MealsMTemp+1:numberOfIntervals
            x_hat(dayOffset+j,:) = obj.estimate(0, 0);
            obj.find_carbs(j*obj.timeStep);
        end
        end
    
    
    end
end
obj.estimated = x_hat;
x = Xsim(35,:)*0.056;
x=x';
%time = 0:1:(length(Xsim(35,:))-1); %We take the time vector depending on size of Xsim
%since the amount of samples we simulate now is random.
%plot(time,Xsim(35,:)*0.056)
%xlabel('time in dyas')
%ylabel('Glucose~[mmol/l]')