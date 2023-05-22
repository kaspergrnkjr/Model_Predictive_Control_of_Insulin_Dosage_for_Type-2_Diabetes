function dose = find_dose(obj, t, states)
% find_dose     [FIX with class] Give you the optimal dose (MPC or casadi)
% These includes H_p, H_u, Q,R together with timestep and perferable the
% meals as well.
% 
% Used as wantedDose = (t,states,dt), where t is the given time, states is
% the given state for the patient and dt is the discrete time interval
% length.
    dose = 0;
    injectionFlag = 0;
    loadingMessages = ["Getting a mug" "Pouring coffee" "Sipping.." "Too hot!" "Blowing.." "Sipping..." "Perfect!" "Ahhh..." "Jokke er grem"];
    if(obj.controlMethod == 0)
        if(round(mod(t + obj.timeStep,1),10) <= round(obj.actualMealInfo(1,1),10) && round(mod(t + 2*obj.timeStep,1),10)  > round(obj.actualMealInfo(1,1),10))            
            if(unifrnd(0,1) <= obj.adherence)
                if t < 7
                    dose = (40/7*t)/obj.timeStep;
                else
                    dose = 40/obj.timeStep;
                end
                dose = 0;
            end
        end

    elseif(obj.controlMethod == 1)
        if(round(mod(t + obj.timeStep,1),10) <= round(obj.actualMealInfo(1,1),10) && round(mod(t + 2*obj.timeStep,1),10)  > round(obj.actualMealInfo(1,1),10))
            injectionFlag = 1;
            % Sensitivity update
            obj.updateSensitivity(t);
            obj.saveStuffForSensitivityUpdate(states, t);
            if(unifrnd(0,1) <= obj.adherence)
               

                if(obj.meal)
                    % Finding the dose with MPC
                    dose = obj.MPCWithMeal(t, states);
                else
                  
                    % Finding the dose with MPC
                    dose = obj.MPCNoMeal(t, states);
                end  
            end
        end

    elseif (obj.controlMethod == 2)
        if(round(mod(t + obj.timeStep,1),10) <= round(obj.actualMealInfo(1,1),10) && round(mod(t + 2*obj.timeStep,1),10)  > round(obj.actualMealInfo(1,1),10))
            injectionFlag = 1;
            % Sensitivity update
            obj.updateSensitivity(t);
            obj.saveStuffForSensitivityUpdate(states, t);
            if(unifrnd(0,1) <= obj.adherence)

                dose = obj.CASADIDoseWithMeal(t,states);
            end
        end
    end
    
    if(obj.NonAdherent==1)
        NonAdherentDose=0;
        while(NonAdherentDose < 0.25)
            NonAdherentDose=abs(normrnd(0,0.5))*-1+1;
        end
        dose = dose*NonAdherentDose;
    end

    if(obj.conservativeDose)
        if(t < 2) % First day
            dose = dose*0.25; % Only inject 1/4 of the dose the first day
        elseif(t < 3) % Second day
            dose = dose*0.5; % Only inject half of the dose the second day
        elseif(t < 4) % Third day
            dose = dose*0.75; % Only inject 3/4 of the dose the third day
        end
    end

    day = floor(t);
    idx = day;
    if ((day >= obj.simDays) ||(idx > length(loadingMessages)))
        idx = length(loadingMessages);
    end
    day = num2str(day);
    waitbar(t/obj.simDays, obj.progressBar, "Day " + day + " of " + num2str(obj.simDays) + newline + loadingMessages(idx));
    if(getappdata(obj.progressBar, 'canceling'))
        delete(obj.progressBar)
        error("Canceled by user");
    end

    obj.injectionHistory(int16((t-1)*obj.stepsPerDay) + 1) = dose;
    obj.lastDose = dose;
    
    if(dose ~= 0) % Saving dose for sensitivity update
        obj.u_yesterday = dose;
    end

    if(obj.RealPatient == 1)
        dose = dose*obj.timeStep;
    end
end