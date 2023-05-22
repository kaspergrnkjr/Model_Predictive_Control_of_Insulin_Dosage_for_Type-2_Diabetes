function x_t = linearModel(obj, tspan, initialvalues)
    %linearModel    Simulates the linear model 
    
    x_t = zeros(length(tspan), obj.numberOfStates);
    x_t(1,:) = initialvalues;
    x_non = zeros(length(tspan), obj.numberOfStates);
    x_non(1, :) = initialvalues;
    x_est = zeros(length(tspan), obj.numberOfStates);
    x_est(1, :) = initialvalues;
    for i = 1:1:length(tspan) - 1
        
        %Simulate patient
        temp = obj.nonlinearModel([tspan(i) tspan(i)+obj.timeStep], x_non(i, :));
        x_non(i + 1,:) = temp(2, :);

        glucoseForEstimate = 0;

        if(mod(tspan(i) + obj.timeStep, 1)  <= obj.actualMealInfo(1,1) && mod(tspan(i) + 2*obj.timeStep, 1)  > obj.actualMealInfo(1,1))
            if(obj.trajectory)
                obj.Linearization(x_non(i,4));
                obj.findObserverGain();
            end
            
            glucoseForEstimate = obj.previousStates(4,1);
        end
        k1 = obj.A*(x_t(i, :) - obj.xss)' + obj.B*(obj.lastDose - obj.uss);
        k2 = obj.A*((x_t(i, :) + (obj.timeStep/2)*k1') - obj.xss)' + obj.B*(obj.lastDose - obj.uss);
        k3 = obj.A*((x_t(i, :) + (obj.timeStep/2)*k2') - obj.xss)' + obj.B*(obj.lastDose - obj.uss);
        k4 = obj.A*((x_t(i, :) + (obj.timeStep)*k3') - obj.xss)' + obj.B*(obj.lastDose - obj.uss);
        x_t(i + 1, :) = x_t(i, :) + (obj.timeStep/6*(k1+2*k2+2*k3+k4))'; %RK4
        
        if (obj.assumeCGM)
            glucoseForEstimate = obj.previousStates(4,1);
        end
        
        if(obj.estimateMethod ~= 0)
            x_est(i + 1,:) =  obj.estimate(obj.lastDose, glucoseForEstimate)';
        end
        
        % dose
        if(obj.estimateMethod == 0)
            %obj.lastDose = obj.find_dose(tspan(i), [obj.iscss obj.ipss obj.iss obj.gss]');
            obj.lastDose = obj.find_dose(tspan(i), obj.previousStates(:,1));
        else
            obj.lastDose = obj.find_dose(tspan(i), x_est(i,:)');
        end
        
    end
    obj.x_nonLin = x_non;
    obj.estimated = x_est;
end