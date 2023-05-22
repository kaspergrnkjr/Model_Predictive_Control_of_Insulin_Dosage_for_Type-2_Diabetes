function x_t = nonlinearModelMeal(obj, tspan, initialvalues)
    
    x_t = zeros(length(tspan), obj.numberOfStates);
    x_t(1,:) = initialvalues;
    x_est = zeros(length(tspan), obj.numberOfStates);
    x_est(1, :) = initialvalues;
    for i = 1:1:length(tspan) - 1
        
        obj.previousStates = obj.shiftAndAppend(obj.previousStates, x_t(i, :));
        if(obj.controlMethod == 1)
            u = obj.lastDose;
            d = obj.lastMeal;
        elseif (obj.controlMethod == 2)
            u = obj.find_dose(tspan(i), x_est(i,:));
            d = obj.find_carbs(tspan(i));
            obj.lastMeal = d;
            obj.lastDose = u;
            if (u ~= 0 || obj.assumeCGM)
                glucoseForEstimate = obj.previousStates(4,1);
            else
                glucoseForEstimate = 0;                         % Made Changes, if no new measurement
            end
            x_est(i + 1,:) =  obj.estimate(obj.lastDose, glucoseForEstimate)';
            
        else
            u = obj.find_dose(tspan(i), obj.previousStates(:, 1));
            d = obj.find_carbs(tspan(i));
            if (u ~= 0 || obj.assumeCGM)
                glucoseForEstimate = obj.previousStates(4,1);
            else
                glucoseForEstimate = 0;                         % Made Changes, if no new measurement
            end
            x_est(i + 1,:) =  obj.estimate(obj.lastDose, glucoseForEstimate)';
            
        end
        f = @(x,inj,m) [((1/obj.p1_n)*inj)-((1/obj.p1_n)*x(1));
            ((1/obj.p1_n)*x(1))-((1/obj.p1_n)*x(2));
            (obj.p3_n*(x(2)+obj.p7_n*x(4)))-(obj.p3_n*x(3));
            -((obj.p5_n+obj.p4_n*x(3))*x(4))+obj.p6_n+(x(6)/(obj.pv_n*obj.pd_n));
            (obj.molarWeightConst)*m-(1/obj.pd_n)*x(5);
            (1/obj.pd_n)*x(5)-(1/obj.pd_n)*x(6)]; 

        k1 = f(x_t(i, :), u,d);
        k2 = f(x_t(i, :) + (obj.timeStep/2)*k1', u,d);
        k3 = f(x_t(i, :) + (obj.timeStep/2)*k2', u,d);
        k4 = f(x_t(i, :) + obj.timeStep*k3', u,d);
        x_t(i + 1, :) = x_t(i, :) + (obj.timeStep/6*(k1+2*k2+2*k3+k4))'; %RK4
    end

    obj.estimated = x_est;
end