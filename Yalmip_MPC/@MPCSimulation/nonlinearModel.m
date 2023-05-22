function x_t = nonlinearModel(obj, tspan, initialvalues)
%nonlinearModel     Simulate the nonlinear model with meals
% Uses the input of tspan and the initialvalues of the simulation
% Returns the states of the full simulation
    
    x_t = zeros(length(tspan), obj.numberOfStates);
    x_t(1,:) = initialvalues;
    x_est = zeros(length(tspan), obj.numberOfStates);
    x_est(1, :) = initialvalues;
    for i = 1:1:length(tspan) - 1
        
        obj.previousStates = obj.shiftAndAppend(obj.previousStates, x_t(i, :));
        if(obj.controlMethod == 1)
            u = obj.lastDose;
        elseif(obj.controlMethod == 2)
            u = obj.find_dose(tspan(i), x_est(i,:));
            obj.lastDose = u;
            if (u ~= 0 || obj.assumeCGM)
                glucoseForEstimate = obj.previousStates(4,1);
            else
                glucoseForEstimate = 0; % Made Changes, if no new measurement
            end
            x_est(i + 1,:) =  obj.estimate(obj.lastDose, glucoseForEstimate)';
        else
            u = obj.find_dose(tspan(i), obj.previousStates(:,1)); % If no control method is chosen, the hard coded doses are used
        end
        dxdt(1) = (1/obj.p1_n)*(u-x_t(i,1));
        dxdt(2) = (1/obj.p1_n)*(x_t(i,1)-x_t(i,2));
        dxdt(3) = obj.p3_n*(x_t(i, 2)+obj.p7_n*x_t(i,4))-obj.p3_n*x_t(i,3);
        dxdt(4) = -(obj.p5_n+obj.p4_n*x_t(i,3))*x_t(i,4)+obj.p6_n;
        x_t(i + 1, :) = x_t(i, :) + dxdt*obj.timeStep;
        
    end
    obj.estimated = x_est;
end