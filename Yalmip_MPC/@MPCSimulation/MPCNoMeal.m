function DoseNoMeal = MPCNoMeal(obj, t, states, dt)
%MPCNomeal  Returns a dose based on the linear model prediction without
%meal
%
%MPCNoMeal(t, states, dt) where t is the current time, states is the
%previous states and dt is the timestep
    yalmip('clear')

    u = sdpvar(1, obj.PredDays);
    p = sdpvar(3,1);
    constraints = [];
    objective = 0;
    
    
    x = states(:,1);
    fastingStates = x;
    y = x;
    

    for k = obj.stepsPerDay:obj.Hp + (obj.stepsPerDay - 1)
        idx = k - obj.stepsPerDay + 1;
        if(mod(k,obj.stepsPerDay)==0)
            if idx ~= 1
                fastingStates = [fastingStates x];
                constraints = [constraints, fastingStates(4, k/obj.stepsPerDay - 1) - fastingStates(4, k/obj.stepsPerDay) <= obj.NegativeGlucoseDeltaLimit + p(3,1)];
            end
            if (k - obj.stepsPerDay < obj.Hu)
                insulinIdx = u(k/obj.stepsPerDay);
            else
                insulinIdx = u(end);
            end
            xd = obj.A*(x - obj.xss')+ obj.B*(insulinIdx - obj.uss);
            x = x + obj.timeStep*xd;
            objective = objective + (x(4) - obj.referenceVector(idx))*obj.Q_C*(x(4) - obj.referenceVector(idx)) + insulinIdx*obj.R_C*insulinIdx + p(1, 1)*obj.rho + p(2,1)*obj.rho + p(3,1)*obj.rho;
            constraints = [constraints, obj.u_min <= insulinIdx <= obj.u_max];
        else
            xd = obj.A*(x - obj.xss') +obj.B*(insulinIdx - obj.uss);
            x = x + obj.timeStep*xd;
            objective = objective + (x(4) - obj.referenceVector(idx))*obj.Q_C*(x(4) - obj.referenceVector(idx)) + p(1, 1)*obj.rho + p(2,1)*obj.rho + p(3,1)*obj.rho;
            
        end
        constraints = [constraints, x(4) >= obj.y_min - p(2,1)];
        constraints = [constraints, x(4) <= obj.maxGlucoseConstraintVector(idx) + p(1,1)];
        
        y = [y x];
    end

    opt = sdpsettings('verbose', 0);
    optimize([constraints, p(1,1) >= 0, p(2,1) >= 0, p(3,1) >= 0],objective, opt);

    tempu = value(u);
    DoseNoMeal = tempu(find(tempu, 1, 'first'));

end