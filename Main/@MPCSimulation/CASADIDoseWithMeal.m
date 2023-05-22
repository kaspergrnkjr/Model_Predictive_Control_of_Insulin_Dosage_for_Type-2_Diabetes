function dose = CASADIDoseWithMeal(obj, t, initialvalues)
%To do 

dose = 0;

localMealMean = obj.mealMeans;
localMealMean(1,1) = obj.actualMealInfo(1,1);

if(~obj.optiDefined)
    obj.opti = casadi.Opti(); % Let CasADI know we want the opti object to be an optimization problem
    options = struct();
    options.ipopt.print_level = 0;
    options.print_time = 0;

    obj.casadiParameters = obj.opti.parameter(1,8);
    obj.init_states = obj.opti.parameter(obj.numberOfStates,1);
    X = obj.opti.variable(obj.numberOfStates,obj.Hp + 1); % State declarations
    obj.U = obj.opti.variable(1,obj.ContDays); % Control variable declaration
    p = obj.opti.variable(3,1);
    obj.opti.minimize(((X(4,:)-[obj.referenceVector 7])*obj.Q_C*(X(4,:)-[obj.referenceVector 7])')+(obj.U(1,:)*obj.R_C*obj.U(1,:)')+ p(1, 1)*obj.rho + p(2,1)*obj.rho + p(3,1)*obj.rho);
    fastingStates = X(:, 1); 

    d = zeros(1,obj.Hp);
    for j = 1:1:obj.Hp  
        for i = 1:1:size(localMealMean, 1)
            if(round(mod(t + obj.timeStep*(j - 1),1),10) <= round(localMealMean(i, 1),10) && round(mod(t + obj.timeStep*j,1),10) > round(localMealMean(i, 1),10))
                d(1, j) = localMealMean(i, 2)/obj.timeStep;
                break;
            else
                d(1, j) = 0;
            end
        end
    end



    if(obj.meal==1) %Define functions and integration scheme, depending on meals or no meals.
        f = @(x,u,m) [((1/obj.casadiParameters(1,1))*u)-((1/obj.casadiParameters(1,1))*x(1));
                    ((1/obj.casadiParameters(1,1))*x(1))-((1/obj.casadiParameters(1,1))*x(2));
                    (obj.casadiParameters(1,2)*(x(2)+obj.casadiParameters(1,6)*x(4)))-(obj.casadiParameters(1,2)*x(3));
                     -((obj.casadiParameters(1,4)+obj.casadiParameters(1,3)*x(3))*x(4))+obj.casadiParameters(1,5)+(x(6)/(obj.casadiParameters(1,8)*obj.casadiParameters(1,7)));
                     (obj.molarWeightConst)*m-(1/obj.casadiParameters(1,7))*x(5); (1/obj.casadiParameters(1,7))*x(5)-(1/obj.casadiParameters(1,7))*x(6)]; % dx/dt = f(x,u)
        
        for k = obj.stepsPerDay:obj.Hp + (obj.stepsPerDay - 1)
            idx = k - obj.stepsPerDay + 1;
            
            if(mod(k,obj.stepsPerDay)==0)
                if idx ~= 1
                    fastingStates = [fastingStates X(:, idx)];
                    obj.opti.subject_to(fastingStates(4, k/obj.stepsPerDay - 1) - fastingStates(4, k/obj.stepsPerDay) <= obj.NegativeGlucoseDeltaLimit + p(3,1));
                end

                if (k - obj.stepsPerDay < obj.Hu)
                    insulinIdx = obj.U(k/obj.stepsPerDay);
                else
                    insulinIdx = obj.U(end);
                end
                k1 = f(X(:,idx), insulinIdx,d(idx));
                k2 = f(X(:,idx)+(obj.timeStep/2)*k1, insulinIdx,d(idx));
                k3 = f(X(:,idx)+(obj.timeStep/2)*k2, insulinIdx,d(idx));
                k4 = f(X(:,idx)+obj.timeStep*k3, insulinIdx,d(idx));
                x_next = X(:,idx) + obj.timeStep/6*(k1+2*k2+2*k3+k4); %RK4
                obj.opti.subject_to(X(:,idx + 1)==x_next); 
                
            else
                k1 = f(X(:,idx), 0, d(idx));
                k2 = f(X(:,idx)+(obj.timeStep/2)*k1, 0,d(idx));
                k3 = f(X(:,idx)+(obj.timeStep/2)*k2, 0,d(idx));
                k4 = f(X(:,idx)+obj.timeStep*k3,0, d(idx));
                x_next = X(:,idx) + obj.timeStep/6*(k1+2*k2+2*k3+k4); %RK4
                obj.opti.subject_to(X(:,idx + 1)==x_next); 
                
            end
            obj.opti.subject_to(X(4,idx)<= obj.maxGlucoseConstraintVector(idx) + p(1,1));
        end


        obj.opti.subject_to(X(6,1)==obj.init_states(6)); %Update meals when these are included.
        obj.opti.subject_to(X(5,1)==obj.init_states(5));
    else
        f = @(x,u) [((1/obj.casadiParameters(1,1))*u)-((1/obj.casadiParameters(1,1))*x(1));((1/obj.casadiParameters(1,1))*x(1))-((1/obj.casadiParameters(1,1))*x(2));(obj.casadiParameters(1,2)*(x(2)+obj.casadiParameters(1,6)*x(4)))-(obj.casadiParameters(1,2)*x(3)); -((obj.casadiParameters(1,4)+obj.casadiParameters(1,3)*x(3))*x(4))+obj.casadiParameters(1,5)]; % dx/dt = f(x,u)


        for k = obj.stepsPerDay:obj.Hp + (obj.stepsPerDay - 1)
            idx = k - obj.stepsPerDay + 1;
            
            if(mod(k,obj.stepsPerDay)==0)
                if idx ~= 1
                    fastingStates = [fastingStates X(:, idx)];
                    obj.opti.subject_to(fastingStates(4, k/obj.stepsPerDay - 1) - fastingStates(4, k/obj.stepsPerDay) <= obj.NegativeGlucoseDeltaLimit + p(3,1));
                end
                if (k - obj.stepsPerDay < obj.Hu)
                    insulinIdx = obj.U(k/obj.stepsPerDay);
                else
                    insulinIdx = obj.U(end);
                end
                k1 = f(X(:,idx), insulinIdx);
                k2 = f(X(:,idx)+(obj.timeStep/2)*k1, insulinIdx);
                k3 = f(X(:,idx)+(obj.timeStep/2)*k2, insulinIdx);
                k4 = f(X(:,idx)+obj.timeStep*k3, insulinIdx);
                x_next = X(:,idx) + obj.timeStep/6*(k1+2*k2+2*k3+k4); %RK4
                obj.opti.subject_to(X(:,idx+1)==x_next); 
                
            else
                k1 = f(X(:,idx), 0);
                k2 = f(X(:,idx)+(obj.timeStep/2)*k1, 0);
                k3 = f(X(:,idx)+(obj.timeStep/2)*k2, 0);
                k4 = f(X(:,idx)+obj.timeStep*k3,0);
                x_next = X(:,idx) + obj.timeStep/6*(k1+2*k2+2*k3+k4); %RK4
                obj.opti.subject_to(X(:,idx+1)==x_next);
                
            end
            obj.opti.subject_to(X(4,idx)<= obj.maxGlucoseConstraintVector(idx) + p(1,1));
        end

    end

    %Constraints on path of X and U variables
    obj.opti.subject_to(0<=obj.U(:)<=obj.u_max);
    obj.opti.subject_to(0<=X(1,:));
    obj.opti.subject_to(0<=X(2,:));
    obj.opti.subject_to(0<=X(3,:));
    obj.opti.subject_to(obj.y_min - p(2,1)<=X(4,:));
    obj.opti.subject_to(0 <= p(1,1));
    obj.opti.subject_to(0 <= p(2,1));
    obj.opti.subject_to(0 <= p(3,1));
    
    %Intial conditions
    
    obj.opti.subject_to(X(4,1)==obj.init_states(4));
    obj.opti.subject_to(X(3,1)==obj.init_states(3));
    obj.opti.subject_to(X(2,1)==obj.init_states(2));
    obj.opti.subject_to(X(1,1)==obj.init_states(1));
      
    obj.opti.solver('ipopt', options); % set solver method, ipopt = interior point optimization
    obj.optiDefined = 1;
end

obj.opti.set_value(obj.casadiParameters, [obj.p1 obj.p3 obj.p4 obj.p5 obj.p6 obj.p7 obj.pd obj.pv]);
obj.opti.set_value(obj.init_states, initialvalues);

sol = obj.opti.solve();   % actually solve the problem
dose = sol.value(obj.U(1,1));

end