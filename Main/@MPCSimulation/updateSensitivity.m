function updateSensitivity(obj, currentTime)
    if(currentTime >= 2 && obj.sensitivityUpdate == 1)
        x_end = obj.x_yesterday;
        tspan = obj.t_yesterday:obj.timeStep:currentTime;
        switch obj.controlMethod
            case {0, 2}
                if(obj.meal)
                    f = @(x,u,m) [((1/obj.p1)*u)-((1/obj.p1)*x(1));((1/obj.p1)*x(1))-((1/obj.p1)*x(2));(obj.p3*(x(2)+obj.p7*x(4)))-(obj.p3*x(3)); -((obj.p5+obj.p4*x(3))*x(4))+obj.p6+(x(6)/(obj.pv*obj.pd)); (obj.molarWeightConst)*m-(1/obj.pd)*x(5); (1/obj.pd)*x(5)-(1/obj.pd)*x(6)]; % dx/dt = f(x,u)
                    for i = 1:1:length(tspan) - 1
                        % Finding the insulin injection
                        u = 0;
                        if (i == 1)
                            u = obj.u_yesterday; % The first step of this simulation is the injection time
                        end
                        
                        % Finding the meal intake
                        m = 0;
                        for j = 1:1:size(obj.actualMealInfo, 1)
                            if(round(mod(tspan(i),1),10) <= round(obj.actualMealInfo(j, 1),10) && round(mod(tspan(i+1),1),10)  > round(obj.actualMealInfo(j, 1),10))
                                m = obj.actualMealInfo(j, 2);
                            end
                        end
                        
                        % Simulating the nonlinear model with meal
                        k1 = f(x_end, u, m);
                        k2 = f(x_end + (obj.timeStep/2)*k1, u, m);
                        k3 = f(x_end + (obj.timeStep/2)*k2, u, m);
                        k4 = f(x_end + obj.timeStep*k3, u, m);
                        x_end = x_end + obj.timeStep/6*(k1+2*k2+2*k3+k4); %RK4
                    end
                else
                    f = @(x,u) [((1/obj.p1)*u)-((1/obj.p1)*x(1));((1/obj.p1)*x(1))-((1/obj.p1)*x(2));(obj.p3*(x(2)+obj.p7*x(4)))-(obj.p3*x(3)); -((obj.p5+obj.p4*x(3))*x(4))+obj.p6]; % dx/dt = f(x,u)
                    for i = 1:1:length(tspan) - 1
                        % Finding the insulin injection
                        u = 0;
                        if (i == 1)
                            u = obj.u_yesterday; % The first step of this simulation is the injection time
                        end
                        
                        % Simulating the nonlinear model without meal
                        k1 = f(x_end, u);
                        k2 = f(x_end+(obj.timeStep/2)*k1, u);
                        k3 = f(x_end+(obj.timeStep/2)*k2, u);
                        k4 = f(x_end+obj.timeStep*k3, u);
                        x_end = x_end + obj.timeStep/6*(k1+2*k2+2*k3+k4); %RK4
                    end
                end
            case 1
                if(obj.meal)
                    for i = 1:1:length(tspan) - 1
                        % Finding the insulin injection
                        u = 0;
                        if (i == 1)
                            u = obj.u_yesterday; % The first step of this simulation is the injection time
                        end
                        
                        % Finding the meal intake
                        m = 0;
                        for j = 1:1:size(obj.actualMealInfo, 1)
                            if(round(mod(tspan(i),1),10) <= round(obj.actualMealInfo(j, 1),10) && round(mod(tspan(i+1),1),10)  > round(obj.actualMealInfo(j, 1),10))
                                m = obj.actualMealInfo(j, 2);
                            end
                        end
                        
                        input = [u; m];
                        inputss = [obj.uss_yesterday;obj.dss_yesterday];
                        
                        % Simulate linear model with meal
                        xd = obj.A_yesterday*(x_end' - obj.xss_yesterday)' + obj.B*(input - inputss);
                        x_end = x_end + obj.timeStep*xd;
                    end
                else
                    for i = 1:1:length(tspan) - 1
                        % Finding the insulin injection
                        u = 0;
                        if (i == 1)
                            u = obj.u_yesterday; % The first step of this simulation is the injection time
                        end
                        
                        % Simulate linear model without meal
                        xd = obj.A_yesterday*(x_end' - obj.xss_yesterday)' + obj.B*(u - obj.uss_yesterday);
                        x_end = x_end + obj.timeStep*xd;
                    end
                end
            otherwise
        end
        deviation = x_end(4) - obj.meas_today;
        gain = 0.2;
        obj.p4 = obj.p4 + gain*deviation;
        obj.p4 = max(obj.p4, 0.2);
        fprintf("Deviation: %f p4: %f \n", deviation, obj.p4);
        %writematrix([gain obj.p4 deviation], ['p4-update/gain', num2str(gain), '.csv'],'Delimiter',',','WriteMode','append');
    end
end