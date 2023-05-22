function x_hat = UKF(obj, injection, measurement)

    % Time Update Start
    
    % Sigma Posterior Definition Start
    obj.sigma_x_posterior(:, 1) = obj.x_posterior(:, 1);
    obj.sigma_x_posterior(:, 2:obj.numberOfStates+1) = obj.x_posterior(:, 1) + obj.eta * chol(obj.P_x_posterior);
    obj.sigma_x_posterior(:, obj.numberOfStates+2:2*obj.numberOfStates+1) = obj.x_posterior(:, 1) - obj.eta * chol(obj.P_x_posterior);
    % Sigma Posterior Definition End
    
    % Function f Start
    Ra = 0;

    if(obj.meal)
        Ra = obj.sigma_x_posterior(6, :)/(obj.pv*obj.pd);
        obj.estimate_dx(5, :) = obj.molarWeightConst*obj.meanCarbs - 1/obj.pd*obj.sigma_x_posterior(5, :);
        obj.estimate_dx(6, :) = 1/obj.pd*obj.sigma_x_posterior(5, :) - 1/obj.pd*obj.sigma_x_posterior(6, :);
    end
    
    obj.estimate_dx(1, :) = (1/obj.p1)*(injection - obj.sigma_x_posterior(1, :));
    obj.estimate_dx(2, :) = (1/obj.p1)*(obj.sigma_x_posterior(1, :) - obj.sigma_x_posterior(2, :));
    obj.estimate_dx(3, :) = obj.p3*(obj.sigma_x_posterior(2, :) + obj.p7*obj.sigma_x_posterior(4, :)) - obj.p3*obj.sigma_x_posterior(3, :);
    obj.estimate_dx(4, :) = -(obj.p5 + obj.p4*obj.sigma_x_posterior(3, :)) .* obj.sigma_x_posterior(4, :) + obj.p6 + Ra;

    
    
    obj.sigma_x_prior(:, :) = obj.sigma_x_posterior + obj.estimate_dx*obj.timeStep;% + sum(normrnd(0, obj.Rv), 2);
    % Function f End
    
    obj.x_prior = ( obj.Wm * obj.sigma_x_prior')';
    
    obj.P_x_prior = obj.P_x_prior * 0;
    for loop_Px=1:obj.numberofSigmaPoints
        obj.P_x_prior = obj.P_x_prior + obj.Wc(1, loop_Px) * (obj.sigma_x_prior(:, loop_Px) - obj.x_prior) * (obj.sigma_x_prior(:, loop_Px) - obj.x_prior)';
    end
    obj.P_x_prior = obj.P_x_prior + obj.Rv;
    
    obj.sigma_y_prior(1, :) = obj.sigma_x_prior(4, :);%+ normrnd(0, obj.Rn);
    
    obj.y_hat_prior = obj.Wm * obj.sigma_y_prior';
    % Time Update End
    
    if measurement > 0            % Checks if there is new measurement
        % Measurement Update Start
        obj.P_y_prior = obj.P_y_prior * 0;
        for loop_Py=1:obj.numberofSigmaPoints
            obj.P_y_prior = obj.P_y_prior + obj.Wc(1, loop_Py) * (obj.sigma_y_prior(:, loop_Py) - obj.y_hat_prior) * (obj.sigma_y_prior(:, loop_Py) - obj.y_hat_prior)';
        end
        obj.P_y_prior = obj.P_y_prior + obj.Rn;
        
        
        obj.P_xy_prior = obj.P_xy_prior * 0;
        for loop_Pxy=1:obj.numberofSigmaPoints
            obj.P_xy_prior = obj.P_xy_prior + obj.Wc(1, loop_Pxy) * (obj.sigma_x_prior(:, loop_Pxy) - obj.x_prior) * (obj.sigma_y_prior(:, loop_Pxy) - obj.y_hat_prior)';
        end
        
        obj.K_gain = obj.P_xy_prior / obj.P_y_prior;
        
        obj.x_posterior(:, 1) = obj.x_prior + obj.K_gain * (measurement - obj.y_hat_prior);
        
        obj.P_x_posterior = obj.P_x_prior - obj.K_gain * obj.P_y_prior * obj.K_gain';
    
        x_hat = obj.x_posterior;
        % Measurement Udate End
    else                                % Runs if there is no new measurement
        obj.x_posterior = obj.x_prior;
        x_hat = obj.x_posterior;
    end

end