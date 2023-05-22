function x_hat = particleFilter(obj,injection, measurement)

    if measurement > 0
        
        % Measurement Update Start
        obj.pi_posterior = obj.pi_prior .* normpdf( (measurement - obj.x_prior(4, :)) / obj.measurementNoiseCovariance ); % Z = (x-mu)/sigma
        obj.pi_posterior = max(obj.pi_posterior, 0.000001);
        obj.pi_posterior = obj.pi_posterior / sum(obj.pi_posterior);
    
        obj.x_posterior = obj.x_prior;
        % Measurement Update End


    else
        obj.x_posterior = obj.x_prior;
    end

    % Time Update Start
    
    % function start
    if(obj.meal)
        obj.estimate_dx(1, :) = (1/obj.p1)*(injection - obj.x_posterior(1, :));
        obj.estimate_dx(2, :) = (1/obj.p1)*(obj.x_posterior(1, :) - obj.x_posterior(2, :));
        obj.estimate_dx(3, :) = obj.p3*(obj.x_posterior(2, :) + obj.p7*obj.x_posterior(4, :)) - obj.p3*obj.x_posterior(3, :);
        Ra = obj.x_posterior(6, :)/(obj.pv*obj.pd);
        obj.estimate_dx(4, :) = -(obj.p5+obj.p4*obj.x_posterior(3, :)).*obj.x_posterior(4, :)+obj.p6 + Ra;
        obj.estimate_dx(5, :) = obj.molarWeightConst*obj.meanCarbs - 1/obj.pd*obj.x_posterior(5, :);
        obj.estimate_dx(6, :) = 1/obj.pd*obj.x_posterior(5, :) - 1/obj.pd*obj.x_posterior(6, :);
    else
        obj.estimate_dx(1, :) = (1/obj.p1)*(injection - obj.x_posterior(1, :));
        obj.estimate_dx(2, :) = (1/obj.p1)*(obj.x_posterior(1, :) - obj.x_posterior(2, :));
        obj.estimate_dx(3, :) = obj.p3*(obj.x_posterior(2, :) + obj.p7*obj.x_posterior(4, :)) - obj.p3*obj.x_posterior(3, :);
        obj.estimate_dx(4, :) = -(obj.p5 + obj.p4*obj.x_posterior(3, :)) .* obj.x_posterior(4, :) + obj.p6;
    end
    % function end

    obj.x_prior = obj.x_posterior + obj.estimate_dx*obj.timeStep + normrnd(0,obj.processNoiseCovariance, [obj.numberOfStates, obj.NumberOfParticles]); % From the system equation (1);

    obj.pi_prior= 1/obj.NumberOfParticles;
    % Time Update End

    % Calculate x_hat
    x_hat = obj.x_prior * obj.pi_posterior';

end