function  x_hat = fullOrderObserver(obj, injection, measurement)
    
    if(measurement == 0)
        measurement = obj.C*obj.x_posterior;
    end

    if(obj.meal)
        inputss = [obj.uss;obj.dss];
        x_hat_dot = obj.A*(obj.x_posterior - obj.xss') + obj.B*([injection;obj.meanCarbs] - inputss) + obj.L*(obj.C*obj.x_posterior - measurement);
    else
        x_hat_dot = obj.A*(obj.x_posterior - obj.xss') + obj.B*(injection - obj.uss) + obj.L*(obj.C*obj.x_posterior - measurement);
    end

    x_hat = obj.x_posterior + obj.timeStep*x_hat_dot;

    obj.x_posterior = x_hat;

end