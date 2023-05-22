function x_hat = estimate(obj,injection, measurement)

    x_hat = obj.previousStates(:,1);

    if(obj.noisyMeasurements && measurement > 0)
        measurement = measurement + normrnd(0,obj.measurementNoiseCovariance);
    end

    % Saving the measurement for sensitivity update
    if (measurement ~= 0)
        obj.meas_today = measurement;
    end

    if(obj.estimateMethod == 1)
        x_hat = obj.particleFilter(injection, measurement);
    elseif(obj.estimateMethod == 2)
        x_hat = obj.UKF(injection, measurement);
    elseif(obj.estimateMethod == 3)
        x_hat = obj.fullOrderObserver(injection, measurement);
    end
    x_hat = max(0, x_hat);

end