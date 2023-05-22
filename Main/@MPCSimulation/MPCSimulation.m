classdef MPCSimulation < handle

    properties
        controlMethod;
        RealPatient;
        timeStepInMin;
        stepsPerDay;
        simDays;
        progressBar;
        numOfSavedStates;
        initialValues;
        stochasticMeals;
        xss;
        molarWeightConst = 1000/180.1559;
        glucoseRateLimit;
        timeStep;
        mealMeans;
        actualMealInfo;
        mealStd;
        ipss;
        iscss;
        adherence;
        steadyStateGlucose;
        meal;
        Hp;
        Hu;
        PredDays;
        ContDays;
        rho;
        u_min;
        u_max;
        y_min;
        y_maxFasting;
        y_maxNonFasting
        referenceVector;
        maxGlucoseConstraintVector;
        Q_C;
        R_C;
        gss;
        iss;
        A;
        B;
        C;
        uss;
        p1;
        p3;
        p4;
        p5;
        p6;
        p7;
        previousStates;
        numberOfStates;
        pd;
        pv;
        p1_n;
        p3_n;
        p4_n;
        p5_n;
        p6_n;
        p7_n;
        pv_n;
        pd_n;
        lastDose;
        lastMeal;
        x_nonLin;
        trajectory;
        D1ss;
        D2ss;
        dss;
        assumeCGM;
        injectionHistory;
        estimateMethod;
        estimated;
        modelMismatch;
        meanCarbs;
        noisyMeasurements;
        carbLowerBound;
        maybeBad;
        NonAdherent;
        mealAdherence;
        referenceFasting;
        referenceNonFasting;
        NegativeGlucoseDeltaLimit;
        conservativeDose;
        

        %Particle filter
        
        estimate_dx;
        NumberOfParticles;
        pi_posterior;
        pi_prior;
        x_posterior;
        x_prior;
        processNoiseCovariance;
        measurementNoiseCovariance;


        % UKF 

        numberofSigmaPoints;
        P_x_posterior;          % Initial state covariance matrix
        P_x_prior;              % Initial state covariance matrix
        P_y_prior;
        P_xy_prior;
        K_gain;
        sigma_x_prior;
        sigma_x_posterior;
        sigma_y_prior;
        y_hat_prior;
        alpha;
        beta;
        kappa;
        lambda;
        Wm;
        Wc;
        eta;
        c;
        Rv;
        Rn;

        % Full order observer

        L;
        
        % For updating insulin sensitivity
        sensitivityUpdate;
        x_yesterday;
        u_yesterday;
        t_yesterday;
        meals_yesterday;
        A_yesterday;
        xss_yesterday;
        uss_yesterday;
        dss_yesterday;
        meas_today;


        %CASADI
        opti;
        optiDefined = 0;
        casadiParameters;
        U;
        init_states;
    end

    methods
        ConsumedCarbs = find_carbs(obj, t)
        DoseToInject = find_dose(obj, t, states)
        ShiftedArray = shiftAndAppend(obj, array, newColumn)
        SimulationNonLinearWithMeal = nonlinearModelMeal(obj, tspan, initialvalues)
        SimulationNonLinear = nonlinearModel(obj, tspan, initialvalues)
        SimulationLinear = linearModel(obj, tspan, initialvalues)
        DoseNoMeal = MPCNoMeal(obj, t, states)
        DoseWithMeal = MPCWithMeal(obj, t, states)
        x_hat = estimate(obj, injection, measurement)
        x_hat = UKF(obj, injection, measurement)
        CASADIDose = CASADIDoseWithMeal(obj, t, initialvalues) %initial values not used
        x_hat = fullOrderObserver(obj, injection, measurement)
        x_hat = particlefilter(obj, injection, measurement)
        SimulationRealPatient = RealPatientSim(obj,timespan, initialvalues)
        Linearization(obj, x_non)
        findObserverGain(obj)
        updateSensitivity(obj, currentTime, measurement)
        saveStuffForSensitivityUpdate(obj, states, t)
    end

end

