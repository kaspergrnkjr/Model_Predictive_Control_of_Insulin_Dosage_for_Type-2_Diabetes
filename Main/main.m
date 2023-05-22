
%% Clean up and workspace setup
clc
clear
close all
try
    delete(findall(0))
catch

end
addpath(genpath('YALMIP-master'));
addpath(genpath('casadi'));
addpath(genpath('aau-t2d-simulator_Model4'))
import casadi.*

warning('off', 'MATLAB:structOnObject')

%% Setup

MAC = MPCSimulation;

MAC.progressBar = waitbar(0, 'Simulating..', 'CreateCancelBtn', 'setappdata(gcbf, ''canceling'', 1)');
setappdata(MAC.progressBar, 'canceling', 0)
set(MAC.progressBar, 'Position', [550 400 360 100])
% Basic simulation setup
MAC.controlMethod = 1; % 0 = no control - only simulation, 1 = linear MPC, 2 = nonlinear control (CasADi)
MAC.meal = 1; % 0 = no meal included in the simulation, 1 = meals are included in the simulation
MAC.RealPatient = 1; % 0 = the big model isnt used, 1 = the big model is used
MAC.simDays = 2; % number of days to simulate
MAC.adherence = 1; % chance of remembering to inject
MAC.trajectory = 1; % 0 = linearize only once upon simulation start, 1 = a new linearization is performed once a day
MAC.steadyStateGlucose = 5; % This is the linearization point if trajectory is 0
MAC.numOfSavedStates = 5; % This is how many of the previous states is carried along in the simulation. So e.g. state at time k-5 is always available
MAC.initialValues = [0 0 17.6 10.5]; % States at the start of the simulation
MAC.timeStepInMin = 5; % This determines the how long one time step is in minutes
MAC.timeStep = MAC.timeStepInMin/(24*60); % Calculates timestep from timestep in min input
MAC.stochasticMeals = 0; % 0 = meals are deterministic and equal to meal means, 1 = meals are drawn from a random distribution based on meanMeans
MAC.assumeCGM = 0; % 0 = if we only get a measurement once a day(SMBG), 1 = we know the blood glucose at all times (CGM)
MAC.NegativeGlucoseDeltaLimit = 20; % This parameter sets the maximum mmol/L the glucose concentration is allowed to drop from day to day, (0 if no limit)
MAC.estimateMethod = 2; % 0 = none, 1 = particle filter, 2 = UKF, 3 = full order observer
MAC.modelMismatch = 0; % 0 = the model parameters in the patient simulation is the same as the model parameters used for MPC and estimation, 1 = the model parameters in the (MPC, estimation) and patient simulation differ
MAC.noisyMeasurements = 1; % 0 = No noise is added to the states taken from the simulation, 1 = Noise is added to the values passed to the estimator.
MAC.carbLowerBound = 250; %We decided to use the sum of the mean meal carbs minus lunch as the lower bound
MAC.maybeBad = 0; %Want to have the patient maybe be a bad patient?
MAC.NonAdherent = 0; %Default value
MAC.conservativeDose = 1; % 0 = The dose the MPC suggests is administered, 1 = The dose is scaled down in the beginning
MAC.sensitivityUpdate = 1; % 0 = The sensitivity is constant, 1 = The sensitivity is updated each morning

if(MAC.maybeBad==1)
    MAC.NonAdherent=(unifrnd(0,1)>0.567);
end


if(mod(1/MAC.timeStep, 1) ~= 0)
    error("The multiplicative inverse of the timestep must be an integer") % because we do some indexing based on this value :P
end

% Meal setup - 4 meals a day specifed with time and amount
MAC.mealMeans = [7.000005 70 ; 12 100 ;  15 40 ; 18 140]; %[hour carbs]
MAC.mealStd = [0.37 10 ; 0.27 12 ; 1.5 20 ; 0.87 40]; %[hour carbs]
MAC.mealAdherence = [0.9; 0.9; 0.6; 0.95];

% Control parameters
MAC.u_min = 0 / MAC.timeStep; % Units / timeStep || minimum insulin injection
MAC.u_max = 300 / MAC.timeStep; % Units / timeStep || maximum insulin injection
MAC.y_min = 3.9; % mmol/L || minimum blood glucose concentration
MAC.y_maxFasting = 7; % mmol/L || maximum blood glucose concentration
MAC.y_maxNonFasting = 10; % mmol/L || maximum blood glucose concentration
MAC.referenceFasting = 5; % mmol/L  || Desired bood glucose concentration during fasting period
MAC.referenceNonFasting = 7;  % mmol/L  || Desired bood glucose concentration during non fasting period
MAC.Q_C = 90 / MAC.timeStep; % the cost for not being at refernce. It is scaled such that the penalty for this and for injecting is in the same range.
MAC.R_C = 0.1 / MAC.timeStep; % the cost for injection || R_C << Q_C due to the high value u takes
MAC.PredDays = 5; % prediction horizon in days
MAC.ContDays = 2; % control horizon in days
MAC.Hp = MAC.PredDays*1/MAC.timeStep; % converting PredDays into days corresponding to the defined timestep
MAC.Hu = MAC.ContDays*1/MAC.timeStep; % converting ContDays into days corresponding to the defined timestep
MAC.rho = 1*10^10; % Penalty on the slack variable


%% None of your business

if(MAC.RealPatient == 1 && MAC.estimateMethod == 0)
    disp("##############-WARNING-################")
    warning("Estimator cannot be turned off when simulating real patient. Full order observer is being used.");
    disp("##############-WARNING-################")

    MAC.estimateMethod = 3;
end
% Miscellaneous
MAC.uss = 0;
MAC.lastDose = 0;
MAC.lastMeal = 0;
MAC.injectionHistory = zeros(1, MAC.simDays / MAC.timeStep);
MAC.stepsPerDay = 1/MAC.timeStep;
MAC.measurementNoiseCovariance = 0.16; % From ISO 15197:2013
MAC.processNoiseCovariance = 0.1;

endFasting = MAC.mealMeans(1, 1); % Time the fasting period ends
timeBeforeFasting = 10; % [Hours] 
startFasting = mod((MAC.mealMeans(end,1) + timeBeforeFasting), 24); % Time the fasting period starts
aux = 1:MAC.stepsPerDay;
% Logicvector, 1 for fasting time, 0 for non-fasting
logicVec = (aux > round((startFasting/24)*MAC.stepsPerDay) & aux < round((endFasting/24)*MAC.stepsPerDay)); 

MAC.referenceVector(logicVec) = MAC.referenceFasting;
MAC.referenceVector(~logicVec) = MAC.referenceNonFasting;
y_ref = repmat(MAC.referenceVector, 1, MAC.simDays);
MAC.referenceVector = repmat(MAC.referenceVector, 1, MAC.PredDays);

MAC.maxGlucoseConstraintVector(logicVec) = MAC.y_maxFasting;
MAC.maxGlucoseConstraintVector(~logicVec) = MAC.y_maxNonFasting;
y_patch = repmat(MAC.maxGlucoseConstraintVector, 1, MAC.simDays);
MAC.maxGlucoseConstraintVector = repmat(MAC.maxGlucoseConstraintVector, 1,MAC.PredDays);



% Patient parameters
MAC.p1 = 0.5;   % tau_1, tau_2
MAC.p3 = 15.8;  % p2
MAC.p4 = 1.8;   % S_I
MAC.p5 = 3.31;  % G_EZI
MAC.p6 = 368;   % E_GP
MAC.p7 = 1.68;  % beta
MAC.pv = 22;    % V_G
MAC.pd = 0.03;  % tau_m
% For non- linear model
if(MAC.modelMismatch)
    MAC.p1_n = MAC.p1*0.7;   % tau_1, tau_2
    MAC.p3_n = MAC.p3*0.7;  % p2
    %MAC.p4_n = normrnd(MAC.p4, 0.0675);   % S_I
    MAC.p4_n = MAC.p4*0.7;
    MAC.p5_n = MAC.p5*0.7;  % G_EZI
    %MAC.p6_n = normrnd(MAC.p6, 3.75);   % E_GP
    MAC.p6_n = MAC.p6*0.7;   % E_GP
    %MAC.p7_n = normrnd(MAC.p7, 0.16);  % beta
    MAC.p7_n = MAC.p7*0.7;  % beta
    MAC.pv_n = MAC.pv*0.7;    % V_G
    MAC.pd_n = MAC.pd*0.7;  % tau_m
else
    MAC.p1_n = MAC.p1;   % tau_1, tau_2
    MAC.p3_n = MAC.p3;  % p2
    MAC.p4_n = MAC.p4;   % S_I
    MAC.p5_n = MAC.p5;  % G_EZI
    MAC.p6_n = MAC.p6;   % E_GP
    MAC.p7_n = MAC.p7;  % beta
    MAC.pv_n = MAC.pv;    % V_G
    MAC.pd_n = MAC.pd;  % tau_m
end

% initialization

if( MAC.meal == 1) % Creates the two additional steadystates and defines the number of states
    MAC.numberOfStates = 6;
    MAC.initialValues = [MAC.initialValues 0 0];
    MAC.D1ss = MAC.initialValues(5);
    MAC.D2ss = MAC.initialValues(6);
    MAC.dss = 0;
else
    MAC.numberOfStates = 4;
end

% defining the steady states based on the initial values defined in setup
MAC.iscss = MAC.initialValues(1);
MAC.ipss = MAC.initialValues(2);
MAC.iss = MAC.initialValues(3);

if(~MAC.trajectory) % if no trajectory then linearize once now and never again
    MAC.Linearization(MAC.steadyStateGlucose);

else
    MAC.Linearization(MAC.initialValues(4));
end

% Just some initialization of variables
MAC.mealMeans(:, 1) = MAC.mealMeans(:, 1)./24;
MAC.mealStd(:, 1) = MAC.mealStd(:, 1)./24;
MAC.actualMealInfo = MAC.mealMeans;
MAC.previousStates = zeros(MAC.numberOfStates, MAC.numOfSavedStates);
MAC.previousStates(:,1) = MAC.initialValues';

tspan = 1:MAC.timeStep:(MAC.simDays + 1 - MAC.timeStep);

MAC.A = [-1/MAC.p1 0 0 0 ;
    1/MAC.p1 -1/MAC.p1 0 0 ;
    0 MAC.p3 -MAC.p3 MAC.p3*MAC.p7 ;
    0 0 (-MAC.p4*MAC.gss) -(MAC.p5+MAC.p4*MAC.iss)];

MAC.B = [1/MAC.p1 0 0 0]';

MAC.C = [0 0 0 1];

if(MAC.meal)
    newCollumns = zeros(MAC.numberOfStates, 2);
    newCollumns(4,2) = 1/(MAC.pv*MAC.pd);
    newCollumns(5,1) = -1/MAC.pd;
    newCollumns(6,1) = 1/MAC.pd;
    newCollumns(6,2) = -1/MAC.pd;

    MAC.A = [MAC.A ; zeros(2,4)];
    MAC.A = [MAC.A newCollumns];

    MAC.B = [MAC.B ; 0 ; 0];
    newCollumns = zeros(6, 1);
    newCollumns(5,1) = MAC.molarWeightConst;
    MAC.B = [MAC.B newCollumns];

    MAC.C = [MAC.C 0 0];
end


%Particle filter init
if(MAC.estimateMethod == 1)
    MAC.NumberOfParticles = 1000;

    MAC.pi_posterior = zeros(1, MAC.NumberOfParticles);
    MAC.pi_prior = zeros(1, MAC.NumberOfParticles);

    MAC.pi_prior(1, :) = 1/MAC.NumberOfParticles;
    MAC.pi_posterior = MAC.pi_prior(1, :) * 1;
    
    MAC.x_posterior = normrnd(0, 3, MAC.numberOfStates, MAC.NumberOfParticles) + MAC.initialValues';  % Particle Estimate
    MAC.x_prior = normrnd(0, 3, MAC.numberOfStates, MAC.NumberOfParticles) + MAC.initialValues';

    MAC.estimate_dx = zeros(MAC.numberOfStates, MAC.NumberOfParticles);
elseif(MAC.estimateMethod == 2)
    % UKF

    % Initialize Sigma Points
    MAC.x_posterior = zeros(MAC.numberOfStates, 1) + MAC.initialValues';   % Initial estimate of state variables
    MAC.x_prior = zeros(MAC.numberOfStates, 1);
    MAC.numberofSigmaPoints = 2*MAC.numberOfStates + 1;
    MAC.sigma_x_prior = zeros(MAC.numberOfStates, MAC.numberofSigmaPoints);
    MAC.sigma_x_posterior = zeros(MAC.numberOfStates, MAC.numberofSigmaPoints);

    MAC.sigma_y_prior = zeros(1, MAC.numberofSigmaPoints);

    MAC.y_hat_prior = zeros(1, 1);

    MAC.alpha = 1e-3;   % Determines the spread of the sigma points                                  % le-4 < alpha < 1

    MAC.P_x_posterior = eye(MAC.numberOfStates)*MAC.alpha;  %alpha because it's a small number :P  Initial state covariance matrix
    MAC.P_x_prior = eye(MAC.numberOfStates);                               % Initial state covariance matrix
    MAC.P_y_prior = eye(1);
    MAC.P_xy_prior = ones(MAC.numberOfStates,1);

    MAC.K_gain = eye(MAC.numberOfStates, 1);

    MAC.beta = 2;                                         % beta = 2, optimal for Gaussian
    MAC.kappa = 0;                                        % kappa = 0 or 3-L
    MAC.lambda = MAC.numberOfStates * (MAC.alpha^2 - 1); % or lambda = alpha^2 * (L + kappa) - L, composite scaling parameter
    MAC.c = MAC.numberOfStates + MAC.lambda;
    MAC.eta = sqrt(MAC.numberOfStates + MAC.lambda); % Yet another scaling parameter
    MAC.Wm = zeros(1,MAC.numberofSigmaPoints); % Weights for sigma points
    MAC.Wc = zeros(1,MAC.numberofSigmaPoints); % Weights for covariance matricies
    MAC.Wm(1, 1) = MAC.lambda/MAC.c;
    MAC.Wc(1, 1) = MAC.lambda/MAC.c + (1-MAC.alpha^2+MAC.beta);

    for i = 2:2*MAC.numberOfStates+1
        MAC.Wm(1, i) = 1/(2*MAC.c);
        MAC.Wc(1, i) = 1/(2*MAC.c);
    end

    MAC.estimate_dx = zeros(MAC.numberOfStates, MAC.numberofSigmaPoints);

    MAC.Rv = eye(MAC.numberOfStates) * MAC.processNoiseCovariance;               % Process noise covariance matrix
    MAC.Rn = eye(1) * MAC.measurementNoiseCovariance;                            % Measurement noise covariance matrix

elseif(MAC.estimateMethod == 3)
    MAC.findObserverGain()
    MAC.x_posterior = zeros(MAC.numberOfStates, 1) + MAC.initialValues';
end
%% Simulation
try
    if(MAC.RealPatient)
        x = MAC.RealPatientSim(tspan, MAC.initialValues);
    else
        switch MAC.controlMethod
            case {0, 2}
                if(MAC.meal)
                    x = MAC.nonlinearModelMeal(tspan, MAC.initialValues);
                else
                    x = MAC.nonlinearModel(tspan, MAC.initialValues);
                end

            case 1
                if(MAC.meal)
                    x = MAC.linearModelMeal(tspan, MAC.initialValues);
                else
                    x = MAC.linearModel(tspan, MAC.initialValues);
                end

            otherwise
                disp("What are you even doing fam?");
        end
    end

    %% Plotting

    if(MAC.controlMethod == 1 && MAC.RealPatient ~= 1)
        figure(1)
        set(gcf, 'units', 'normalized', 'Position', [0 0 1 1])

        subplot(4, 1, 1)
        plot(tspan-1, x(:,1), 'Linewidth', 2, 'DisplayName', 'SC linear')
        title('Insulin concentration')
        ylabel('Insulin concentration [U/L]')
        hold on
        plot(tspan-1, MAC.x_nonLin(:, 1), 'Linewidth', 2, 'DisplayName', 'SC nonlinear')
        if(MAC.estimateMethod ~= 0 && MAC.controlMethod ~= 0)
            plot(tspan-1, MAC.estimated(:, 1), 'Linewidth', 2, 'DisplayName', 'SC estimation')
        end
        plot(tspan-1, x(:,2), 'Linewidth', 2, 'DisplayName', 'P linear')
        plot(tspan-1, MAC.x_nonLin(:, 2), 'Linewidth', 2, 'DisplayName', 'P nonlinear')
        if(MAC.estimateMethod ~= 0 && MAC.controlMethod ~= 0)
            plot(tspan-1, MAC.estimated(:, 2), 'Linewidth', 2, 'DisplayName', 'P estimation')
        end
        legend
        grid on
        hold off

        subplot(4, 1, 2)
        plot(tspan-1, x(:,3), 'Linewidth', 2, 'DisplayName', 'Linear')
        hold on
        plot(tspan -1, MAC.x_nonLin(:, 3), 'Linewidth', 2, 'DisplayName', 'Nonlinear')
        if(MAC.estimateMethod ~= 0 && MAC.controlMethod ~= 0)
            plot(tspan-1, MAC.estimated(:, 3), 'Linewidth', 2, 'DisplayName', 'Estimation')
        end
        legend
        title('Glucose-lowering effect of insulin')
        ylabel('Effective insulin [1/days]')
        grid on
        hold off

        subplot(4, 1, 3)
        y_patch = [MAC.y_min y_patch MAC.y_min MAC.y_min];
        x_patch = [0 tspan-1 tspan(end)-1 0];
        patch(x_patch, y_patch, [0.1 1 0.1], 'FaceAlpha', 0.2, 'DisplayName', 'Target range')
        hold on
        plot(tspan-1, x(:,4), 'Linewidth', 2, 'DisplayName', 'Linear')
        plot(tspan -1, MAC.x_nonLin(:, 4), 'Linewidth', 2, 'DisplayName', 'Nonlinear')
        if(MAC.estimateMethod ~= 0 && MAC.controlMethod ~= 0)
            plot(tspan-1, MAC.estimated(:, 4), 'Linewidth', 2, 'DisplayName', 'Estimation')
        end
        plot(tspan-1,y_ref, '--', 'Linewidth', 2, 'DisplayName', 'Reference', 'Color', 'Black');
        legend
        title('Blood glucose concentration')
        ylabel('Blood glucose [mmol/L]')
        grid on
        hold off

        subplot(4, 1, 4)
        stem(tspan - 1, MAC.injectionHistory * MAC.timeStep, 'Marker', 'none', 'LineWidth', 2, 'DisplayName', 'Injections')
        legend
        title('Insulin injections')
        xlabel('Time [days]')
        ylabel('Injection [U]')
        grid on
    else
        figure(1)
        set(gcf, 'units', 'normalized', 'Position', [0 0 1 1])

        subplot(4, 1, 1)
        title('Insulin concentration')
        ylabel('Insulin concentration [U/L]')
        hold on
        plot(tspan-1, MAC.estimated(:, 1), 'Linewidth', 2, 'DisplayName', 'Subcutaneous')
        plot(tspan-1, MAC.estimated(:, 2), 'Linewidth', 2, 'DisplayName', 'Plasma')
        legend
        grid on
        hold off

        subplot(4, 1, 2)
        hold on
        if(MAC.RealPatient ~= 1)
            plot(tspan-1, x(:,3), 'LineWidth', 2, 'DisplayName', 'Simulated')
        end
        plot(tspan-1, MAC.estimated(:, 3), 'Linewidth', 2, 'DisplayName', 'Estimated')
        legend
        title('Glucose-lowering effect of insulin')
        ylabel('Insulin effect [1/days]')
        grid on
        hold off

        subplot(4, 1, 3)
        hold on
        y_patch = [MAC.y_min y_patch MAC.y_min MAC.y_min];
        x_patch = [0 tspan-1 tspan(end)-1 0];
        patch(x_patch, y_patch, [0.1 1 0.1], 'FaceAlpha', 0.2, 'DisplayName', 'Target range');
        if(MAC.RealPatient == 1)
            plot(tspan-1, x(:,1), 'LineWidth', 2, 'DisplayName', 'Simulated')
        else
            plot(tspan-1, x(:,4), 'LineWidth', 2, 'DisplayName', 'Simulated')
        end
        if(MAC.estimateMethod ~= 0 && MAC.controlMethod ~= 0)
            plot(tspan-1, MAC.estimated(:, 4), 'Linewidth', 2, 'DisplayName', 'Estimated')
        end
        plot(tspan-1,y_ref, '--', 'Linewidth', 2, 'DisplayName', 'Reference', 'Color', 'Black');
        legend
        title('Blood glucose concentration')
        ylabel('Blood glucose [mmol/L]')
        grid on
        hold off

        subplot(4, 1, 4)
        stem(tspan - 1, MAC.injectionHistory * MAC.timeStep, 'Marker', 'none', 'LineWidth', 2, 'DisplayName', 'Injections')
        legend
        title('Insulin injections')
        xlabel('Time [days]')
        ylabel('Injection [U]')
        grid on
    end
    delete(MAC.progressBar)
catch err
    fprintf(2, err.message)
    fprintf(2, '\n')
    for e = 1:length(err.stack)
        fprintf(2, "File: <strong>%s</strong> \n", err.stack(e).name)
        fprintf(2, "Line: %i \n \n", err.stack(e).line)
    end
    delete(MAC.progressBar)
end


















