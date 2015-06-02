%MAIN_Design_Controller.m
%

clc; clear;

%%% Derive the objective and constraint equations:
% Derive_Equations_Ideal.m;
% Derive_Equations_Robust.m;

% %%% Build mex functions:
% coder -build oneStep.prj
% coder -build manyStep.prj
% coder -build simulateRobust.prj

%%% Set up parameters:
m = 9.91; config.dyn.m = m;
g = 9.81; config.dyn.g = g;
l = 0.96; config.dyn.l = l;

%%% Set up the bounds on state and actuation for the optimization
phi.low = 0*pi/180;
phi.upp = 30*pi/180; config.bnd.phi = phi;
p.low = 0;
p.upp = 0.4*m*sqrt(g*l); config.bnd.p = p;
w.low = 0*sqrt(g/l);  % USED FOR FLIGHT CONSTRAINT
w.upp = 1*sqrt(g/l); config.bnd.w = w;

%%% Batch processing variables:
nMeasured = 50;
nTarget = 3;
wBnd = [0,0.8];
wMeasured = sqrt(g/l)*linspace(wBnd(1),wBnd(2), nMeasured);
wTarget = sqrt(g/l)*linspace(wBnd(1),wBnd(2), nTarget+2); wTarget([1,end]) = [];

%%% Optimization parameters:
config.param.regularization = 1e-6;  %Only used in initialization
beta = 0.05;  config.param.perturbationScale = beta;

%%% Robustness Testing parameters:
config.robust.nSim = 1000;
config.robust.nStep = 1000;

%%% Perturbation Parameters for robust optimization:
config.param.perturbationScale = beta;
config.param.perturbationNormalization.l = l;  %Leg Length
config.param.perturbationNormalization.p = p.upp-p.low;  %push-off
config.param.perturbationNormalization.phi = phi.upp-phi.low; %Step Angle
config.param.perturbationNormalization.w = diff(wBnd); %Mid-stance speed
config.param.perturbation.l = beta*l;  %Leg Length
config.param.perturbation.p = beta*(p.upp-p.low);  %push-off
config.param.perturbation.phi = beta*(phi.upp-phi.low); %Step Angle
config.param.perturbation.w = beta*diff(wBnd); %Mid-stance speed

%%% Allocate Memory:
C.p = zeros(nMeasured,nTarget);
C.phi = zeros(nMeasured,nTarget);
C.exitFlag = zeros(nMeasured,nTarget);
C.fVal = zeros(nMeasured,nTarget);
C.cpuTime = zeros(nMeasured,nTarget);
C.wFinal = zeros(nMeasured,nTarget);
C.gamma = zeros(nMeasured,nTarget);
C.activeConstraint = false(nMeasured,nTarget,7); %[phi,p,flyEarly, flyPush, flyLate, fallBack]
C.wFinalRobust = zeros(nMeasured,nTarget,16);
C.wFinalMax = zeros(nMeasured,nTarget);
C.wFinalMin = zeros(nMeasured,nTarget);
C.delMax = zeros(nMeasured,nTarget,4);
C.delMin = zeros(nMeasured,nTarget,4);

%%% Loop over desired inputs:
nTotal = nMeasured*nTarget;
idx = 0;
MaxAttempts = 20; %Maximum number of attempts to restart optimization
for i=1:nMeasured;
    for j=1:nTarget;
        
        %%% Set up individual trial:
        in.w0 = wMeasured(i);
        in.wTarget = wTarget(j);
        
        %%% Design the controller:   (THIS IS THE KEY PART)
        tic
        for k=1:MaxAttempts  
            % Run an optimization got get a good initialization
            soln_ideal = getIdealControl(in, config, k);
            if soln_ideal.exitFlag == 1 || k==MaxAttempts 
                %Then successful initialization - try robust version
                soln_robust = getRobustControl(in, config, soln_ideal);
                if soln_robust.exitFlag == 1  %Then success! 
                    break;  %Expect to reach here on k==1 for most cases
                end
            end
        end
        
        %%% Test the maximum perturbation:   (game theory!)
        in.pushOff = soln_robust.p;
        in.stepAngle = soln_robust.phi;
        maxDist = getMaxDisturbance(in, config);
        cpuOptimizeTime = toc;
        
        %%% Pack up the results:
        C.phi(i,j) = soln_robust.phi;
        C.p(i,j) = soln_robust.p;
        C.exitFlag(i,j) = soln_robust.exitFlag;
        C.fVal(i,j) = soln_robust.fVal;
        C.cpuTime(i,j) = cpuOptimizeTime;
        C.wFinal(i,j) = soln_robust.wFinal;
        C.wFinalRobust(i,j,:) = reshape(soln_robust.wFinalRobust,1,1,16);
        C.failRobustTest(i,j) = maxDist.fail;
        C.wFinalMax(i,j) = maxDist.wFinalMax;
        C.wFinalMin(i,j) = maxDist.wFinalMin;
        C.delMax(i,j,:) = reshape(maxDist.delMax,1,1,4);
        C.delMin(i,j,:) = reshape(maxDist.delMin,1,1,4);
    end
    fprintf('Progress: %3d%%\n',round(i*100/nMeasured));
end
C.wMeasured = wMeasured';
C.wTarget = wTarget;
C.config = config;

%%% Run a massive simulation:
C = testSimulateRobust(C);

%%% Compute stability regions:
C = getStabilityRegion(C);

%%% Plot the solution:
figure(111); plotController(C);

%%% Format Results and then save:
R = formatResults(C);
saveResults(C,R);







