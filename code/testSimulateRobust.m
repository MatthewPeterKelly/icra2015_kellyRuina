function C = testSimulateRobust(C)
% C = testSimulateRobust(C)
% 
% This function runs a massive simulation to test that the controller using
% linear interpolation still works (it does).
%

input.nSim = C.config.robust.nSim;
input.nStep = C.config.robust.nStep;
input.nTarget = length(C.wTarget);

del = C.config.param.perturbation;
dyn = C.config.dyn;

Ctrl.w = C.wMeasured;
Ctrl.p = C.p;
Ctrl.phi = C.phi;

%%% THIS IS THE KEY LINE:
C.robust.nFail = simulateRobust_mex(input, Ctrl, del, dyn);
C.robust.nSim = input.nSim;

end