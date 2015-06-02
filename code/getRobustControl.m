function soln = getRobustControl(input, config, init)
% soln = getRobustControl(in, config, soln_ideal)
%
% This function solves the optimal control problem for the simplest walker
% such that it is robust to perturbations.
%

%%% Problem Setup
w0 = input.w0;
wTarget = input.wTarget;

%%% Physical paramters for the robot:
m = config.dyn.m;
g = config.dyn.g;
l = config.dyn.l;

%%% Bounds on state and actuation
phiLow = config.bnd.phi.low;
phiUpp = config.bnd.phi.upp;
pLow = config.bnd.p.low;
pUpp = config.bnd.p.upp;
wLow = config.bnd.w.low;
wUpp = config.bnd.w.upp;

%%% Perturbation magnitude:
del = [...
    config.param.perturbation.l,...
    config.param.perturbation.p,...
    config.param.perturbation.phi,...
    config.param.perturbation.w];

%%% Set up decision variables
nDecVar = 2 + 3*2^4;
decVarGuess = zeros(1,nDecVar);
decVarLow = zeros(1,nDecVar);
decVarUpp = zeros(1,nDecVar);
decVarGuess(1:2) = [init.phi, init.p];
decVarLow(1:2) = [phiLow, pLow];
decVarUpp(1:2) = [phiUpp, pUpp];
wGuess = [init.wMinus, init.wPlus, init.wFinal];
for i=1:17
    idx = 2 + (i-1)*3 + [1,2,3];
    decVarGuess(idx) = wGuess;
    decVarLow(idx) = wLow*ones(1,3);
    decVarUpp(idx) = wUpp*ones(1,3);
end


%%% Set up the fmincon problem struct
problem.objective = @(decVar)ObjFunRobust(decVar, wTarget);
problem.nonlcon = @(decVar)CstFunRobust(decVar,del,w0,wTarget,m,g,l);
problem.options = optimset(...
    'Algorithm','interior-point',...
    'Display', 'off',...
    'MaxFunEvals',1e4,...
    'TolFun',1e-6,...
    'TolX',1e-12,...
    'TolCon',1e-6,...
    'GradObj','off',...
    'GradConstr','on');
problem.lb = decVarLow;
problem.ub = decVarUpp;
problem.x0 = decVarGuess;
problem.Aineq = []; problem.Aeq = [];
problem.bineq = []; problem.beq = [];
problem.solver = 'fmincon';


%%% Solve the optimization problem:
[decVarSoln, fVal, exitFlag] = fmincon(problem);

%%% Extract wFinal for all perturbed cases:
idxSpeedNextRobust = 2+3+(0:3:(3*16-1));
wFinalRobust = decVarSoln(idxSpeedNextRobust);

%%% Return the solution:
soln.phi = decVarSoln(1);
soln.p = decVarSoln(2);
soln.fVal = fVal;
soln.exitFlag = exitFlag;
soln.w0 = w0;
soln.wFinal = decVarSoln(end);
soln.wTarget = wTarget;
soln.wFinalRobust = wFinalRobust;

end