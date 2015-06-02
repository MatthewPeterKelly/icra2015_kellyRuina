function soln = getIdealControl(in, config, attemptId)
% soln = getIdealControl(in, config, k)
%
% This function computes the ideal controls for the simplest walker under
% idea conditions
%

%%% Problem Setup
w0 = in.w0;
wTarget = in.wTarget;

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

%%% Compute regularization target:
% The problem is not well posed, since there are multiple ways to achieve a
% deadbeat controller. The solution is to add some regularization by giving
% the optimization a nominal actuation to keep near. One the first attempt
% this is always the middle of the actuation, but if that fails, then a
% random value is selected to try to break free of the local minimum.
%
if attemptId == 1  %Then first try
    phiNom = 0.5*(phiLow+phiUpp);
    pNom = 0.5*(pLow+pUpp);
else %Then failed on last attempt - try a new regularization:
    r1 = rand(1); r2 = rand(1); 
    phiNom = r1*phiLow + (1-r1)*phiUpp;
    pNom = r2*pLow + (1-r2)*pUpp;
end
alpha = sqrt(config.param.regularization)*[phiUpp-phiLow, pUpp-pLow];

%%% Set up the fmincon problem struct
problem.objective = @(decVar)ObjFunIdeal(decVar, wTarget,alpha,phiNom,pNom);
problem.nonlcon = @(decVar)CstFunIdeal(decVar,w0,m,g,l);
problem.options = optimset(...
    'Algorithm','interior-point',...
    'Display', 'off',...
    'GradObj','on',...
    'GradConstr','on',...
    'Hessian','user-supplied',...
    'HessFcn',@(decVar,lambda)Hessian(decVar,lambda, w0,m,g,l,alpha));
problem.lb = [phiLow, pLow, wLow*ones(1,3)];
problem.ub = [phiUpp, pUpp, wUpp*ones(1,3)];
problem.x0 = 0.5*(problem.lb + problem.ub);
problem.Aineq = []; problem.Aeq = [];
problem.bineq = []; problem.beq = [];
problem.solver = 'fmincon';

%%% Solve the optimization problem:
[decVarSoln, ~, exitFlag] = fmincon(problem);

%%% Return the results:
%   decVar(1) = phi;
%   decVar(2) = p;
%   decVar(3) = wMinus;
%   decVar(4) = wPlus;
%   decVar(5) = wFinal;
soln.phi = decVarSoln(1);
soln.p = decVarSoln(2);
soln.wMinus = decVarSoln(3);
soln.wPlus = decVarSoln(4);
soln.wFinal = decVarSoln(5);
soln.exitFlag = exitFlag;

end