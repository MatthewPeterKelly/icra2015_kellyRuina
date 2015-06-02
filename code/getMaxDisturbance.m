function out = getMaxDisturbance(in, config)
%
% Computes the worst possible disturbance for maximizing speed errors at
% the next step, in both the positive and negative directions.
%

%%% User parameters:
nSamplesInitial = 1000;

%%% Unpack the nominal controls
p = in.pushOff;
phi = in.stepAngle;
w0 = in.w0;

%%% Physical paramters for the robot:
m = config.dyn.m;
g = config.dyn.g;
l = config.dyn.l;

%%% Parameters for perturbation:
alpha = config.param.perturbationScale;
maxDel = config.param.perturbationNormalization;
delBnd = [maxDel.l, maxDel.p, maxDel.phi, maxDel.w];


%%% Compute the speed for the corner cases - sanity check. Don't use these
%%% for initialization, since they are all gaurenteed local minima
xCorner = alpha*(1-2*de2bi(0:15,4));
del.l = l + xCorner(:,1)*maxDel.l;%Leg length
del.p = p + xCorner(:,2)*maxDel.p;%pushOff
del.phi = phi + xCorner(:,3)*maxDel.phi;%Step angle
del.w0 = w0 + xCorner(:,4)*maxDel.w;%Sensor Error
wCorner = manyStep_mex(m, g, del.l, del.phi, del.p, del.w0);
[fCornerMax, idxMax] = max(wCorner); delCornerMax = xCorner(idxMax,:);
[fCornerMin, idxMin] = min(wCorner); delCornerMin = xCorner(idxMin,:);


%%% Sample dynamics for initial distribution:
samples = alpha*(1-2*rand(nSamplesInitial,4));
del.l = l + samples(:,1)*maxDel.l;%Leg length
del.p = p + samples(:,2)*maxDel.p;%pushOff
del.phi = phi + samples(:,3)*maxDel.phi;%Step angle
del.w0 = w0 + samples(:,4)*maxDel.w;%Sensor Error
[wFinal, v, failCount] = manyStep_mex(m, g, del.l, del.phi, del.p, del.w0);

%%% Find the minimum and maximum error:
[~, idxMax] = max(wFinal);
[~, idxMin] = min(wFinal);


%%% Refine the solution using fmincon:
problem.lb = -alpha*delBnd;
problem.ub = alpha*delBnd;
problem.nonlcon = [];
problem.Aineq = []; problem.Aeq = [];
problem.bineq = []; problem.beq = [];
problem.solver = 'fmincon';
problem.options = optimset(...
    'Algorithm','interior-point',...
    'Display', 'off');

%%% First solve for minimum final speed:
problem.x0 = delBnd.*samples(idxMax,:);
problem.objective = @(x)objFunMin(x,w0,p,phi,m,g,l);
[xMin, fMin] = fmincon(problem);
if fMin < fCornerMin
    out.delMin = xMin./(alpha*delBnd);
    out.wFinalMin = fMin;
else
    out.delMin = delCornerMin/alpha;
    out.wFinalMin = fCornerMin;
end

%%% Then solve for maximum final speed:
problem.x0 = delBnd.*samples(idxMin,:);
problem.objective = @(x)objFunMax(x,w0,p,phi,m,g,l);
[xMax, fMax] = fmincon(problem);
if -fMax > fCornerMax
    out.delMax = xMax./(alpha*delBnd);
    out.wFinalMax = -fMax;
else
    out.delMax = delCornerMax/alpha;
    out.wFinalMax = fCornerMax;
end

%%% Check for failures in random samples
cstFail = sum(failCount) > 0;
if cstFail
    [~, idxFail] = max(v);
else
    idxFail = [];
end
out.fail = cstFail;
out.idxFail = idxFail;

end


function cost = objFunMin(x,w0,p,phi,m,g,l)

cost = oneStep_mex(m, g, l+x(1), phi+x(3), p+x(2), w0+x(4));

end


function cost = objFunMax(x,w0,p,phi,m,g,l)

cost = -oneStep_mex(m, g, l+x(1), phi+x(3), p+x(2), w0+x(4));

end

