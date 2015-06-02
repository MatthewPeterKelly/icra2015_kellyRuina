function R = formatResults(C)
%
% This function extracts some of the more important information from C and
% puts it into a nice format that is convienant for plotting
%

m = C.config.dyn.m;
g = C.config.dyn.g;
l = C.config.dyn.l;

R.param.m = m;
R.param.g = g;
R.param.l = l;

R.config.nMeasuredSpeeds = length(C.wMeasured);
R.config.nTargetSpeeds = length(C.wTarget);
R.config.perturbationScale = C.config.param.perturbationScale;

R.cpuTime.mean = mean(mean(C.cpuTime));
R.cpuTime.total = sum(sum(C.cpuTime));

R.robust = C.robust;

pMin = C.config.bnd.p.low;
pMax = C.config.bnd.p.upp;
phiMin = C.config.bnd.phi.low;
phiMax = C.config.bnd.phi.upp;
wMin = min(C.wMeasured);
wMax = max(C.wMeasured);

for i=1:R.config.nTargetSpeeds
    R.control(i).pushOff = (C.p(:,i) - pMin)/(pMax-pMin);
    R.control(i).stepAngle = (C.phi(:,i) - phiMin)/(phiMax-phiMin);
    R.speed(i).min = (C.wFinalMin (:,i) - wMin)/(wMax-wMin);
    R.speed(i).nom = (C.wFinal(:,i) - wMin)/(wMax-wMin);
    R.speed(i).max = (C.wFinalMax(:,i) - wMin)/(wMax-wMin);
    R.speed(i).target = (C.wTarget(i) - wMin)/(wMax-wMin);
    R.measuredSpeed = (C.wMeasured - wMin)/(wMax-wMin);
    
    R.bnd.nextMidStanceSpeed(i).axis = ([...
        min(min(C.wFinalMin(:,i))), max(max(C.wFinalMax(:,i)))] - wMin)/(wMax-wMin);
end

end






