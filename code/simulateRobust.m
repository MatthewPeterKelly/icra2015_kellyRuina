function nFail = simulateRobust(input, C, del, dyn)

nSim = input.nSim;
nStep = input.nStep;
nTarget = input.nTarget;

m = dyn.m;
g = dyn.g;
l = dyn.l;

nFail = 0;
nMeasured = length(C.w);
ctrl.w = C.w;
for i=1:nSim
    
    idxTarget = randi(nTarget);  %Which target speed to use?
    ctrl.p = C.p(:,idxTarget );
    ctrl.phi = C.phi(:,idxTarget );
    
    w0 = ctrl.w(randi(nMeasured));
    
    l_noise = l + (1-2*rand(1))*del.l;   %Model Error
    
    for j=1:nStep
        w0_noise = w0 + (1-2*rand(1))*del.w;   %Sensor Noise
        [p,phi] = InterpController(w0_noise,ctrl);   % Get Control
        p_noise = p + (1-2*rand(1))*del.p;    %Actuator Error
        phi_noise = phi + (1-2*rand(1))*del.phi;   %Actuator Error
        [wFinal, stepFailed] = oneStepFast(m,g,l_noise,phi_noise,p_noise,w0);
        if stepFailed
            nFail = nFail + 1;
            break;
        else
            w0 = wFinal;
        end
    end
end

end

function [p,phi] = InterpController(w,C)
%[p,phi] = InterpController(w,C)
%
% This function computes the robust contols for measured speed w, using
% controller C. If w is out of bounds, then the closest point in C is used
% instead of interpolating. Since each point in the controller must be in
% the valid range of the corresponding actuator, and linear interpolation
% is used, the controller cannot return a value for p or phi that is out of
% bounds
%

if w < C.w(1)   %Out of bounds low!
    p = C.p(1);
    phi = C.phi(1);
elseif w > C.w(end)  %Out of bounds high!
    p = C.p(end);
    phi = C.phi(end);
else  %speed is in bounds - woo!
    p = interp1(C.w, C.p, w, 'linear');
    phi = interp1(C.w, C.phi, w, 'linear');
end

end