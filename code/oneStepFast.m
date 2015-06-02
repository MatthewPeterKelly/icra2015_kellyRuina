function [wFinal, fail] = oneStepFast(m,g,l,phi,p,w0)
%[wFinal, fail] = oneStep(m,g,l,phi,p,wInit)
%
% This function performs one step for the simple walker, but skips
% calculating some of the details in oneStep.m. OneStepFast.m is designed
% to be called inside a mex function for doing very long simulations for
% robustness testing.
%
% INPUTS:
%   m = mass
%   g = gravity
%   l = leg length
%   phi = step angle
%   p = push-off impulse
%   wInit = initial mid-stance angular rate
%
% OUTPUTS
%   wFinal = next mid-stance angular rate
%   fail = was a constraint violated?
%

fail = false;

%Precompute trig functions
c = cos(phi); s = sin(phi);
cs = c*s;
cc_ss = (c*c-s*s);
wFly = sqrt((g/l)*c);
wFall = sqrt(2*g*(1-cos(phi))/l);

%Swing-Down
wMinus = sqrt(w0*w0 + 2*(g/l)*(1-c));
if wMinus > wFly %check leg tension
    fail = true;
    wMinus = wFly;
end

%Push-off then heel-strike
wPlus = wMinus*cc_ss + (2*p/(m*l))*cs;
C = 2*m*l*wMinus*cs - p*cc_ss;   %Heelstrike impulse
if C<0 %Check heel-strike
    fail = true;
end

if wPlus > wFly  %check leg tension
    fail = true;
    wPlus = wFly;
end

%Swing-Up
if wPlus < wFall  %check leg tension
    fail = true;
    wFinal = 0;
else
    wFinal = sqrt(wPlus*wPlus - 2*(g/l)*(1-c));
end

end