function [wFinal, v, fail] = oneStep(m,g,l,phi,p,wInit)
%[wFinal, v, fail] = oneStep(m,g,l,phi,p,wInit)
%
% This function performs one step for the simple walker
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
%   v = normalized constraint violation
%   fail = boolean row vector, indicating failure mode:
%       fail(1) = Too fast at swing-down
%       fail(2) = Heel-strike collision negative
%       fail(3) = Too fast after collision
%       fail(4) = Too slow after collision: fall backwards
%

%Precompute trig functions
c = cos(phi); s = sin(phi);
cs = c*s;
cc_ss = (c*c-s*s);
wFly = sqrt((g/l)*c);
wFall = sqrt(2*g*(1-cos(phi))/l);
fail = false(1,4);

%Swing-Down
wMinus = sqrt(wInit*wInit + 2*(g/l)*(1-c));
if wMinus > wFly %check leg tension
    fail(1) = true;
    v1 = (wMinus - wFly)/wFly;
    wMinus = wFly;
else
    v1 = 0;
end

%Push-off then heel-strike
wPlus = wMinus*cc_ss + (2*p/(m*l))*cs;
C = 2*m*l*wMinus*cs - p*cc_ss;   %Heelstrike impulse
if C<0 %Check heel-strike
    fail(2) = true;
    v2 = -C/(m*sqrt(g*l));
else
   v2 = 0; 
end
if wPlus > wFly  %check leg tension
    fail(3) = true;
    v3 = (wPlus - wFly)/wFly;
    wPlus = wFly;
else
    v3 = 0;
end

%Swing-Up
if wPlus < wFall  %check leg tension
    fail(4) = true;
    v4 = (wFall-wPlus)/wFall;
    wFinal = 0;
else
    v4 = 0;
    wFinal = sqrt(wPlus*wPlus - 2*(g/l)*(1-c));
end

%Sum up the constraint violation:
v = v1.^2 + v2.^2 + v3.^2 + v4.^2;

end