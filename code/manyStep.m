function [wFinal, v, failCount] = manyStep(m,g,l,phi,p,wInit)
%[wf v] = MANYSTEP(m,g,l,phi,p,w0)
%
% This function performs one step for the simple walker
%
% INPUTS:
%   m = mass 
%   g = gravity
%   l = leg length (Nx1)
%   phi = step angle (Nx1)
%   p = push-off impulse (Nx1)
%   wInit = initial mid-stance angular rate (Nx1)
%
% OUTPUTS
%   wFinal = next mid-stance angular rate (Nx1)
%   v = normalized constraint violation (Nx1)
%   fail = failure mode count (1x4):
%       fail(1) = Too fast at swing-down
%       fail(2) = Heel-strike collision negative
%       fail(3) = Too fast after collision
%       fail(4) = Too slow after collision: fall backwards
%

N = length(phi);
failCount = zeros(1,4);
wFinal = zeros(N,1);
v = zeros(N,1);
for i=1:length(phi) %Loop through each set of conditions:
    [wFinal(i), v(i), fail] = oneStep(m,g,l(i),phi(i),p(i),wInit(i));
    failCount = failCount + fail;
end

end