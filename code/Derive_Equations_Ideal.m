%Derive_Equation_Ideal.m
%
% This script writes the objective and constraint functions that are used
% by FMINCON to solve the optimal controls for taking one step with the
% simplest walker, under idea conditions. It also computes the analytic
% jacobian and hessian matricies for the objective and constraint
% functions.
%
% The ideal version of the problem is used to initialize the "robust
% version" of the problem (where there are disturbances inside of the
% optimization).
%

clc; clear;

syms m g l 'real'  %Physical Parameters
syms w0 wTarget  'real' %Target Speed
alpha = sym('alpha',[1,2]);  assume(alpha, 'real'); %Cost Function regularization
syms pNom phiNom 'real';   %Nominal control values for regularization
decVar = sym('decVar',[1,5]); assume(decVar, 'real');  %Decision variables
%   (1) = phi;
%   (2) = p;
%   (3) = wMinus;
%   (4) = wPlus;
%   (5) = wFinal;

phi = decVar(1);
p = decVar(2);
wMinus = decVar(3);
wPlus = decVar(4);
wFinal = decVar(5);

%%% Commonly used expressions:
s = sin(phi);
c = cos(phi);
wFly = sqrt(g*c/l);
wCrit2 = 2*g*(1-c)/l;
ccss = c*c-s*s;

%%% Inequality Constraints:
C = [...; %C <= 0 Enforced to be true
    wMinus - wFly;
    0 - ( (2*m*l)*wMinus*c*s - p*(ccss) );
    wPlus - wFly;
    sqrt(wCrit2) - wPlus];

%%% Equality Constraints: (Dynamics)
Ceq = [...
    wMinus - sqrt(w0*w0+wCrit2);
    wPlus - ( wMinus*(ccss) + (2*p/(m*l))*c*s );
    wFinal - sqrt(wPlus*wPlus - wCrit2)];

%%% Gradient of Equality Constraints:
GC = jacobian(C,decVar)';

%%% Gradient of InEquality Constraint:
GCeq = jacobian(Ceq,decVar)';

%%% Objective Function:
F = (wFinal-wTarget)^2 + (alpha(1)*(phi-phiNom))^2 + (alpha(2)*(p-pNom))^2;

%%% Gradient of Objective Function:
GF = jacobian(F,decVar)';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the Hessian...

%Lagrange Multipliers
lambdaC = sym('lambdaC',[length(C),1]);
lambdaCeq = sym('lambdaCeq',[length(Ceq),1]);

H = hessian(F,decVar);

for i=1:length(C)
    H = H + lambdaC(i)*hessian(C(i),decVar);
end

for i=1:length(Ceq)
    H = H + lambdaCeq(i)*hessian(Ceq(i),decVar);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Use matlab to derive the equations:

matlabFunction(C,Ceq,GC,GCeq,'file','CstFunIdeal.m',...
    'vars',{decVar,w0,m,g,l});

matlabFunction(F,GF,'file','ObjFunIdeal.m',...
    'vars',{decVar,wTarget,alpha,phiNom,pNom});

matlabFunction(H,'file','HessIdeal.m',...
    'vars',{decVar,lambdaC, lambdaCeq, w0,m,g,l,alpha});













