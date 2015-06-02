% Derive_Equation_Robust.m
%
% This script derives the objective and constraint functions that are used
% by FMINCON to solve the optimal controls for the simplest walker that are
% robust to perturbations.
%
% This is accomplished by computing the one-step simulation of the walker
% under ideal conditions, and also with all 16=2^4 combinations of maximal
% perturbations. The contraction ratio for the nominal case is constrained
% to be less than one, corresponding to a Lyapunov function for this case.
% The objective function is to minimize the squared error in mid-stance
% speed at the next mid-stance.
%

clc; clear;

syms m g l 'real' %Physical Parameters
syms w0 wTarget 'real' %Initial and Target Speed
del = sym('del',[1,4]); assume(del,'real');
%       del(1) = leg length perturbation magnitude
%       del(2) = push-off perturbation magnitude
%       del(3) = step angle perturbation magnitude
%       del(4) = measured speed perturbation magnitude

% [phi, p, (perturbted intermediate states), (nominal states)]; 
nDecVar = 2 + 3*(2^4 + 1); % Each simulation gets three decision variables.
decVar = sym('decVar',[1,nDecVar]); assume(decVar,'real');
%   (1) = phi;
%   (2) = p;
%
%   (4) = wMinus(1);  
%   (5) = wPlus(1);
%   (6) = wFinal(1);
%   ...
%   (end-2) = wMinus(17);
%   (end-1) = wPlus(17);
%   (end) = wFinal(17);
%

phi = decVar(1);  %Step angle
p = decVar(2);   %Push-Off

C = sym('C',[4*17+1,1]); %C <= 0 Enforced to be true
Ceq = sym('Ceq',[3*17,1]); %Ceq == 0 Enforced to be true
F = sym(0);   %Start with cost for perturbation magnitude
fprintf('Computing Objective and Constraint Functions: \n');
for i=1:17  %Loop over each possible perturbation
    decIdx = 3*(i-1) + 2;
    wMinus = decVar(decIdx+1);
    wPlus = decVar(decIdx+2);
    wFinal = decVar(decIdx+3);
    
    %%% Apply perturbations in model, actuation, and sensing:
    if i <= 16
    direction = 1-2*de2bi(i-1,4);
    del_l = l + direction(1)*del(1);
    del_p = p + direction(2)*del(2);
    del_phi = phi + direction(3)*del(3);
    del_w0 = w0 + direction(4)*del(4);
    else %i==17   % Nominal case (no perturbation)
        del_l = l;
        del_p = p;
        del_phi = phi;
        del_w0 = w0;
    end
    
    %%% Commonly used expressions:
    s = sin(del_phi);
    c = cos(del_phi);
    wFly = sqrt(g*c/del_l);
    wCrit2 = 2*g*(1-c)/del_l;   %Positive by definition
    ccss = c*c-s*s;
    
    %%% Inequality Constraints:
    cIdx = 4*(i-1);  
    C(cIdx+1) = wMinus - wFly;
    C(cIdx+2) = 0 - ( (2*m*del_l)*wMinus*c*s - del_p*(ccss) );
    C(cIdx+3) = wPlus - wFly;
    C(cIdx+4) = sqrt(wCrit2) - wPlus; %Real by definition
    
    %Stability Constraint
    errFinal = (wFinal - wTarget)^2;
    if i==17  % Nominal Case
        errInit = (w0 - wTarget)^2;
        C(cIdx+5) = errFinal - errInit;
    end
    
    %%% Equality Constraints: (Dynamics)
    % Some equations are squared to prevent imaginary values.
    ceqIdx = 3*(i-1);
%     Ceq(ceqIdx + 1) = wMinus - sqrt(del_w0^2+wCrit2);
    Ceq(ceqIdx + 1) = wMinus^2 - (del_w0^2+wCrit2);  %Squared Constraint
    Ceq(ceqIdx + 2) = wPlus - ( wMinus*(ccss) + (2*del_p/(m*del_l))*c*s );
%     Ceq(ceqIdx + 3) = wFinal - sqrt(wPlus*wPlus - wCrit2);   
    Ceq(ceqIdx + 3) = wFinal^2 - (wPlus*wPlus - wCrit2);  %Squared Constraint
    
    %%% Objective Function:
    if i<=16  %robust case
        F = F + errFinal;
    end
    
    fprintf('.');
end %loop over perturbations
fprintf('  done!\n')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the gradients:
fprintf('Computing gradients')

%%% Gradient of Equality Constraints:
GC = jacobian(C,decVar)'; fprintf('.');

%%% Gradient of InEquality Constraint:
GCeq = jacobian(Ceq,decVar)'; fprintf('.');

%%% Gradient of Objective Function:
GF = jacobian(F,decVar)'; fprintf('.  done!\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Use matlab to automatically write function files:

fprintf('Writing Objective Function... ')
matlabFunction(F,GF,'file','ObjFunRobust.m',...
    'vars',{decVar,wTarget}); 
fprintf('   done!\n');

fprintf('Writing Constraint Function... ')
matlabFunction(C,Ceq,GC,GCeq,'file','CstFunRobust.m',...
    'vars',{decVar,del,w0,wTarget,m,g,l}); 
fprintf('   done!\n');












