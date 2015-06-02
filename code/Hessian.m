function H = Hessian(decVar,lambda, w0,m,g,l,alpha)
% H = Hessian(decVar,lambda, w0,m,g,l,alpha)
%
% This function computes the hessian for the FMINCON optimization of the
% simplest walker under ideal conditions.
%N
% See Derive_Equations_Ideal.m for a description of input parameters to
% this function.
%

lambdaC = lambda.ineqnonlin;
lambdaCeq = lambda.eqnonlin;
H = HessIdeal(decVar',lambdaC,lambdaCeq,w0,m,g,l,alpha);

end