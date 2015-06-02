# README.md - Matlab Code for ICRA 2015 Paper
### Non-linear robust control for 2D walking

## Scripts for writing functions:
- **'Derive_Equations_Ideal.m'** is used to generate the objective and constraint functions for the ideal fmincon optimization, the results of which are used to initialize the robust fmincon optimization.  
- **'Derive_Equations_Robust.m'** is used to generate the objective and constraint functions for the robust fmincon optimization. 

## MEX Functions:
There are a few mex functions that are used in this project. Each mex function has a matching m-file that does the same exact thing. It also has a project that can be used to rebuild the mex function for a different architecture. The naming conventions that I use are:
- 'functionName.m'  Matlab version of function
- 'functionName.prj'  Project file to build mex function
- 'functionName_mex.mex*'  Mex Function to be called from matlab
