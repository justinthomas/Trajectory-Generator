#include "mex.h"
#include "matrix.h"
#include "string.h"

/*
 * trajectory_generator.c
 *
 * Description goes here
 *
 * This is a MEX-file for MATLAB
 */

void trajgen(double *traj, double *args)
{
  
}

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  double *x,*y;
  double *waypoints, *bounds, *options;
  double *traj;
  // size_t mrows,ncols;
  const char *fields[1]; /* Pointers to the field names */
  
  /* Check for proper number of arguments. */
  if(nrhs==0) {
    mexErrMsgIdAndTxt("MATLAB:trajgen:invalidNumInputs", "At least one input is required.");
  } else if(nrhs>3){
    mexErrMsgIdAndTxt("MATLAB:trajgen:invalidNumInputs", "A maximum of three inputs are allowed.");
  } else if(nlhs>1) {
    mexErrMsgIdAndTxt("MATLAB:trajgen:maxlhs", "Too many output arguments.");
  }
  
  /* The input datatypes must be structs */
  if(!mxIsStruct(prhs[0])) {
    mexErrMsgIdAndTxt("MATLAB:trajgen:inputTypes", "The first input must be a struct.");
  }else if(nrhs > 1 && !mxIsStruct(prhs[1]) && !mxIsEmpty(prhs[1])){
    mexErrMsgIdAndTxt("MATLAB:trajgen:inputTypes", "The second input must be a struct.");
  }else if(nrhs > 2 && !mxIsStruct(prhs[2]) && !mxIsEmpty(prhs[2])){
    mexErrMsgIdAndTxt("MATLAB:trajgen:inputTypes", "The third input must be a struct.");
  }
  
  /* Create struct for the return argument. */
  /* Dummy struct for now */
  fields[0] = "t";
  fields[1] = "x";
  fields[2] = "y";
  fields[3] = "z";
  fields[4] = "psi";
  
  plhs[0] = mxCreateStructMatrix(1, 1, 5, fields);
  // mexPrintf(fields[0]); 
  
  // Create structs for ipnuts arguments
  // Assign pointers to each input and output
  waypoints = mxGetPr(prhs[0]);
  bounds = mxGetPr(prhs[1]);
  options = mxGetPr(prhs[2]);
  traj = mxGetPr(plhs[0]);
  
  // Call the trajgen subroutine
  trajgen(traj, waypoints);
  
  return;
}
