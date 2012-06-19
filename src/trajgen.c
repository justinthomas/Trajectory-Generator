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

void trajgen(mxArray *traj, mxArray *waypoints, mxArray *bounds, mxArray *options)
{
  
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  mxArray *waypoints, *bounds, *options;
  mxArray *traj;
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
  
  // Assign pointers to each input
  if(nrhs>=1) {
    waypoints = mxGetData(prhs[0]);
  }
  if(nrhs>=2) {
    bounds = mxGetData(prhs[1]);
  }
  if(nrhs>=3) {
    options = mxGetData(prhs[2]);
  }
  
  // Assign a pointer to the output
  traj = mxGetData(plhs[0]);

  // Call the trajgen subroutine
  trajgen(traj, waypoints, bounds, options);
  
  return;
}
