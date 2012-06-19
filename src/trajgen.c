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

/* void trajgen(double y[], double x[])
{
  y[0] = 2.0*x[0];
  }*/

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  /* double *x,*y;
     size_t mrows,ncols;*/
  const char *fields[1]; /* Pointers to the field names */
  
  /* Check for proper number of arguments. */
  if(nrhs==0) {
    mexErrMsgIdAndTxt("MATLAB:trajgen:invalidNumInputs", "At least one input is required.");
  } else if(nrhs>3){
    mexErrMsgIdAndTxt("MATLAB:trajgen:invalidNumInputs", "A maximum of three inputs are allowed.");
  } else if(nlhs>1) {
    mexErrMsgIdAndTxt("MATLAB:trajgen:maxlhs", "Too many output arguments.");
  }
  
  /* The input datatypes must be a cell array and a struct */
  if(!mxIsCell(prhs[0])) {
    mexErrMsgIdAndTxt("MATLAB:trajgen:inputTypes", "The first input must be a cell array.");
  }else if(nrhs > 1 && !mxIsStruct(prhs[1]) && !mxIsEmpty(prhs[1])){
    mexErrMsgIdAndTxt("MATLAB:trajgen:inputTypes", "The second input must be a struct.");
  }else if(nrhs > 2 && !mxIsStruct(prhs[2]) && !mxIsEmpty(prhs[2])){
    mexErrMsgIdAndTxt("MATLAB:trajgen:inputTypes", "The third input must be a struct.");
    }
  
  /* Create struct for the return argument. */
  /* Dummy stuff for now */
  fields[0] = "Field1";
  /* fields[1]="Field2"; */
  plhs[0] = mxCreateStructMatrix(1, 1, 1, fields);
  // mexPrintf(fields[0]); 
  
  /* Assign pointers to each input and output. */
  /* x = mxGetPr(prhs[0]); */
  /* y = mxGetPr(plhs[0]); */
  
  /* Call the trajgen subroutine. */
  /* trajgen(y,x); */
  return;
}
