#include "mex.h"

void
mexFunction (int nlhs, mxArray* plhs[], int nrhs, 
	     const mxArray* prhs[])
{ 
  int nfields;
  mxArray *fout;
  
  
  if (nrhs != 1 || ! mxIsCell (prhs[0]) || mxGetNumberOfElements (prhs[0]) != 1)
    mexErrMsgTxt ("Expects cell with only one element");
  
  nfields = mxGetNumberOfFields(mxGetCell (prhs[0], 0));

  mexPrintf("You have %d fields\n", nfields);

  //fout = mxDuplicateArray(prhs[0]);
  
  fout = mxCreateCellMatrix(1, 1);

  mxSetCell( fout, 0,  mxGetCell (prhs[0], 0));

  //plhs[0] = mxGetFieldByNumber (mxGetCell (prhs[0], 0),0,0);
  
  plhs[0] = fout;
}
