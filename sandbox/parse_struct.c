#include "mex.h"
#include "string.h"

void
mexFunction (int nlhs, mxArray* plhs[], int nrhs, 
	     const mxArray* prhs[])
{ 
  int nfields;
  mxArray *tmp;  
  char *vname = "velocity";
  const char *fnames[2];
 

  fnames[0] = "v";
  fnames[1] = "v2";

  if (nrhs != 1 || ! mxIsStruct (prhs[0]) || mxGetNumberOfElements (prhs[0]) != 1)
    mexErrMsgTxt ("Expects struct with only one element");

  nfields = mxGetNumberOfFields(prhs[0]);

  plhs[0] = mxCreateStructMatrix(1, 1, 2, fnames);

  mexPrintf("You have %d fields\n", nfields);

  
  tmp = mxGetField(prhs[0],0,vname);
  if (tmp==NULL ){
    mexPrintf("%s is not a field in the struct\n",vname);
    mexErrMsgTxt("Missing velocity field");
  }
  
  //if (mxIsChar(tmp)){
    //mexPrintf("field1 is a string");}
  //else{
  //mexPrintf("field1 is NOT a string");}
  
  //fout = mxDuplicateArray(prhs[0]);
  
  //fout = mxCreateCellMatrix(1, 1);

  //mxSetCell( fout, 0,  mxGetCell (prhs[0], 0));

  //plhs[0] = mxGetFieldByNumber (mxGetCell (prhs[0], 0),0,0);
  
  //plhs[0] = tmp;
}
