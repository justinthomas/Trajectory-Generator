// This header file includes useful functions for trajgen.c

// basisgen differentiates an nth order polynomial basis vector d times and evaluates it at time t
float *basisgen(int n, int d, float t)
{
   
  int idx1, idx2, coeff;
  float *vec;

  for(idx1=0; idx1 < n+1; idx1++)
    {
      // Determine the coefficient using npermk: n!/(n-k)! where k is the order of the derivative
      coeff = 1;
      for(idx2=d+1; idx2 < idx1; idx2++)
	coeff = coeff*idx2;

      // Raise t to the respective power and multiply by the coefficent that resulted from the derivative
      vec[idx1] = coeff*pow(t,idx1);
    }
  
  return *vec;
}
