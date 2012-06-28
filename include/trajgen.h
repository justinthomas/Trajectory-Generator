// This header file includes useful functions for trajgen.c

// basisgen differentiates an nth order polynomial basis vector d times and evaluates it at time t
float *basisgen(int n, int d, float t)
{
  int idx1, idx2, coeff;
  float *vec = malloc((n+1) * sizeof(float));
  
  for(idx1=0; idx1 < n+1; idx1++)
    {
      // Determine the coefficient
      coeff = 1;
      for(idx2=idx1; idx2 > idx1-d; idx2--)
	coeff = coeff*idx2;
      
      // And raise it to the appropriate power
      vec[idx1] = coeff*pow(t,idx1-d);
    }
  
  return vec;

}
