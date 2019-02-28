unsigned char selectAP(float errorJ[NPTS], float errorD[NPTS])
{
  unsigned char L = 0;
  float S_errorD = errorD[1];
  float S_errorJ = errorJ[1];
  for (int i = 1; i <= NPTS; i++)
  {
    if ((errorJ[i] < 0) && (errorD[i] < 0))
    {
      if (errorJ[i] <= S_errorJ)
      {
        S_errorJ = errorJ[i];
        L = i;
      }
    }
  }
  if (L == 0)
  {
    S_errorD = errorD[1];
    S_errorJ = errorJ[1];

    for (int i = 1; i <= NPTS; i++)
    {
      if (errorD[i] <= S_errorD)
      {
        S_errorD = errorD[i];
        L = i;
      }
    }
  }
  return L;
}
