void RPO()
{
  float   errorJ      [NPTS];   // Error in potential
  float   errorD      [NPTS];   // Error in distance
  float   x           [NPTS];   // X coordinates of APs
  float   y           [NPTS];   // Y coordinates of APs
  float   AP_J_Obst   [NPTS];   // Potential of obstacle wrt AP
  float   AP_J_Goal   [NPTS];   // Potential of goal wrt AP
  float   AP_J_Total  [NPTS];   // Total potential of the AP
  float   AP_DTG      [NPTS];   // AP's distance to goal
  float   theta       [NPTS];   // Angle of AP wrt MR and X-axis

  int     itration = 1;         // Counts the number of iterations it takes for MR to reach goal
  while (itration != 0)
  {
    // Initial Calculations, for MR, obstacle and goal.
    float J_Obstacle  =  aObst*exp( -uObst * (sq(xRobot - xObstacle) + sq(yRobot - yObstacle)) );
    float J_Goal      = -aGoal*exp( -uGoal * (sq(xRobot - xGoal) + sq(yRobot - yGoal)) );
    float J_Total     = J_Obstacle + J_Goal;
    float DTG         = sqrt(sq(xRobot - xGoal) + sq(yRobot - yGoal));

    // Calculating angles for APs
    for (int i = 1; i <= NPTS; i++)
    {
      theta[i] = (i - 1) * (360 / NPTS);
    }

    // Creating APs around the MR
    for (int i = 1; i <= NPTS; i++)
    {
      // Calculating coordinates of APs, and DTG for each AP
      x[i] = xRobot + Ct * cos((theta[i] * 3.14) / 180);
      y[i] = yRobot + Ct * sin(theta[i] * 3.14 / 180);
      AP_DTG[i] = sqrt(sq(xGoal - x[i]) + sq(yGoal - y[i]));

      // Calculating potentials of APs
      AP_J_Obst[i] =  aObst*exp( -uObst * (sq(x[i] - xObstacle) + sq(y[i] - yObstacle)) );
      AP_J_Goal[i] = -aGoal*exp( -uGoal * (sq(x[i] - xGoal) + sq(y[i] - yGoal)) );
      AP_J_Total[i] = AP_J_Obst[i] + AP_J_Goal[i];

      // Calculating errors in potentials and distances
      errorJ[i] = AP_J_Total[i] - J_Total;
      errorD[i] = AP_DTG[i] - DTG;
    }

    // Selection of best AP
    unsigned char L = selectAP(errorJ, errorD);

    // Moving MR to new coordinates
    xRobot = x[L];
    yRobot = y[L];

    Serial.println(); Serial.print("X = "); 
    Serial.print(xRobot); Serial.print("\t"); Serial.print("Y = "); 
    Serial.print(yRobot); Serial.print("\t");
    Serial.print("DTG = "); Serial.print( sqrt(sq(xRobot - xGoal) + sq(yRobot - yGoal)) );

    // Stopping criteria for the algorithm
    if (  (abs((xGoal - xRobot)) < 1) && (abs((yGoal - yRobot)) < 1)  )
    {
      Serial.println(); Serial.print("Reached Target");Serial.println();
      Serial.print("Number of itrations = "); Serial.print(itration);
      itration = 0;
    }
    else
    {
      itration++;
    }
  }
}
