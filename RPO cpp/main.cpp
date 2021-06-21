/*
 * MR =   Mobile Robot
 * AP =   Artificial Point
 * DTG =  Distance to Goal
 */

#include <iostream>
#include <cmath>

using namespace std;

float xRobot = 0;     // x coordinate of present position of MR
float yRobot = 0;     // y coordinate of present position of MR
int xObstacle = 10;   // x coordinate of Obstacle
int yObstacle = 10;   // y coordinate of Obstacle
int aObst = 1;        // height of Obstacle
int uObst = 4;        // width of Obstacle
int xGoal = 100;      // x coordinate of Goal
int yGoal = 100;      // y coordinate of Goal
int aGoal = 1;        // depth of Goal
int uGoal = 4;        // width of Goal
const int NPTS = 300; // number of APS

int DTG_global = 0;

// Step Size
float Ct = 0.1 * (sqrt(pow((xRobot - xObstacle), 2) + pow((yRobot - yObstacle), 2)));

void RPO();
unsigned char selectAP(float errorJ[NPTS], float errorD[NPTS]);

int main()
{
  cout << endl;
  cout << "Step Size:\t";
  cout << Ct;
  cout << endl;
  cout << "Number of APs:\t";
  cout << NPTS;
  cout << endl;

  while (DTG_global > 2)
  {
    RPO();
  }
  return 0;
}

/*-------------------------------------------------------------------*/
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

/*-------------------------------------------------------------------*/
void RPO()
{
  float errorJ[NPTS];     // Error in potential
  float errorD[NPTS];     // Error in distance
  float x[NPTS];          // X coordinates of APs
  float y[NPTS];          // Y coordinates of APs
  float AP_J_Obst[NPTS];  // Potential of obstacle wrt AP
  float AP_J_Goal[NPTS];  // Potential of goal wrt AP
  float AP_J_Total[NPTS]; // Total potential of the AP
  float AP_DTG[NPTS];     // AP's distance to goal
  float theta[NPTS];      // Angle of AP wrt MR and X-axis

  int itration = 1; // Counts the number of iterations it takes for MR to reach goal
  while (itration != 0)
  {
    // Initial Calculations, for MR, obstacle and goal.
    float J_Obstacle = aObst * exp(-uObst * (pow((xRobot - xObstacle), 2) + pow((yRobot - yObstacle), 2)));
    float J_Goal = -aGoal * exp(-uGoal * (pow((xRobot - xGoal), 2) + pow((yRobot - yGoal), 2)));
    float J_Total = J_Obstacle + J_Goal;
    float DTG = sqrt(pow((xRobot - xGoal), 2) + pow((yRobot - yGoal), 2));
    DTG_global = DTG;

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
      AP_DTG[i] = sqrt(pow((xGoal - x[i]), 2) + pow((yGoal - y[i]), 2));

      // Calculating potentials of APs
      AP_J_Obst[i] = aObst * exp(-uObst * (pow((x[i] - xObstacle), 2) + pow((y[i] - yObstacle), 2)));
      AP_J_Goal[i] = -aGoal * exp(-uGoal * (pow((x[i] - xGoal), 2) + pow((y[i] - yGoal), 2)));
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

    cout << endl;
    cout << "X = ";
    cout << xRobot;
    cout << "\t";
    cout << "Y = ";
    cout << yRobot;
    cout << "\t";
    cout << "DTG = ";
    cout << sqrt(pow((xRobot - xGoal), 2) + pow((yRobot - yGoal), 2));

    // Stopping criteria for the algorithm
    if ((abs((xGoal - xRobot)) < 1) && (abs((yGoal - yRobot)) < 1))
    {
      cout << endl;
      cout << "Reached Target";
      cout << endl;
      cout << "Number of itrations = ";
      cout << itration;
      itration = 0;
    }
    else
    {
      itration++;
    }
  }
}