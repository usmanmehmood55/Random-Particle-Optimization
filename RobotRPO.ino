/*
 * MR =   Mobile Robot
 * AP =   Artificial Point
 * DTG =  Distance to Goal
 */

float   xRobot = 0;         // x coordinate of present position of MR
float   yRobot = 0;         // y coordinate of present position of MR

#define xObstacle   10      // x coordinate of Obstacle
#define yObstacle   10      // y coordinate of Obstacle
#define aObst       1       // height of Obstacle
#define uObst       4       // width of Obstacle

#define xGoal       100     // x coordinate of Goal
#define yGoal       100     // y coordinate of Goal
#define aGoal       1       // depth of Goal
#define uGoal       4       // width of Goal

#define NPTS        300     // number of APS
#define Ct  0.1 * (sqrt(sq(xRobot - xObstacle) + sq(yRobot - yObstacle)))   // Step Size

void RPO();
unsigned char selectAP(float errorJ[NPTS], float errorD[NPTS]);

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.print("Step Size:\t"); Serial.print(Ct); Serial.println();
  Serial.print("Number of APs:\t"); Serial.print(NPTS);  Serial.println();
}

void loop()
{
  RPO();
  while (1);
}
