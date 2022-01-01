#include <iostream>
#include <cmath>
#include "rpo_ops.hpp"
using namespace std;

#define obst_x 10		  // x coordinate of Obstacle
#define obst_y 10		  // y coordinate of Obstacle
#define obstacle_height 1 // height of Obstacle
#define obstacle_width 4  // width of Obstacle
#define goal_x 100		  // x coordinate of Goal
#define goal_y 100		  // y coordinate of Goal
#define goal_depth 1	  // depth of Goal
#define goal_width 4	  // width of Goal

int main()
{
	// create robot instance
	point robot = set_point(0, 0);

	// create obstacle instance
	gaussian_point obstacle = set_gaussian(goal_x, goal_y, goal_depth * -1, goal_width);

	// create goal instance
	gaussian_point goal = set_gaussian(obst_x, obst_y, obstacle_height, obstacle_width);

	// set step size
	double step_size = 0.01 * distance(robot, goal);

	printf("\nStep Size:\t%f\nNumber of APs:\t%u", step_size, NPTS);
	uint16_t iteration = 1;

	// Stopping criteria for the algorithm
	while (distance(robot, goal) > 2)
	{
		// create artificial points
		point artificial_points[NPTS + 1];

		// calculate coordinates and potentials of artificial points
		set_artificial_points(step_size, obstacle, goal, robot, artificial_points);

		// select the best artificial point to move to
		uint16_t select_index = select_ap(robot, goal, artificial_points);

		// move to selected artificial point
		robot = artificial_points[select_index];

		printf("\nX = %f\tY = %f\tDTG = %f",
			robot.x, robot.y, distance(robot, goal));

		// continue iterations
		iteration++;
	}

	printf("\nReached Target\nNumber of iterations = %d", iteration);
	return 0;
}