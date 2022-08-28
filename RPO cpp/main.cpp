#include <iostream>
#include <cmath>
#include "rpo.hpp"
using namespace std;

#define OBSTACLE_X      10  // x coordinate of Obstacle
#define OBSTACLE_Y      10  // y coordinate of Obstacle
#define OBSTACLE_HEIGHT 1   // height of Obstacle
#define OBSTACLE_WIDTH  4   // width of Obstacle
#define GOAL_X          100 // x coordinate of Goal
#define GOAL_Y          100 // y coordinate of Goal
#define GOAL_DEPTH      1   // depth of Goal
#define GOAL_WIDTH      4   // width of Goal

int main()
{
    // create robot instance
    point robot = set_point(0, 0);

    // create goal instance
    gaussian_point goal = set_gaussian(GOAL_X, GOAL_Y, GOAL_DEPTH * -1, GOAL_WIDTH);

    // create obstacle instance
    gaussian_point obstacle = set_gaussian(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_HEIGHT, OBSTACLE_WIDTH);

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

        printf("\nX = %f\tY = %f\tDTG = %f", robot.x, robot.y, distance(robot, goal));

        // continue iterations
        iteration++;
    }

    printf("\nReached Target\nNumber of iterations = %d", iteration);
    return 0;
}