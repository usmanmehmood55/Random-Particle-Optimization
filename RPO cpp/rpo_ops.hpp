#pragma once

#include <iostream>
#include <cmath>

#define NPTS 300

struct point
{
	double x = 0;
	double y = 0;
	double obstacle_potential = 0;
	double goal_potential = 0;
};

struct gaussian_point
{
	double x = 0;
	double y = 0;
	int height = 0;
	int width = 0;
};

/**
 * @brief calculates gaussian potential for a single point in relation
 * to a gaussian point
 * 
 * @param gaussian 
 * @param _point 
 * @return double 
 */
double potential(gaussian_point gaussian, point _point)
{
	double ds_sq = (pow((_point.x - gaussian.x), 2) + pow((_point.y - gaussian.y), 2));
	double potntl = gaussian.height * exp(-1 * (double)gaussian.width * ds_sq);
	return potntl;
}

/**
 * @brief Sets the non gaussian point object
 * 
 * @param x x coordinate of point
 * @param y y coordinate of point
 * @return point 
 */
point set_point(double x, double y)
{
	point _point;
	_point.x = x;
	_point.y = y;
	return _point;
}

/**
 * @brief Sets the gaussian object with the given parameters
 * For goal object, the height must be negative. For obstacle object, 
 * the height must be positive.
 * 
 * @param x x coordinate of the gaussian object
 * @param y y coordinate of the gaussian object
 * @param height height of the gaussian object
 * @param width width of the gaussian object
 * @return gaussian_point 
 */
gaussian_point set_gaussian(double x, double y, int height, int width)
{
	gaussian_point _gaussian_point;
	_gaussian_point.x = x;
	_gaussian_point.y = y;
	_gaussian_point.width = width;
	_gaussian_point.height = height;
	return _gaussian_point;
}

/**
 * @brief Sets coordinates of artificial points by calculating the angles 
 * depending on number of points, and moving one step_size away from robot
 * in each angle. Then calculates potentials of each point.
 * 
 * @param step_size the global step size for robot to move in any instance
 * @param obstacle obstacle instance
 * @param goal goal instance
 * @param robot robot instance
 * @param artificial_points array of artificial points
 */
void set_artificial_points(
	double step_size,
	gaussian_point obstacle,
	gaussian_point goal,
	point robot,
	point artificial_points[NPTS])
{
	for (uint16_t i = 1; i <= NPTS; i++)
	{
		point this_point;

		// Calculating coordinates of APs, and DTG for each
		double theta = ((i - 1) * (360 / (double)NPTS));
		this_point.x = robot.x + step_size * cos((theta * 3.14) / 180);
		this_point.y = robot.y + step_size * sin(theta * 3.14 / 180);

		// Calculating potentials of APs
		this_point.obstacle_potential = potential(obstacle, this_point);
		this_point.goal_potential = potential(goal, this_point);

		artificial_points[i] = this_point;
	}
}

/**
 * @brief selects the best artificial point to move to based on their 
 * potentials with respect to the positions and potentials of the
 * goal, obstacle, and robot.
 * 
 * @param robot robot instance
 * @param goal goal instance
 * @param ap artificial points array
 * @return uint16_t index of the best artificial point
 */
uint16_t select_ap(point robot, gaussian_point goal, point *ap)
{
	double error_potential[NPTS + 1];
	double error_distance[NPTS + 1];

	double robot_total_potential = robot.goal_potential + robot.obstacle_potential;
	for (uint16_t i = 1; i <= NPTS; i++)
	{
		// Calculating errors in potentials and distances
		double ap_total_potential = ap[i].goal_potential + ap[i].obstacle_potential;
		error_potential[i] = ap_total_potential - robot_total_potential;
		error_distance[i] = distance(ap[i], goal) - distance(robot, goal);
	}

	uint16_t select_index = 0;
	double selected_error_distance = error_distance[1];
	double selected_error_potential = error_potential[1];
	for (uint16_t i = 1; i <= NPTS; i++)
	{
		if ((error_potential[i] < 0) && (error_distance[i] < 0))
		{
			if (error_potential[i] <= selected_error_potential)
			{
				selected_error_potential = error_potential[i];
				select_index = i;
			}
		}
	}
	if (select_index == 0)
	{
		selected_error_distance = error_distance[1];
		selected_error_potential = error_potential[1];

		for (int i = 1; i <= NPTS; i++)
		{
			if (error_distance[i] <= selected_error_distance)
			{
				selected_error_distance = error_distance[i];
				select_index = i;
			}
		}
	}
	return select_index;
}

double distance(gaussian_point a, gaussian_point b)
{
	return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

double distance(point a, gaussian_point b)
{
	return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

double distance(point a, point b)
{
	return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}