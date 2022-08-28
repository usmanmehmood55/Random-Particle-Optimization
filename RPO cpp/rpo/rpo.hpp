#pragma once

#include <iostream>
#include <cmath>
using namespace std;

#define NPTS 300 // number of artificial points to be generated for the mobile robot

typedef struct 
{
    double x = 0;
    double y = 0;
    double obstacle_potential = 0;
    double goal_potential = 0;
} point;

typedef struct
{
    double x = 0;
    double y = 0;
    int height = 0;
    int width = 0;
} gaussian_point;

/**
 * @brief Calculates the distance between two gaussian points
 * 
 * @param a first gaussian point
 * @param b second gaussian point
 * @return double distance between the points
 */
double distance(gaussian_point a, gaussian_point b);

/**
 * @brief Calculates the distance between a point and a gaussian point
 * 
 * @param a point
 * @param b gaussian point
 * @return double distance between the points
 */
double distance(point a, gaussian_point b);

/**
 * @brief Calculates the distance between a gaussian point and a point
 * 
 * @param a gaussian point
 * @param b point
 * @return double distance between the points
 */
double distance(gaussian_point a, point b);

/**
 * @brief Calculates the distance between two points
 * 
 * @param a first point
 * @param b second point
 * @return double distance between the points
 */
double distance(point a, point b);

/**
 * @brief Calculates gaussian potential for a single point in relation
 * to a gaussian point
 * 
 * @param gaussian 
 * @param _point 
 * @return double 
 */
double potential(gaussian_point gaussian, point _point);

/**
 * @brief calculates gaussian potential for a single point in relation
 * to a gaussian point
 * 
 * @param _point
 * @param gaussian
 * @return double 
 */
double potential(point _point, gaussian_point gaussian);

/**
 * @brief Sets the non gaussian point object
 * 
 * @param x x coordinate of point
 * @param y y coordinate of point
 * @return point 
 */
point set_point(double x, double y);

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
gaussian_point set_gaussian(double x, double y, int height, int width);

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
    point artificial_points[NPTS]);

/**
 * @brief Selects the best artificial point to move to based on their 
 * potentials with respect to the positions and potentials of the
 * goal, obstacle, and robot.
 * 
 * @param robot robot instance
 * @param goal goal instance
 * @param ap artificial points array
 * @return uint16_t index of the best artificial point
 */
uint16_t select_ap(point robot, gaussian_point goal, point *ap);