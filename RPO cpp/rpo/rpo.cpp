#include "rpo.hpp"

#define pi (double)3.14159265358979323

/**
 * @brief Calculates the distance between two gaussian points
 * 
 * @param a       first gaussian point
 * @param b       second gaussian point
 * 
 * @return double distance between the points
 */
double distance(gaussian_point a, gaussian_point b)
{
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

/**
 * @brief Calculates the distance between a point and a gaussian point
 * 
 * @param a       point
 * @param b       gaussian point
 * 
 * @return double distance between the points
 */
double distance(point a, gaussian_point b)
{
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

/**
 * @brief Calculates the distance between a gaussian point and a point
 * 
 * @param a       gaussian point
 * @param b       point
 * 
 * @return double distance between the points
 */
double distance(gaussian_point a, point b)
{
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

/**
 * @brief Calculates the distance between two points
 * 
 * @param  a      first point
 * @param  b      second point
 * 
 * @return double distance between the points
 */
double distance(const point a, const point b)
{
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

/**
 * @brief Calculates gaussian potential for a single point in relation
 * to a gaussian point
 * 
 * @param  p_gaussian 
 * @param  p_point 
 * 
 * @return double 
 */
double potential(const gaussian_point * p_gaussian, const point * p_point)
{
    double dist_squared = (pow((p_point->x - p_gaussian->x), 2) + pow((p_point->y - p_gaussian->y), 2));
    double potential    = p_gaussian->height * exp(-1 * (double)p_gaussian->width * dist_squared);
    return potential;
}

/**
 * @brief calculates gaussian potential for a single point in relation
 * to a gaussian point
 * 
 * @param p_point
 * @param p_gaussian
 * 
 * @return double 
 */
double potential(const point * p_point, const gaussian_point * p_gaussian)
{
    return potential(p_gaussian, p_point);
}

/**
 * @brief Sets the non gaussian point object
 * 
 * @param x      x coordinate of point
 * @param y      y coordinate of point
 * 
 * @return point filled object
 */
point set_point(double x, double y)
{
    return (point){.x = x, .y = y};
}

/**
 * @brief Sets the gaussian object with the given parameters
 * For goal object, the height must be negative. For obstacle object, 
 * the height must be positive.
 * 
 * @param x               x coordinate of the gaussian object
 * @param y               y coordinate of the gaussian object
 * @param height          height of the gaussian object
 * @param width           width of the gaussian object
 * 
 * @return gaussian_point filled object
 */
gaussian_point set_gaussian(double x, double y, int height, int width)
{
    gaussian_point _gaussian_point = 
    {
        .x      = x,
        .y      = y,
        .height = height,
        .width  = width,
    };
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
    const gaussian_point * p_obstacle,
    const gaussian_point * p_goal,
    point robot,
    point artificial_points[NPTS])
{
    for (uint16_t  i = 1; i <= NPTS - 1; i++)
    {
        // calculate the angle of artificial point
        double theta = ((i - 1) * ((double)360 / (double)NPTS));

        // create a new point instance and set its parameters
        point this_point = 
        {
            .x                  = robot.x + (step_size * cos((theta * pi) / 180.0)),
            .y                  = robot.y + (step_size * sin(theta * pi / 180.0)),
            .obstacle_potential = potential(p_obstacle, &this_point),
            .goal_potential     = potential(p_obstacle, &this_point),
        };

        artificial_points[i] = this_point;
    }
}

/**
 * @brief selects the best artificial point to move to based on their 
 * potentials with respect to the positions and potentials of the
 * goal, obstacle, and robot.
 * 
 * @param robot             robot instance
 * @param goal              goal instance
 * @param artificial_points artificial points array
 * 
 * @return uint16_t         index of the best artificial point
 */
uint16_t select_ap(const point * p_robot, const gaussian_point * p_goal, point artificial_points[NPTS])
{
    double error_potential[NPTS];
    double error_distance [NPTS];

    double robot_total_potential = p_robot->goal_potential + p_robot->obstacle_potential;
    for (uint16_t this_point = 0; this_point < NPTS; this_point++)
    {
        // Calculating errors in potentials and distances
        double ap_total_potential = artificial_points[this_point].goal_potential + artificial_points[this_point].obstacle_potential;
        error_potential[this_point] = ap_total_potential - robot_total_potential;
        error_distance[this_point] = distance(artificial_points[this_point], * p_goal) - distance(* p_robot, * p_goal);
    }

    uint16_t select_index             = 0;
    double   selected_error_distance  = error_distance[1];
    double   selected_error_potential = error_potential[1];
    for (uint16_t this_point = 0; this_point < NPTS; this_point++)
    {
        if ((error_potential[this_point] < 0) && (error_distance[this_point] < 0))
        {
            if (error_potential[this_point] <= selected_error_potential)
            {
                selected_error_potential = error_potential[this_point];
                select_index             = this_point;
            }
        }
    }

    if (select_index == 0)
    {
        selected_error_distance  = error_distance [1];
        selected_error_potential = error_potential[1];

        for (uint16_t this_point = 0; this_point < NPTS; this_point++)
        {
            if (error_distance[this_point] <= selected_error_distance)
            {
                selected_error_distance = error_distance[this_point];
                select_index            = this_point;
            }
        }
    }

    return select_index;
}