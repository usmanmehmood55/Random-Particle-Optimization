clc
clear

robot = [0, 0];                                         % x,y coordinates of robot
obstacle = [input('x_obst?   '), input('y_obst?   ')];	% x,y coordinates of obstacle
a_obstacle = 10;                                         % height of Obstacle
u_obstacle = 10;                                        % width of Obstacle
goal = [100, 100];                                      % x,y coordinates of goal
a_goal = 1;                                             % depth of Goal
u_goal = 5;                                             % width of Goal
NPTS = 6;                                               % number of APS

Ct = 0.01 * (norm(robot - obstacle));

x_robot = [];
y_robot = [];

disp('Step Size:')
disp(Ct)
disp('Number of APs:')
disp(NPTS)

disp('Going to loop')

error_j(NPTS)=0;        % Error in potential
error_d(NPTS)=0;        % Error in distance
artificial_pt_x = 0;    %
artificial_pt_y = 0;    %
AP_j_obst(NPTS)=0;      % Potential of obstacle wrt AP
AP_j_goal(NPTS)=0;      % Potential of goal wrt AP
AP_j_total(NPTS)=0;     % Total potential of the AP
AP_DTG(NPTS)=0;         % AP's distance to goal
theta(NPTS)=0;          % Angle of AP wrt MR and X-axis
itration = 1;           % Counts the number of iterations it takes for MR to reach goal

while 1
    
    % Initial Calculations, for MR, obstacle and goal.
    j_obstacle = potential(a_obstacle, u_obstacle, robot, obstacle);
    j_goal = potential(-a_goal, u_goal, robot, goal);
    j_total = j_obstacle + j_goal;
    DTG = norm(robot - goal);
    
    % Calculating angles for APs
    for  i = 1:NPTS
        theta(i) = (i - 1) * (360 / NPTS);
    end
    
    % Creating APs around the MR
    for  i = 1:NPTS
        
        % Calculating coordinates of APs, and DTG for each AP
        artificial_pt_x(i) = robot(1) + Ct * cos((theta(i) * 3.14) / 180);
        artificial_pt_y(i) = robot(2) + Ct * sin(theta(i) * 3.14 / 180);
        artificial_pt = [artificial_pt_x(i), artificial_pt_y(i)];
        AP_DTG(i) = norm(artificial_pt - goal);
        
        % Calculating potentials of APs
        AP_j_obst(i) = potential(a_obstacle, u_obstacle, artificial_pt, obstacle);
        AP_j_goal(i) = potential(-a_goal, u_goal, artificial_pt, goal);
        AP_j_total(i) = AP_j_obst(i) + AP_j_goal(i);
        
        % Calculating errors in potentials and distances
        error_j(i) = AP_j_total(i) - j_total;
        error_d(i) = AP_DTG(i) - DTG;
    end
    
    % Selection of best AP
    %-----------------------------------------------------------------------------
    AP_index = AP_selector(NPTS, error_d, error_j);
    %-----------------------------------------------------------------------------
    
    
    % Moving MR to new coordinates
    %-----------------------------------------------------------------------------
    robot = [artificial_pt_x(AP_index), artificial_pt_y(AP_index)]
    DTG = norm(robot - goal);
    
    x_robot = [x_robot robot(1)];
    y_robot = [y_robot robot(2)];
    axis([0 110 0 110])
    plot (x_robot, y_robot)
    
    hold on
    theta_for_plot = linspace(0,2*pi);
    x_for_plot = u_obstacle*cos(theta_for_plot) + obstacle(1);
    y_for_plot = u_obstacle*sin(theta_for_plot) + obstacle(2);
    axis([0 110 0 110])
    plot(x_for_plot, y_for_plot, 'r')
    
    x_for_plot = u_goal*cos(theta_for_plot) + goal(1);
    y_for_plot = u_goal*sin(theta_for_plot) + goal(2);
    axis([0 110 0 110])
    plot(x_for_plot, y_for_plot, 'g')
    hold off
    grid
    %-----------------------------------------------------------------------------
    
    
    % Stopping criteria for the algorithm
    if DTG <= Ct
        
        disp('Reached Target');
        disp('Number of itrations = ');
        disp( itration );
        itration = 1;
        return
        
    else
        itration = itration + 1;
    end
end