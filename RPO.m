clc
clear

xRobot = 0;       % x coordinate of present position of MR
yRobot = 0;       % y coordinate of present position of MR
xObstacle = input('xObstacle?   ');   % x coordinate of Obstacle
yObstacle = input('yObstacle?   ');   % y coordinate of Obstacle
aObst = 2;        % height of Obstacle
uObst = 5;        % width of Obstacle
xGoal = 100;      % x coordinate of Goal
yGoal = 100;      % y coordinate of Goal
aGoal = 2;        % depth of Goal
uGoal = 5;        % width of Goal
NPTS = 300;       % number of APS

Ct = 0.025 * (sqrt((xRobot - xObstacle)^2 + (yRobot - yObstacle)^2));

xRobotCords = [];
yRobotCords = [];

disp('Step Size:')
disp(Ct)
disp('Number of APs:')
disp(NPTS)

disp('Going to loop')

while sqrt((xRobot - xGoal)^2 + (yRobot - yGoal)^2) >= 1
    
    disp('in loop')
    errorJ(NPTS)=0;     % Error in potential
    errorD(NPTS)=0;     % Error in distance
    x(NPTS)=0;          % X coordinates of APs
    y(NPTS)=0;          % Y coordinates of APs
    AP_J_Obst(NPTS)=0;  % Potential of obstacle wrt AP
    AP_J_Goal(NPTS)=0;  % Potential of goal wrt AP
    AP_J_Total(NPTS)=0; % Total potential of the AP
    AP_DTG(NPTS)=0;     % AP's distance to goal
    theta(NPTS)=0;      % Angle of AP wrt MR and X-axis
    itration = 2;       % Counts the number of iterations it takes for MR to reach goal
    
    while itration ~= 1
        
        % Initial Calculations, for MR, obstacle and goal.
        J_Obstacle = aObst * exp(-uObst * ((xRobot - xObstacle)^2 + (yRobot - yObstacle)^2));
        J_Goal = -aGoal * exp(-uGoal * ((xRobot - xGoal)^2 + (yRobot - yGoal)^2));
        J_Total = J_Obstacle + J_Goal;
        DTG = sqrt((xRobot - xGoal)^2 + (yRobot - yGoal)^2);
        
        % Calculating angles for APs
        for  i = 1:NPTS
            theta(i) = (i - 1) * (360 / NPTS);
        end
        
        % Creating APs around the MR
        for  i = 1:NPTS
            
            % Calculating coordinates of APs, and DTG for each AP
            x(i) = xRobot + Ct * cos((theta(i) * 3.14) / 180);
            y(i) = yRobot + Ct * sin(theta(i) * 3.14 / 180);
            AP_DTG(i) = sqrt((xGoal - x(i))^2 + (yGoal - y(i))^2);
            
            % Calculating potentials of APs
            AP_J_Obst(i) = aObst * exp(-uObst * ((x(i) - xObstacle)^2 + (y(i) - yObstacle)^2));
            AP_J_Goal(i) = -aGoal * exp(-uGoal * ((x(i) - xGoal)^2 + (y(i) - yGoal)^2));
            AP_J_Total(i) = AP_J_Obst(i) + AP_J_Goal(i);
            
            % Calculating errors in potentials and distances
            errorJ(i) = AP_J_Total(i) - J_Total;
            errorD(i) = AP_DTG(i) - DTG;
        end
        
        % Selection of best AP
        %-----------------------------------------------------------------------------
        L = 1;
        S_errorD = errorD(1);
        S_errorJ = errorJ(1);
        for i = 1:NPTS
            
            if ((errorJ(i) < 0) && (errorD(i) < 0))
                
                if (errorJ(i) <= S_errorJ)
                    
                    S_errorJ = errorJ(i);
                    L = i;
                end
            end
        end
        if L == 1
            
            S_errorD = errorD(1);
            S_errorJ = errorJ(1);
            
            for i = 1:NPTS
                
                if errorD(i) <= S_errorD
                    S_errorD = errorD(i);
                    L = i;
                end
            end
        end
        %-----------------------------------------------------------------------------
        
        
        % Moving MR to new coordinates
        %-----------------------------------------------------------------------------
        xRobot = x(L);
        yRobot = y(L);
        DTG = sqrt((xRobot - xGoal)^2 + (yRobot - yGoal)^2);
        x_y_DTG = [xRobot yRobot DTG]
        
        xRobotCords = [xRobotCords xRobot];
        yRobotCords = [yRobotCords yRobot];
        plot (xRobotCords,yRobotCords)
        
        hold on
        plot(xObstacle, yObstacle, 'o', 'MarkerFaceColor','r')
        plot(xGoal, yGoal, 'o', 'MarkerFaceColor','g')
        hold off
        grid
        
        %-----------------------------------------------------------------------------
        
        
        % Stopping criteria for the algorithm
        if ((abs((xGoal - xRobot)) < 1) && (abs((yGoal - yRobot)) < 1))
            
            disp('Reached Target');
            disp('Number of itrations = ');
            disp( itration );
            itration = 1;
            return
            
        else
            itration = itration + 1;
        end
    end
end