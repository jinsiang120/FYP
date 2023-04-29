%% Initialization
numRobots = 10;
leaderInit = 1;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.10;
env.showTrajectory = true;
rng(3);
if numRobots == 20 || numRobots == 30
    poses = [(randi(29,1,numRobots)); (randi(29,1,numRobots))
         pi*rand(1,numRobots)];
else
    poses = [(randi(20,1,numRobots)); (randi(20,1,numRobots))
         pi*rand(1,numRobots)];
end


%% Sensor Properties
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 9;
    detector.fieldOfView = pi*2;
    detectors{rIdx} = detector;
end
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals'
env.showTrajectory = false;


%% Create Map

ObsMap = binaryOccupancyMap(30,30,5);
walls = zeros(150,150);
setOccupancy(ObsMap,[1 1],walls,"grid")
env.mapName = 'ObsMap';
%% Simulation Loop

sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:85;        % Time array    
vel = zeros(3,numRobots);
xAxis = (poses(1,:))';
cal_circum = 1;

init = 0;
yAxis = (poses(2,:))';
final = zeros(numRobots,2);

for idx = 2:numel(tVec)
   % Update the environment
   xlim([0 20]);
   ylim([0 20]);
   env(1:numRobots, poses);
   s =[];
   t =[];
   weight = [];
   weightMatrix = [];
   sum_center = [0,0];

% Read the sensor for each robot
    for rIdx = 1:numRobots
       detections = step(detectors{rIdx});
       
        if(~isempty(detections))
               [numRows,numCols] = size(detections);
               listofDetected = (detections(:,numCols))';
               newWeight = (detections(:,1))';
              
               for i = 1:numRows
                   s = horzcat(s,rIdx);
               end
               
               % For Proximity Graph
               t = horzcat(t,listofDetected);
               weight = horzcat(weight,newWeight);
               bufferzero =zeros(1, numRobots);
               weightMatrix= [weightMatrix;bufferzero];
                    for a = 1:numRows
                        bufferDetect = detections(a,3);
                        weightMatrix(rIdx,bufferDetect) = detections(a,1);
                    end
                
               if cal_circum == 1
                   % For Average Circumcenter position
                    neighbor_pose_x = []
                    neighbor_pose_y = []
                
                    for i=1:numRows
                        buffer_neigh = detections(i,3);
                        buffer_pose_x = poses(1,buffer_neigh);
                        buffer_pose_y = poses(2,buffer_neigh);
                        neighbor_pose_x = cat(1,buffer_pose_x,neighbor_pose_x);
                        neighbor_pose_y = cat(1,buffer_pose_y,neighbor_pose_y);
                    end
                    
                    [center,radius] = minboundcircle(neighbor_pose_x,neighbor_pose_y);
                    final(rIdx,:) = center;
                   
               end
        end
    end
    total_d = 0;
    goalRadius = 2;
    if cal_circum ==1
      
        for i= 1:numRobots
           pose = poses(1:2,i)';
           final_p = final(i,:);
           distanceToGoal = norm(pose- final_p);
            
            total_d = total_d + distanceToGoal;             
        end
        
        if total_d < goalRadius
                  cal_circum = 0;
        end
    end

  
   % Information from each of the robot is the distance which relates
   % to the weight of edges and the weight matrix
    
   if init == 0
        G = graph(s,t,weight);
        if  ismultigraph(G)
                 G = simplify(G);
                 figure(3);
                 plot(G,'EdgeLabel',G.Edges.Weight);
        end
   end

    %Execute movement of all robots 
    
    for rIdx = 1:numRobots
        vel(:,rIdx)= rendezvousController(poses,rIdx,final);
    end

    % Discrete integration of pose 
     poses = poses + vel*sampleTime;

     xpose = (poses(1,:))';
     xAxis = horzcat(xAxis,xpose);
     ypose = (poses(2,:))';
     yAxis = horzcat(yAxis,ypose);
     init =1;

end

 [nR,nC] = size(xAxis);
 objfunc1 = [];

 for j = 1 :nC
     objfunc1(1,j) = 0;
     for i = 1:nR
         buffer_x = xAxis(i,j);
         buffer_y = yAxis(i,j);
         final_X = final(i,1);
         final_Y = final(i,2);
         distance = sqrt((final_X - buffer_x).^2 + (final_Y - buffer_y).^2);
         objfunc1(1,j) = distance +objfunc1(1,j);
     end
 end

%  objfunc = abs(objfunc);



figure(6);
plot(tVec,objfunc1);
title("Objective Function vs Time");
xlabel("Time(s)");
ylabel("Objective Function");
xlim([0 100]);
ylim([-20 50]);
grid on  

figure(7)
title("Trajectory of Mobile Robot (Circumcenter Algortihm) ")
hold on
xlim([0 20]);
ylim([0 20]);

% Define a custom color order
colorOrder = lines(numRobots);

% Set the color order for the current axes
set(gca, 'ColorOrder', colorOrder);

for i = 1:numRobots
    colorIndex = mod(i-1, size(colorOrder,1)) + 1;
    plot(xAxis(i,1),yAxis(i,1),Marker="o",Color=colorOrder(colorIndex,:),MarkerSize=3)
    plot(xAxis(i,:),yAxis(i,:),LineWidth=1.25,Color=colorOrder(colorIndex,:))
    % Label the marker with the robot number
    text(xAxis(i,1), yAxis(i,1), sprintf('%d', i), ...
       "HorizontalAlignment","left","VerticalAlignment","cap",...
         'Color', 'red');
end
hold off

totaldistanceTravel = 0;
  for i = 2:nC
    for j =1:numRobots
        nowX = xAxis(j,i);
        prevX = xAxis(j,i-1);
        nowY = yAxis(j,i);
        prevY = yAxis(j,i-1);

        distanceTravel =sqrt((nowX - prevX).^2 + (nowY - prevY).^2);
        totaldistanceTravel = distanceTravel + totaldistanceTravel;
    end
  end
 



%% Rendezvous Controller
function vel = rendezvousController(poses,rIdx,final)

    final_1 = final(rIdx,:);

    final2 = final_1.';
    pose = poses(:,rIdx);       % Current Robot Pose
    controller = controllerPurePursuit;
    controller.Waypoints = final_1;
    controller.LookaheadDistance = 0.3;
    controller.DesiredLinearVelocity = 0.3;
    controller.MaxAngularVelocity = 3.5;

    robotGoal = final2;
    distanceToGoal = norm(pose(1:2)- robotGoal);
    goalRadius = 0.05;
    
   if distanceToGoal > goalRadius
        [v,w,lookAheadPt] = controller(pose);
   else
            v = 0;
            w = 0;
   end

    % Convert to global velocity
    vel = bodyToWorld([v;0;w],pose);
    
end
%% 
% DIJKSTRA Find length of shortest path between nodes in a graph
%
% D = dijkstra(A, p) 
% Takes a graph represented by its adjacency matrix 'A' along with a node 
% 'p' as input and returns a vector 'D' containing the length of the 
% shortest path from 'p' to all other nodes in the graph. 

% Copyright 2018 The MathWorks, Inc.
function [D,cost] = dijkstra(A, p,weightMatrix) %#codegen

    narginchk(2,3);
    [m, n] = size(A);

    % Assertions to make sure inputs are valid
    assert(m == n, "Input adjacency matrix for graph must be a square matrix");
    assert(rem(p, 1) == 0 && p <= m && p > 0, "Input src must be a node in the graph");
      
    % Initialization
    max = realmax;
    D = repmat(max, 1, m);
    cost = repmat(max, 1, m);
    D(p) = 0;
    cost(p) = 0;
    visited = false(1, m);
        
    for i = 1:m
        
        % Select next node to visit
        min = max;
        u = -1;
        
        for v = 1:n
            if ~visited(v) && D(v) <= min
                min = D(v);
                u = v;
            end
        end

        % Mark selected node as visited
        visited(u) = true;
        
        %{ 
          Update distances of nodes adjacent to selected node that are yet
          to be visited
        %}
        for v = 1:n
            if(~visited(v) && A(u, v) ~= 0 && D(u) ~= max)   %% Depends on A(u,v) the most
                distVal = D(u) + A(u, v);
                costD = cost(u) + weightMatrix(u,v);
                if distVal < D(v)
                    D(v) = distVal;
                end
                if costD < cost(v)
                    cost(v) = costD;
                end 
            end
        end
    end
end


