%% Initialization
numRobots = 15;
leaderInit = 1;
numleaders = 2;
env = MultiRobotEnv(numRobots);
env.hasWaypoints = true;
env.robotRadius = 0.15;
env.showTrajectory = true;
L = 0.15;
rng(3);
poses = [(randi(14,1,numRobots)); (randi(6,1,numRobots))
         pi*rand(1,numRobots)];
x_range = [1 14];
y_range = [12 14];
x_points = randi(x_range, [numleaders, 1]);
y_points = randi(y_range, [numleaders, 1]);
waypoints = ([x_points y_points]);
%% Create Map

ObsMap = binaryOccupancyMap(15,15,5)
walls = zeros(75,75);
walls(1,:) = 1; % Top wall
walls(end,:) = 1; % Bottom wall
walls(:,1) = 1; % Left wall
walls(:,end) = 1; % Right wall
walls(20:40,20:60) =1;
setOccupancy(ObsMap,[1 1],walls,"grid")
env.mapName = 'ObsMap';


%% Sensor Properties
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 3;
    detector.fieldOfView = pi*2;
    detectors{rIdx} = detector;
end
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals'
env.showTrajectory = false;

%% Simulation Loop

sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:75;        % Time array    
vel = zeros(3,numRobots);

comp_x = zeros(numRobots,1);
comp_y = zeros(numRobots,1);
[compCols, compRows] = size(comp_x);
xAxis = (poses(1,:))';
yAxis = (poses(2,:))';

for i = 1:compCols
    comp_x(i,1) = waypoints(1,1);
    comp_y(i,1) = waypoints(1,2);
end

obj = sqrt((comp_x - xAxis).^2 + (comp_y - yAxis).^2);

  for idx = 2:numel(tVec)
   % Update the environment
  
   env(1:numRobots, poses,waypoints);
   xlim([0 15]);
   ylim([0 15]);
   s =[];
   t =[];
   weight = [];
   weightMatrix = [];

    % Read the sensor for each robot
    for rIdx = 1:numRobots
       detections = step(detectors{rIdx});   
       [numRows, numCols] = size(detections);
       
       % Extract information from detections
       newWeight = detections(:, 1)';
       listofDetected = detections(:, numCols)';
       
       % Update s, t, and weight
       s = [s, ones(1, numRows) * rIdx];
       t = [t, listofDetected];
       weight = [weight, newWeight];
       
       % Update weightMatrix
       weightMatrix = [weightMatrix; zeros(1, numRobots)];
       for a = 1:numRows
           bufferDetect = detections(a, 3);
           weightMatrix(rIdx, bufferDetect) = detections(a, 1);
       end
    end

   % Information from each of the robot is the distance which relates
   % to the weight of edges and the weight matrix

   G = graph(s,t,weight);

        if  ismultigraph(G)
                 G = simplify(G);
        end

   % Leader Selection
          A = adjacency (G);          
          if leaderInit == 1
            [cost, bestLeaders,steps] = leaderSelect(A,weightMatrix,numRobots,numleaders); 
         
            rootedTree = graphContruction(G,cost,bestLeaders,steps)
            leaderInit = leaderInit + 1;
            color = zeros(numRobots,3);
            for i= 1 :numleaders
                leader = bestLeaders(i);
                color(leader,1) = 1;
                env.robotColors = color;
            end
            figure(4);
            plot(G,'EdgeLabel',G.Edges.Weight);
            xlim([0 10]);
            ylim([0 10]);
            plot(rootedTree);
                
            %   Path Generation for Leader
            final = allocation(bestLeaders,poses,waypoints); 
            leaderPath = cell(1, 0);
            for i = 1:numleaders
                startLocation = (poses(1:2,bestLeaders(i)))';
                mapInflated = copy(ObsMap)
                inflate(mapInflated,0.5);
                prm = robotics.PRM(mapInflated);
                prm.NumNodes = 100;
                prm.ConnectionDistance = 10;
                for j = 1:numleaders
                    if final(j) == bestLeaders(i)
                        ending = waypoints(j,:);
                    end
                end
                path = findpath(prm, startLocation, ending);
                leaderPath{i}= path;
            end
          end

    %Execute movement of all robots 
    for rIdx = 1:numRobots
        detections = step(detectors{rIdx});
        vel(:,rIdx)= rendezvousController(poses,rIdx,detections,bestLeaders,rootedTree,leaderPath);
    end

    % Discrete integration of pose 
     poses = poses + vel*sampleTime;

    % Objective Function
     xpose = (poses(1,:))';
     xAxis = horzcat(xAxis,xpose);
     ypose = (poses(2,:))';
     yAxis = horzcat(yAxis,ypose);
     distance = sqrt((comp_x - xpose).^2 + (comp_y - ypose).^2);
     obj = horzcat(obj,distance);
  end


 [~,leadrows] = size(bestLeaders);
 [nR,nC] = size(xAxis);
 objfunc = [];


  for j = 1 :nC
     objfunc(1,j) = 0;
     for i = 1:leadrows
        bufferleader = bestLeaders(i);
         
        for z = 1 :numleaders
            if final(z) == bestLeaders(i)
                ending = waypoints(z,:);
            end
        end 
        dfs = dfsearch(rootedTree, bufferleader);
        [childcol,~] = size(dfs);
        x_leader = ending(1);
        y_leader = ending(2);
        childcol = childcol -1;
        for k =1:childcol
            bufferfollower = dfs(k);
            x_follower = xAxis(bufferfollower,j);
            y_follower = yAxis(bufferfollower,j);
            distance = sqrt((x_leader - x_follower).^2 + (y_leader - y_follower).^2);
            objfunc(1,j) = objfunc(1,j) + distance;
        end
     end
  end
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
 


figure(6);
plot(tVec,objfunc);
title("Objective Function vs Time");
xlabel("Time(s)");
ylabel("Objective Function");
xlim([0 100]);
ylim([-20 50]);
grid on

%%
figure(7)
title("Trajectory of Mobile Robot during Rendezvous Task ")
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

for i = 1:numleaders
    plot(waypoints(i,1),waypoints(i,2),"x",MarkerSize=12,Color = "red")
end
hold off


%% Rendezvous Controller
function vel = rendezvousController(poses,rIdx,detections,bestleader,T,leader_path)


    [~, leaderrows] = size(bestleader);
    ref_path = 1;
    leader = 0;
    pose = poses(:,rIdx);       % Current Robot Pose
    [a,b] = size(detections);
    range = 0;
    angle = 0;
    w = 0;
    v = 0;

    for i = 1:leaderrows
        if rIdx == bestleader(i)
            leader = rIdx;
            ref_path = i;
        end
    end

    if rIdx == leader
        % Pure Pursuit Controller
        path = leader_path{ref_path};
        controller = controllerPurePursuit;
        controller.Waypoints = path;
        controller.LookaheadDistance = 0.3;
        controller.DesiredLinearVelocity = 0.3;
        controller.MaxAngularVelocity = 2.5;
        robotGoal = path(end,:)';
        distanceToGoal = norm(pose(1:2)- robotGoal);
        goalRadius = 0.05;
    end

    ranges = mean(detections(:,1));

    templeader = predecessors(T,rIdx);
    tempchild = successors(T,rIdx);

        for i = 1:a
            bufferleader = detections(i,b);
            if templeader == bufferleader
              range = detections(i,1);
              angle = detections(i,2);
            end
         end
    
  
    if rIdx ~= leader
        if (angle > pi/12 || angle < -pi/12) 
            if angle >pi/12
                w = 1;
            else
                w = -1;         
            end
            v =0;

        else
            if range > 0.05
                v = 0.5;
                w = 0;
            elseif range <= 0.05
                v = 0;
                w = 0;
            end
        end
    else
        v = 0;
        w = 0;
    end
    
    
    if rIdx == leader
        if distanceToGoal > goalRadius
        [v,w,lookAheadPt] = controller(pose);
        else
            v = 0;
            w = 0;
        end
    end

    [c,~] = size(tempchild);
    childrange =4;
    for i = 1:c
        bufferchild = tempchild(i);

         for x = 1:a
            if bufferchild == detections(x,3)
                childrange = detections(x,1);
            end    
         end
        if childrange > 2
            v = 0;
        end
    end

    % Convert to global velocity
    vel = bodyToWorld([v;0;w],pose);

end
%% Leader Selection
function [cost, bestLeaders,step] = leaderSelect(A,weightMatrix,numRobots,numleaders)

      step =[];
      costDistance =[];
       for p=1:numRobots
          % Calculate shortest distance from 'p' to every other node in graph G
           [D,cost] = dijkstra(A, p,weightMatrix);
           step = [step;D];
           costDistance = [costDistance;cost];

       end
       max = realmax;
       totalCost = max;
       [numRows2,numCols2] = size(A);
       C = nchoosek(1:numRows2, numleaders);
       
       cost = zeros(1, numleaders);
       


 for i=1 :size(C, 1)
     leaders = C(i, :);
     costofRow = 0;
     visited = false(1, numRows2);
     costing = repmat(realmax, numleaders, numRows2);
            for j= 1: numleaders
                a = leaders(j);
                visited(a) = 1;
            end
       

       stepping = 1;
       for k = 1: numRows2
       
            for z = 1:numleaders
                a = leaders(z);
                costing(:,a)=0;
                leaderstep =  step(a,:);
                leaderdistance = costDistance(a,:);
                for j = 1:numCols2
                    
                    bufferstep = leaderstep(j);
                    if bufferstep == stepping
                        xz = any(leaders(:) == j);
                        if xz == 0
                            buffercost = leaderdistance(j);
                            costing(z,j) = buffercost;
                            visited(j) = true;
                        end
                    end
                end

              if visited == true
                  for l = 1 :numCols2
                        buffercost = realmax;
                        buffer = costing(:,l);
                        for q = 1 :numleaders
                            if buffer(q) ~= 0 && buffer(q) < buffercost
                                 buffercost = buffer(q);
                                 costofRow = q;
                            end
                        end

                        for k = 1:numleaders
                            if k ~= costofRow
                                costing(k,l) =0;
    
                            end
                        end
                  end
                  total = sum(costing(:));
                  if total < totalCost
                        bestLeaders = leaders;
                        totalCost = total;
                        cost = costing;
                  end
                    break;
              end
            end

            stepping = stepping +1;
       end
 end
   
end

%%
function rootedTree = graphContruction(G,cost,bestLeaders,step)


   q = [];
   w = [];
  ref = G.Edges.EndNodes;
  [~,column1] = size(cost);
  refweight = (G.Edges.Weight);
  [refcols,~] = size(ref);
  [~,leadrows] = size(bestLeaders);
  visited = zeros(1,column1);

  
  for j= 1:leadrows
      leader = bestLeaders(j);
      bufferstep = step(leader,:);
      maxstep = max(bufferstep);
      [~,column] = size(bufferstep);
     
      for i = 1:maxstep
           for k = 1:column
                if i == bufferstep(k)
                    
                    for z = 1: refcols
                        comp1 =[leader,k];
                        comp2 = [k,leader];
                        if (ref(z,:) == comp1)
                            buffercost = refweight(z);
                        end
                        if(ref(z,:) == comp2)
                            buffercost = refweight(z);
                        end
                    end
                    if buffercost == cost(j,k)
                        q = [q,leader];
                        w = [w,k];
                        visited(k) =1;
                    end
                    
                    if buffercost ~= cost(j,k) && i >1
                    %Find their immediate parent
                    pparent = [];
                    costparent = [];
                    init = 1;
                    if visited(k)==0
                        for o = 1:i
                            for r = 1:column                                
                                if bufferstep(r) == o && o ~= i
                                    if o == 2 && init == 1 && i ==3
                                        secondlayerparent = pparent;
                                        secondlayercost = costparent;
                                        pparent = [];
                                        init = 2;
                                    end

                                    pparent = [pparent,r];
                                    comp3 =[leader,r];
                                    comp4 = [r,leader];

 
                                    for p = 1:refcols
                                        if (ref(p,:) == comp3)
                                            buffercost2 = refweight(p);
                                            costparent = [costparent,buffercost2];
                                        end
                                        if(ref(p,:) == comp4)
                                             buffercost2 = refweight(p);
                                             costparent = [costparent,buffercost2];
                                        end
                                    end
                                end

                                if o == i 
                                    totalcost=0;
                                    done = 0;
                                    [~,parentrow] = size(costparent);

                                    if init ==1
                                        for az = 1:parentrow
                                            comp6 = [pparent(az),k]
                                            comp7 = [k,pparent(az)]
                                            for p = 1:refcols
                                                if (ref(p,:) == comp6)
                                                    buffercosting = refweight(p);
                                                    totalcost = buffercosting + costparent(az);
                                                end
                                                if(ref(p,:) == comp7)
                                                     buffercosting = refweight(p);
                                                     totalcost = buffercosting + costparent(az);
                                                end
                                            end
                                            if totalcost == cost(j,k) && totalcost ~=0
                                                parent = pparent(az);
                                                q = [q,parent];
                                                w = [w,k];
                                                visited(k) =1;
                                                done = 1;
                                            end
                                            if done ==1;
                                                pparent = [];
                                                costparent = [];
                                                break;
                                            end
                                        end
                                    end

                                    if init ==2
                                        [~,parent2row] = size(secondlayerparent);
                                        [~,parentrow] = size(pparent);
                                        totalcost2 = 0;
                                        for az = 1:parent2row
                                            for as = 1:parentrow
                                                firstcomp1 = [secondlayerparent(az),pparent(as)];
                                                secondcomp1 = [pparent(as),secondlayerparent(az)];
                                                for p = 1:refcols
                                                    if (ref(p,:) == firstcomp1)
                                                        buffercosting = refweight(p);
                                                        totalcost2 = buffercosting + costparent(az);
                                                    end
                                                    if(ref(p,:) == secondcomp1)
                                                         buffercosting = refweight(p);
                                                         totalcost2 = buffercosting + costparent(az);
                                                    end
                                                end
                                                if totalcost2 ~=0
                                                 firstcomp2 = [k,pparent(as)];
                                                 secondcomp2 = [pparent(as),k];
                                                 for p = 1:refcols
                                                    if (ref(p,:) == firstcomp2)
                                                        buffercosting = refweight(p);
                                                        totalcost2 = buffercosting +totalcost2;
                                                    end
                                                    if(ref(p,:) == secondcomp2)
                                                         buffercosting = refweight(p);
                                                         totalcost2 = buffercosting +totalcost2;
                                                    end
                                                 end

                                                 if totalcost2 == cost(j,k)
                                                        parent = pparent(as);
                                                        q = [q,parent];
                                                        w = [w,k];
                                                        visited(k) =1;
                                                        done = 1;
                                                        break;

                                                 end
                                                end
                                            end

                                                 if done ==1
                                                    pparent = [];
                                                    costparent = [];
                                                    break;
                                                 end
                                        end
                                    end
                                end
                            end
                        end
                    end
                    end
                end
           end
      end
  end

 G = digraph(q,w);
 rootedTree = simplify(G);
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

%%
function visited = allocation(bestLeaders,poses,waypoints)

[numpoint,~] = size(waypoints);
[~,numleaders] = size(bestLeaders);
visited = zeros(1,numpoint);
max = realmax;
cost = repmat(max, 1, numpoint);

for j = 1:numpoint
    bufferpoint = waypoints(j,:);
    
    for i= 1 : numleaders
            done = 0;
            leader = bestLeaders(i);
            for k = 1:numpoint
                if visited(k) == leader
                    done =1;
                end
            end

            pose = poses(1:2,leader);
            distance = sqrt((pose(1) - bufferpoint(1)).^2 + (pose(2) - bufferpoint(2)).^2);
            if distance < cost(j) && done == 0
                cost(j) = distance;
                visited(j) = leader;
            end
     
    end
end

end