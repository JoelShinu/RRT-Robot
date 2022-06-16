%% Pseudocode for RRT algorithm 

% Qgoal //region that identifies success
% Counter = 0 //keeps track of iterations
% lim = n //number of iterations algorithm should run for
% G(V,E) //Graph containing edges and vertices, initialized as empty
% While counter < lim:
%     Xnew  = RandomPosition()
%     if IsInObstacle(Xnew) == True:
%         continue
%     Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
%     Link = Chain(Xnew,Xnearest)
%     G.append(Link)
%     if Xnew in Qgoal:
%         Return G
% Return G

% ---------------------------------------------------------------------- %

% clearvars
% close all
% 
% EPS = 2;
% Nodes = 1000;
% 
% map = imread("maps/map3.png");
% [map_dim1,map_dim2] = size(map);
% 
% start = [100,100];
% goal = [1100,700];
% 
% nodes(1,:) = start;
% figure(1)
% axis([0 map_dim1 0 map_dim2])
% imshow(map)
% hold on

function RRT(map,samples,radius,line_distance,start,goal)

map = imread(map);
imshow(map)
axis xy;
axis on;
hold on

[map_dim1,map_dim2] = size(map);


node(1,:) = [start,0];

plot(start(1),start(2), 'o', 'Color', 'Green','LineWidth',5,'Tag','Start');
text((start(1)+10),start(2),'\leftarrow START','FontSize',9,'Color','BLUE');

plot(goal(1),goal(2), 'o', 'Color', 'Green','LineWidth',5);
text((goal(1)+10),goal(2),'\leftarrow GOAL','FontSize',9,'Color','BLUE');

for i = 1:samples
    while(1)
        points = [floor(rand(1)*map_dim2) floor(rand(1)*map_dim1)]; % generates random x and y co-ordinates that fit within the map
        x = points(1);
        y= points(2);
        if x == 0 % if values of x and y are equal to 0 which is not possible to plot on the map it will shift the point to (1,1)
            x = [floor(rand(1))+1];
        end
        if y == 0
            y = [floor(rand(1))+1];
        end
        if (map(y,x)==0) % does not plot point for black spaces
            continue
        else
            break
        end
    end

    if map(y,x) == 255 % plots point for white spaces
    plot(points(1),points(2), 'o', 'Color', 'Red','LineWidth',0.1);
    hold on
    end

    node_sz = size(node);
    success = 0;
    for j = 1:node_sz(1)
        complete = distance(node(j, 1:2), goal); % checks if the node is within the goal co-ordinates and its set radius
        if complete <= radius
            success = 1;
            break
        end
    end

    if success == 1; % breaks and completes algorithm if goal node is reached
        break
    end

    dist = [];
    for j = 1:node_sz(1);
        n = node(j,:);
        d = distance(n,points); % measures distance between random points and all nodes in the existing tree
        dist = [dist d]; % picks the closest node from existing tree to branch out from
    end

    [val indx] = min(dist); 
    q_near = node(indx,:); % creates the nearest point between node and random point with the specified maximum distance
    q_new = steer(points,q_near,val,line_distance)
    new_point = [round(q_new) indx];
    
    
    if map(new_point(2),new_point(1)) == 255 % plots line if on white space
        node = [node;new_point];
        line([q_near(1),new_point(1)],[q_near(2), new_point(2)],'Color','Blue','LineWidth',0.5); % plots line between nearest point and previous point
        drawnow
        hold on
    end
end
node
D = [];
for j = 1:node_sz(1)
    t_dist = distance(node(j,1:2),goal);
    D = [D t_dist];
end

[val idx] = min(D); 
q_final = node(idx,:);


goal = [goal idx];
node = [node; goal];
while idx ~= 0 % once goal has been reached finds the minimum distance between start and goal point within the tree
    next = node(idx, :);
    line([goal(1), next(1)], [goal(2), next(2)], 'Color', 'Green', 'LineWidth', 5)
    drawnow
    hold on
    idx = next(3);
    goal = next;
end

% for i = 1:1:Nodes
%     points = [floor(rand(1)*map_dim1) floor(rand(1)*map_dim2)];
%     plot(points(1), points(2), 'x', 'Color',  [0 0.4470 0.7410])
% 
%     for j = 1:1:length(Nodes)
%         if Nodes(j) == goal
%             break
%         end
%     end
% 
%     distance = [];
%     for j = 1:1:length(Nodes)
%         n = Nodes(j);
%         tmp = dist(n, points);
%         distance = [distance tmp];
%     end
%     [val, idx] = min(distance);
%     q_near = Nodes(idx);
%     ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
%     new_points = steer(points, q_near, val, EPS);
%     if noCollision(points, q_near, obstacle)
%         line([q_near(1), new_points(1)], [q_near(2), new_points(2)], 'Color', 'k', 'LineWidth', 2);
%         drawnow
%         hold on
%     end
%    
% 
% D = [];
% for j = 1:1:length(Nodes)
%     tmpdist = dist(Nodes(j), goal);
%     D = [D tmpdist];
% end
% 
% [val, idx] = min(D);
% q_final = Nodes(idx);
% goal.parent = idx;
% q_end = goal;
% Nodes = [Nodes goal];
% while q_end.parent ~= 0
%     start = q_end.parent;
%     line([q_end(1), Nodes(start)(1)], [q_end(2), Nodes(start)(2)], 'Color', 'r', 'LineWidth', 2);
%     hold on
%     q_end = Nodes(start);
% end
% 
% % if map(x,y) == 0
% 
% 
% 
% 
