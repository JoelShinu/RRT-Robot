clearvars
clear all

map =("maps\map1.png") % selecting the map

start = [50,50]; % start co-ordinates
goal = [1200, 700]; % goal co-ordinates


line_distance = 20; % maximum distance of line from node in tree to connect to new node
samples = 5500; % number of random samples on the map
radius = 50; % the radius from the goal co-ordinates where the program will complete

RRT(map,samples,radius,line_distance,start,goal)