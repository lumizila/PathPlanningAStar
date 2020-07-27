% This is the template script in Octave for classes Perspectives in
% informatics 3, June 2020, by Marija Seder
% The main script for starting A* or E* graph search algorithm
% Use the flag flagAstar=1 to use A* algorithm, or flagAstar=0 to use E*
% This script defines the environment size, obstacles, resolution, start
% and goal:
% For example space=[0,10,0,10] defines the environment boundaries as xmin,xmax,ymin,ymax
% obstacles are also defined as xmin,xmax,ymin,ymax row vectors separated
% by column mark ;
% resolution=2 means there are two cells per meter
% There are 7 map examples which are commented: empty, U-shape, narrow passage, symmetric, S map, S map 2 and
% random map
% Current parameters set use U-shaped map, A* and drawing each expansion with pause (you need to press any key to continue)

flagAstar=1;  %1 Astar, =0 Estar 

%close all Octave figures
close all


nearobst=1; %cost of the cell near the obstacle, needs to be larger than 1
% nearobst=2 means the propagation speed is F=1/2
% this is implemented only in E*

% % % % % % 
% space=[0,10,0,10]; obstacles=[]; resolution=2;  % empty
% start=[1;5]; goal=[8;8];

space=[0,10,0,10]; obstacles=[3,3.5,3,6; 6,6.5,3,6; 3,6.5,6,6.5]; resolution=2;  % U-shape
start=[5;4]; goal=[4;8];

% space=[0,10,0,10]; obstacles=[ 4,7,5.5,8; 3,10,4.5,5]; resolution=2; %narrow passage
% start=[5;4]; goal=[8;7];

% space=[0,10,0,10]; obstacles=[3,7,4.5,5]; resolution=2;  % symmetric
% start=[1.25;4.75]; goal=[8;4.75];

% space=[0,10,0,10]; obstacles=[ 0,6,6,7; 4,10,3,4]; resolution=2;  % S map
% start=[7;1.5]; goal=[3;8.5];

% space=[0,10,0,10]; obstacles=[ 0,5.5,5.5,6.; 4,10,3.5,4; 4,4.5,2,4; 9.5,10,1.5,3; 4,4.5,7.5,8.5; 4.5,8,7.5,8; 7.5,8,8,8.5  ]; 
% start=[7;1.5]; goal=[5.11;8.71]; resolution=2; % S map 2



% % random environment
% resolution=2;
% NOvir=80;
% DIM=20; DIMobst=DIM*.05;
% space=[0,DIM,0,DIM]; 
% obstacles=zeros(NOvir,4);
% 
% sizeX=randi(DIMobst*4, NOvir,1)*0.5;
% sizeY=randi(DIMobst*4, NOvir,1)*0.5;
% 
% startX=randi(DIM*resolution-1, NOvir,1)*1/resolution;
% startY=randi(DIM*resolution-1, NOvir,1)*1/resolution;
% 
% obstacles(:,1) = startX;
% obstacles(:,2) = startX+sizeX;
% obstacles(:,3) = startY;
% obstacles(:,4) = startY+sizeY;
% 
% obstacles(obstacles>DIM)=DIM;
% obstacles(obstacles<0)=0;
% 
% start=[0.75;0.75];
% goal=[16;18];
% 
% % use save command to save obstacles to workspace and reuse it with load
%% save obst1 obstacles
%  load obst1



if flagAstar==0
    astar = EStar(); %use E*
else
    astar = AStar(); %use A*
endif

astar.showMode = 1;  %used for drawing in Octave: 1 draws, 0 not

%creates the grid map according to the bounding box, resolution and
%ostacles
astar.environment(space,resolution, obstacles );   


%use discretized start and goal (indices in grid map)
start=astar.mapToGrid(start)
goal=astar.mapToGrid(goal)

tic
if flagAstar
    astar.find(start, goal); % search the path by A*
else
    astar.find(start, goal, nearobst); % search the path by E*
endif
toc

if astar.showMode~=0
    pathPoints=astar.drawFinalPath(); % draw path
endif