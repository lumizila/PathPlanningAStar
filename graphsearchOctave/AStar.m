% The A* algorithm in Octave for classes Perspectives in informatics 3,
% June 2020, by Marija Seder
% use this template for your code and translate the drawing functions specific
% for Octave: they will not be executed if started with flag showMode=0
% from the main script Script_obstacles.m
% To see each step expansion of nodes updateDraw has pause after plot
% If you do not want to see each step expansion comment updateDraw in 2 places

classdef AStar < handle
properties
    map = []; % Map: 0 - free space, 1 - obstacles
    open = []; closed = []; start = []; goal = []; act = []; path = [];
    showMode = 0;
    allNodes=0;
    % environment
    x_min = 0;x_max = 10;y_min = 0;y_max = 10; resolution=100; free; obst;
     
    pathCost=nan;
    
end
    
methods
    function path = find(obj, start, goal) % start=[x; y], goal=[x; y]
        
        obj.start =start;   obj.goal = goal; obj.path = [];

        obj.closed = []; % Empty closed list
        obj.open = [];
        
        % % Initial open list, node structure
        obj.open = struct('id', 1, ... % starting node id==1
                          'pose',goal,...
                          'srcId', 0, ... % parent id id==0
                          'srcPose',nan,...
                          'cth', 0, ...  % cost to here
                          'ctg', obj.heuristic(start)); % cost to final (start) node (heuristic)
        obj.allNodes=1;
        obj.updateDraw(goal,goal,1); %draw the first node
        
                        
        if obj.isPointInFreeSpace(obj.start)==0 || obj.isPointInFreeSpace(obj.goal)==0
             path = []; 
             disp('Path not feasible, goal or start not in free space!');
             return; % Path not feasible!
        end
        

        
        while true % Search loop
  
            if isempty(obj.open)
                
                disp('empty open')
                obj.get_path();
                break; 
            
            end 
            
            obj.act = obj.open(1); % Get node from the ordered open list,
            obj.closed = [obj.closed, obj.act]; % add it to the closed list
            obj.open = obj.open(2:end); % and remove it from the open list.

                
              % chech if we are sufficiently close to the final node (start if searched from goal)
%             obj.pathCost=nan;
%             dist=sqrt (sum( (obj.act.pose(1:2) - obj.start(1:2)).^2 ));
% 
%             if abs(dist)<0.001  
%                 obj.pathCost=obj.act.cth+obj.act.ctg;
%                 p = obj.act.id; obj.path = [p]; ids = [obj.closed.id];
%                 while (p~=1) % Follow src nodes to the start
%                     p = obj.closed(ids==p).srcId;
%                     obj.path = [p, obj.path];
%                 end
%                 break;
% 
%             end

              
              % extend neigbours
             dx= 1/obj.resolution; dy= 1/obj.resolution;  % increment = unit/resolution 
              % 4 neigbour coordinates
             nxx=obj.act.pose(1)+[dx;0;-dx;0]; nyy=obj.act.pose(2)+[0;dy;0;-dy];  
             % 8 neigbour coordinates
%              nxx=obj.act.pose(1)+[dx;dx;0;-dx;-dx;-dx;0;dx]; nyy=obj.act.pose(2)+[0;dy;dy;dy;0;-dy;-dy;-dy];   
             
             for j=1:length(nxx)  % add neighbours
                   if obj.addNodeToOpenListG(nxx(j), nyy(j)) 
                        obj.updateDraw([nxx(j);nyy(j)],[obj.act.pose],2);  % draw the last node (new added node) 
                        % Sort open list
                        [~,i] = sortrows([[obj.open.cth]*1+[obj.open.ctg]*1; obj.open.ctg].', [1,2]);   
                        obj.open = obj.open(i);
                   end
             end
              
        end
        path = obj.path;
        disp(obj.pathCost);
    end

    
    
 
    function get_path(obj)

        pp=[obj.closed.pose];
        if ~isempty(pp)
            z=abs(pp(1,:)-obj.start(1))<0.0001 & abs(pp(2,:)-obj.start(2))<0.0001;
            if isempty(find(z, 1)), return , end;
            s= find(z, 1);
            a=obj.closed(s);
            obj.pathCost=a.cth;

            p = a.id; obj.path = [p]; ids = [obj.closed.id];
            while (p~=1) % Follow src nodes to the goal
                p = obj.closed(ids==p).srcId;
                obj.path = [p, obj.path];
            end
             
        end                
    end  
    
    
    
    function added=addNodeToOpenListG(obj,xF, yF)
    
        if obj.isPointInFreeSpace([xF;yF])==0 , added=0; return , end; % node not possible

        pp=[obj.closed.pose];
        z=pp(1,:)==xF & pp(2,:)==yF; 
        if ~isempty(find(z, 1)), added=0; return , end;  %is the pose already in the closed list?
        

        xS=obj.act.pose(1);    yS=obj.act.pose(2);% starting pose         
        deltaX = xF-xS; deltaY = yF-yS;
        pose=[xF;yF];
        cth= obj.act.cth + (sqrt(deltaX^2+deltaY^2));
        ctg= obj.heuristic(pose);
        
        
        pp=[obj.open.pose]; s=[];
        if ~isempty(pp)
            z= pp(1,:)==xF & pp(2,:)==yF;
            s= find(z, 1);
        end   
        
        if isempty(find(s, 1)) % Add new node to the open list
           
               % add new node to open list
              node = struct('id', obj.allNodes+1, ... % id
                          'pose',pose,...
                          'srcId', obj.act.id, ... % parent id
                          'srcPose',obj.act.pose,...
                          'cth', cth, ...  % cost to here 
                          'ctg', ctg); % cost to final (start) node (heuristic)
              obj.allNodes=obj.allNodes+1;
              obj.open = [obj.open, node];
              added=1; % node was added           
         else % Update the node in the open list if it has better score
            if cth<obj.open(s).cth
                obj.open(s).cth = cth;
                obj.open(s).srcId = obj.act.id;
                obj.open(s).srcPose = obj.act.pose;
                added=1; % was updated 
            end
            added=0;  
        end   
        
    end
 
    
    
    function h = heuristic(obj, a)
             h = sqrt(sum((a(1:2)-obj.start(1:2)).^2)); % Euclidean distance
%              h = 0; %no heuristics
    end

    
    
    
    function updateDraw(obj,node,srcPose,nodeid)
        if obj.showMode==0, return; end

        figure(10),hold on
                               
                if(nodeid~=1)                    
                    
                    r= [srcPose(1:2,1),node(1:2,1)];
                   
                    plot(node(1), node(2),'rx', r(1,:),r(2,:),'c'); hold on %path sections  and nodes
                else
                    plot(node(1), node(2),'rx'); hold on
                end
         pause
    end
    
    
   
    function environment(obj,limits,res, Rect)
        obj.x_min = limits(1);obj.x_max = limits(2);obj.y_min = limits(3); obj.y_max = limits(4);           
        obj.resolution = res; % res=100 -> 100 cells per meter
        y_delta = obj.y_max - obj.y_min;
        x_delta = obj.x_max - obj.x_min;
        obj.free = 0; obj.obst = 1;
        obj.map = ones(ceil(y_delta*res),ceil(x_delta*res))*obj.free;
        obj.map(1,:)=obj.obst; obj.map(end,:)=obj.obst; obj.map(:,1)=obj.obst; obj.map(:,end)=obj.obst;



        for i=1:size(Rect,1)
             obj.map( ceil(Rect(i,1)*res+1):ceil(Rect(i,2)*res),...
                      ceil(Rect(i,3)*res+1):ceil(Rect(i,4)*res) ) = obj.obst; 

        end

        if obj.showMode~=0 % draw the map
            figure(10),hold on

            Nx=size(obj.map,1);
            Ny=size(obj.map,2);
        
            for g=1:Nx
                for h=1:Ny 
                
                
                    dd= 1/obj.resolution/2; %increment
                    gx=   1/obj.resolution*g-dd;
                    gy=   1/obj.resolution*h-dd;


                    nx=gx+[-dd,dd,dd,-dd, -dd]; ny=gy+[-dd,-dd,dd,dd,-dd];   % cell vertices

                    %draw obstacles
                    if(obj.isPointInFreeSpace([gx;gy]) )
                        line(nx,ny) 
                    else
                        patch(nx,ny,[0.7 0.7 0.7]); 
                    end
                 
        
                end
            end

        end      
            
            
    end

   
    function ok = isPointInFreeSpace(obj, point)
            if point(1)>obj.x_max || point(1)<obj.x_min || point(2)>obj.y_max || point(2)<obj.y_min
                ok = 0;
                return;
            end
            px =  (point(1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1)+1;
            py =  (point(2)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1)+1;
            
            if obj.map(round(px),round(py))==obj.free 
                ok = 1;
            else
                ok = 0;
            end
                   
    end

     
 
    function rr=drawFinalPath(obj)
        p=obj.path; rr=[];
        ids=[ obj.closed.id];
        figure(10);hold on
        
        
        for i=1:length(p)
            ind = (ids==p(i));
            node=obj.closed(ind);
            
            if(p(i)>1)
                r= [node.srcPose(1:2,1),node.pose(1:2,1)];
                plot(r(1,:),r(2,:),'b--','LineWidth',2 );
            end
            
            % draw cells of the path           
            xx=node.pose(1,1); yy=node.pose(2,1);            
            
            dd= 1/obj.resolution/2; %increment
            nx=xx+[-dd,dd,dd,-dd, -dd]; ny=yy+[-dd,-dd,dd,dd,-dd];   % cell vertices
            line(nx,ny);
            
            rr=[rr;xx,yy]; % store points of the optimal path
            
        end
    end
  
     
    function rr=mapToGrid(obj,point)  % just maps to the center of the grid
            px = round( (point(1,1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1*0)+0.5*1);
            py = round( (point(2,1)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1*0)+0.5*1);
            rr=[0;0];
            rr(1)=   1/obj.resolution*px - 1/obj.resolution/2;
            rr(2)=   1/obj.resolution*py - 1/obj.resolution/2;
    end 
     
   
    
    
    
end
end