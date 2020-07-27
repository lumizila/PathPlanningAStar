% The E* algorithm in Octave for classes Perspectives in informatics 3,
% June 2020, by Marija Seder
% use this template for your code and translate the drawing functions specific
% for Octave: they will not be executed if started with flag showMode=0
% from the main script Script_obstacles.m
% To see each step expansion of nodes updateDraw has pause after plot
% If you do not want to see each step expansion comment updateDraw in 3 places

classdef EStar < handle
properties
    map = []; % Map: 0 - free space, 1 - obstacles
    open = []; q=[]; G=[]; closed = []; start = []; goal = []; act = []; path = []; nearobst = 1;
    showMode = 0;
    
    allNodes=0;
    
    % environment
    x_min = 0;x_max = 10;y_min = 0;y_max = 10; resolution=100; free; obst;
     
    pathCost=nan;
end
    
methods
    function path = find(obj, start, goal, nearobst) 
% start=[x; y], goal=[x; y] nearobst is the cost of the cell next to the occupied cell (all free cells have the cost equal 1)
        obj.start =start;   obj.goal = goal; obj.path = [];
        obj.nearobst = nearobst;           
        

        obj.closed = []; % Empty closed list
        obj.open = [];
        obj.q = [];
        obj.G=[];

        
        % % Initial open list, node structure
        obj.open = struct('id', 1, ... % starting node id==1
                          'pose',goal,...
                          'srcId', 0, ... % parent id id==0
                          'srcPose',nan,...
                          'cth', Inf, ...  % cost to here
                          'rhs', 0, ...  % estimation of the cost to here
                          'key', 0, ...  % min(rhs,cth) for sort
                          'srcId1', 0, ...  % id of the best parent
                          'srcId2', 0); % id of the second best parent
        obj.allNodes=1;
        obj.updateDraw(goal,goal,1);
                        
        if obj.isPointInFreeSpace(obj.start)==0 || obj.isPointInFreeSpace(obj.goal)==0
            path = []; 
            disp('Path not feasible, goal or start not in free space!');
            return; % Path not feasible!
        end

        dx= 1/obj.resolution; dy= 1/obj.resolution;  % increment = unit/resolution 

        % 8 neigbour coordinates around the goal
        nxx=obj.goal(1)+[dx;dx;0;-dx;-dx;-dx;0;dx]; nyy=obj.goal(2)+[0;dy;dy;dy;0;-dy;-dy;-dy];   
        obj.act = obj.open(1);     
        for j=1:length(nxx)  % add neighbours
             if obj.addNodeToOpenListG(nxx(j), nyy(j)) 
                  obj.updateDraw([nxx(j);nyy(j)],[obj.act.pose],2);  % draw the last node (new added node) 
             end
        end
        
        [~,i] = sortrows([obj.open.key].', [1]);   
        obj.open = obj.open(i);

        
        while true % Search loop
  
            if isempty(obj.open)
                disp('empty open')
                obj.get_path();
                break;
            end
            
            obj.act = obj.open(1); % Get node from the ordered open list,
            obj.closed = [obj.closed, obj.act]; % add it to the closed list
            obj.open = obj.open(2:end); % and remove it from the open list.
            
            if (obj.act.cth>obj.act.rhs)
                obj.closed(end).cth=obj.act.rhs;
                obj.closed(end).key=obj.act.rhs;
            end

              
              % extend neigbours
            dx= 1/obj.resolution; dy= 1/obj.resolution;  % increment = unit/resolution
            nxx=obj.act.pose(1)+[dx;0;-dx;0]; nyy=obj.act.pose(2)+[0;dy;0;-dy];   % 4 neigbour coordinates
            for j=1:length(nxx)  %compute propagator
                if obj.updateVertex(nxx(j),nyy(j))
                    obj.updateDraw([nxx(j);nyy(j)],[obj.act.pose],2);  % draw the last node (new added node) 
                    [~,i] = sortrows([obj.open.key].', [1]);   
                    obj.open = obj.open(i);
                end
            end
        end
        path = obj.path;
        disp(obj.pathCost);
    end

     
    
    
    function added=addNodeToOpenListG(obj,xF, yF)
    
        if obj.isPointInFreeSpace([xF;yF])==0 , added=0; return , end; % node not possible

            xS=obj.act.pose(1);    
            yS=obj.act.pose(2);% parent node        
            deltaX = xF-xS; deltaY = yF-yS;
            pose=[xF;yF];
            F=1; 
            if obj.isPointCloseToObstacle([xF;yF]) && obj.nearobst>0
                F=1/obj.nearobst;
            end
            rhs= obj.act.rhs + (sqrt(deltaX^2+deltaY^2))/F;
            cth=Inf;

            pp=[obj.open.pose]; s=[];
            if ~isempty(pp)
                z=abs(pp(1,:)-xF)<0.0001 & abs(pp(2,:)-yF)<0.0001;
                s= find(z, 1); %returns at most the first index corresponding to the nonzero entries of the array z
            end   
            if isempty(find(s, 1)) % Add new node to the open list

                   % add new node to open list
                  node = struct('id', obj.allNodes+1, ... % id
                              'pose',pose,...
                              'srcId', obj.act.id, ... % parent id
                              'srcPose',obj.act.pose,...
                              'cth', cth, ...  % cost to here
                              'rhs', rhs, ...  % estimation of the cost to here
                              'key', min(rhs,cth), ...  % min(rhs,cth) for sort
                              'srcId1', obj.act.id, ...  % id best parent
                              'srcId2', obj.act.id); % id second best parent
                  obj.allNodes=obj.allNodes+1;
                  obj.open = [obj.open, node];
                  obj.G=[obj.G, node];
                  added=1; % node was added     
            end   
        
    end
 
    function added=updateVertex(obj,xF, yF)
    
        if obj.isPointInFreeSpace([xF;yF])==0 , added=0; return , end; % node not possible
        ppo=[obj.open.pose];
        ppG=[obj.G.pose];
        if ~isempty(ppG)
            z=abs(ppG(1,:)-xF)<0.0001 & abs(ppG(2,:)-yF)<0.0001;
            if ~isempty(find(z, 1)), added=0; return , end;  %is the pose in the G list?
        end
        
        pp=[obj.closed.pose];
        if ~isempty(pp)
            z=abs(pp(1,:)-xF)<0.0001 & abs(pp(2,:)-yF)<0.0001;
            if ~isempty(find(z, 1)), added=0; return , end; 
        end
        
        pose=[xF;yF];

        dx= 1/obj.resolution; dy= 1/obj.resolution;
        h=dx;
        F=1; 
        if obj.isPointCloseToObstacle([xF;yF]) && obj.nearobst>0
            F=1/obj.nearobst;
        end
        nxx=xF+[dx;0;-dx;0]; nyy=yF+[0;dy;0;-dy];   % 4 neigbour coordinates
        
        s=[];
        obj.q=[];
        for j=1:length(nxx)
            if obj.isPointInFreeSpace([nxx(j);nyy(j)])
                 
                if ~isempty(pp)
                    z=abs(pp(1,:)-nxx(j))<0.0001 & abs(pp(2,:)-nyy(j))<0.0001;
                    s= find(z, 1);
                    if ~isempty(find(s, 1))
                        obj.q = [obj.q, obj.closed(s)];
                        [~,i] = sortrows([obj.q.cth].', [1]);   
                        obj.q = obj.q(i);
                    end
                end
                
                    
            end
            
        end
        if isempty(obj.q)
            z=pp(1,:)==obj.act.pose(1) & pp(2,:)==obj.act.pose(2);
            s= find(z, 1);
            for j=1:length(nxx)
                if obj.isPointInFreeSpace([nxx(j);nyy(j)])
                                 
                    if ~isempty(pp)
                        z=abs(pp(1,:)-nxx(j))<0.0001 & abs(pp(2,:)-nyy(j))<0.0001;
                        s= find(z, 1);
                        if ~isempty(find(s, 1))
                            obj.closed(s)
                            obj.closed(s).pose
                        end
                    end
                
                    
                end
            
            end
        else
            T1 = obj.q(1); % Get node from the ordered q list
            T2=T1;%init
            obj.q = obj.q(2:end);
            tmppose=T1.pose;
            if F==1 && obj.isPointCloseToObstacle([tmppose(1);tmppose(2)]) && obj.nearobst>0
                F=1/obj.nearobst;
            end

            if isempty(obj.q)
                rhs=T1.cth+h/F;
            else
                T2 = obj.q(1);
                tmppose=T2.pose;
                if F==1 && obj.isPointCloseToObstacle([tmppose(1);tmppose(2)]) && obj.nearobst>0
                    F=1/obj.nearobst;
                end
                beta=-(T1.cth+T2.cth);
                gama=0.5*(T1.cth^2+T2.cth^2-h^2/F^2);
                rhs=0.5*(-beta+sqrt(beta^2-(4*gama))); 

            end
        end
        cth=Inf;
        s=[];
        if ~isempty(ppo)
            z=abs(ppo(1,:)-xF)<0.0001 & abs(ppo(2,:)-yF)<0.0001;
            s= find(z, 1);
        end   
        
        if isempty(find(s, 1)) % Add new node to the open list
           
               % add new node to open list
              node = struct('id', obj.allNodes+1, ... % id
                          'pose',pose,...
                          'srcId', obj.act.id, ... % parent id
                          'srcPose',obj.act.pose,...
                          'cth', cth, ...  % cost to here
                          'rhs', rhs, ...  % estimation of the cost to here
                          'key', min(rhs,cth), ...  % min(rhs,cth) for sort
                          'srcId1', T1.id, ...  % id best parent
                          'srcId2', T2.id); % id second best parent
              obj.allNodes=obj.allNodes+1;
              obj.open = [obj.open, node];
              added=1; % node was added           
         else % Update the node in the open list 
                obj.open(s).rhs = rhs;
                obj.open(s).key = min(rhs,obj.open(s).cth);
                obj.open(s).srcId = obj.act.id;
                obj.open(s).srcPose = obj.act.pose;
                obj.open(s).srcId1 = T1.id;
                obj.open(s).srcId2 = T2.id;
                added=1; % was updated 
         end   
        
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
    

    
    
    
    function updateDraw(obj,node,srcPose,nodeid)
        if obj.showMode==0, return; end

        figure(10),hold on
                               
        if(nodeid~=1)                    

            r= [srcPose(1:2,1),node(1:2,1)];

            if obj.isPointCloseToObstacle([node(1); node(2)])
                plot(node(1), node(2),'ko'); hold on
            end
            plot(node(1), node(2),'rx', r(1,:),r(2,:),'c'); hold on %path sections  and nodes
        else
            plot(node(1), node(2),'ko'); hold on
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

    function ok = isPointCloseToObstacle(obj, point)
            if point(1)>obj.x_max || point(1)<obj.x_min || point(2)>obj.y_max || point(2)<obj.y_min
                ok = 0;
                return;
            end
            px =  (point(1)-obj.x_min)/(obj.x_max-obj.x_min)*(size(obj.map,1)-1)+1;
            py =  (point(2)-obj.y_min)/(obj.y_max-obj.y_min)*(size(obj.map,2)-1)+1;
            
            xofs=[-1 1 1 -1 0 0 1 -1];
            yofs=[-1 -1 1 1 1 -1 0 0];
            for d=1:length(xofs)
            if obj.map(round(px)+xofs(d),round(py)+yofs(d))==obj.free 
                ok = 0;
            else
                ok = 1;
                break;
            end
            end
                   
    end
   
     
 
    function rr=drawFinalPath(obj)
        p=obj.path; rr=[];
        ids=[ obj.closed.id];
        pp=[ obj.closed.pose];
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
            
            
        end
        
        stepsize=dd/2;
        xxc=obj.start(1);
        yyc=obj.start(2);
        rr=[xxc yyc];

        ind = (ids==p(end));
        node=obj.closed(ind);

        
        while 1
        
              posint=obj.get_posint(node,xxc,yyc,stepsize);
              xxc=posint(1);
              yyc=posint(2);
              if node.srcId==0
                rr=[rr;xxc,yyc]; % store points of the optimal path
                break;
              end
              posegrid=obj.mapToGrid([xxc; yyc]);
              z=abs(pp(1,:)-posegrid(1))<0.0001 & abs(pp(2,:)-posegrid(2))<0.0001;
              if isempty(find(z, 1))
                break;
              end
              s= find(z, 1);
              node=obj.closed(s);  

              rr=[rr;xxc,yyc]; % store points of the optimal path
        end
        
        plot(rr(:,1), rr(:,2),'r')
    end
  
    
    function posint=get_posint(obj,node,xxc,yyc,stepsize)
    
          ids=[ obj.closed.id];
            %current node pose node.pose
        %current node cth
          if node.srcId==0
            posint=node.pose';
            return;
          end
          T=node.cth;
          %best node B1
          indb=(ids==node.srcId1);
          B1=obj.closed(indb);
          %best node B2
          indb=(ids==node.srcId2);
          B2=obj.closed(indb);

          if(B1.pose(2)==node.pose(2))
            TA=B1.cth;
            TC=B2.cth;
            dx=(node.pose(1)-B1.pose(1))*(TA-T);
            dy=(node.pose(2)-B2.pose(2))*(TC-T);
          else
            TA=B2.cth;
            TC=B1.cth;
            dx=(node.pose(1)-B2.pose(1))*(TA-T);
            dy=(node.pose(2)-B1.pose(2))*(TC-T);
          end
          alfa=stepsize/sqrt(dx^2+dy^2);
          dx=dx*alfa;
          dy=dy*alfa;

          xxc=xxc+dx;
          yyc=yyc+dy;
          posint=[xxc yyc];
    
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
