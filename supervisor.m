classdef supervisor
   properties
      xx
      yy
   end
   methods
        
       function sv = supervisor_create_trajectory(sv,x,y,x_cp,y_cp,G,start, via,final)
            % init values
            x_loc= 0;
            y_loc= 0;
            theta_loc = 0;
    
            % get start via and final points
            
            x_start = start(1);
            y_start = start(2);
            x_via = via(1: length(via)/2);
            y_via = via((length(via)/2) +1 :end);
            x_final = final(1);
            y_final = final(2);
            
            %lab2params % run params
            %load('midtest.mat')% get map     %uncomment and run to get x, y, x_cp, y_cp, G
            %=(whole cell map)
            G(22,21)=1; % adding missed path
            
            % find out which cell the points is inside
            
            check_points =[x_start, x_via, x_final;y_start, y_via, y_final];
           
            in_cell = 0;
            for i = 1: length(check_points)
                for j = 1: length(x)
                    % xq yq point xv yv polygon=x y
                    in_cell = inpolygon(check_points(1,i),check_points(2,i),x(j,:),y(j,:));
                    
                    if in_cell == 1
                        ref_route(i) =j;
                        in_cell = 0;
                    end
                end
                
            end
            
            % start, end dijkstra(G,start,end)
            % Insert the start and end cell into dijsktra function below
            
            % ---------Dijsktra----------------
            % Find shortest and "cheapest" path
            % [cost route] = dijkstra(Graph, source, destination) % answer backward
            
            % pass all via points
            up_route = [];
            for k =1:length(ref_route)-1
                [e, via_route] = dijkstra(G,ref_route(k),ref_route(k+1)) ;
                % Flip backward route, transpose vector and add to route
                up_route = [up_route, fliplr(via_route)]; 
            end
            % remove duplicates
            route = up_route(diff([0 up_route])~=0),
            
            %---- Get points/trajectory for chosen route from cell point (x_cp ,y_cp)
            
            traj_x = zeros(length(route),1);
            traj_y = zeros(length(route),1);
            
            % save cellpoints of route to trajectory variable
            for m = 1: length(route)
                traj_x(m) = x_cp(route(m));
                traj_y(m) = y_cp(route(m));
            end
            traj = [traj_x, traj_y];
            
            % To get smoother traj
            
            % 2 7 15 22 ta bort h�rn punkter cellerna
            %traj_xnew = [traj_x(1:2); traj_x(4);traj_x(6:8);traj_x(10:end)];
            %traj_ynew = [traj_y(1:2); traj_y(4);traj_y(6:8);traj_y(10:end)];
            %traj = [traj_xnew,traj_ynew];
            
            % 4 9
            %traj_xnew = [traj_x(1:3);traj_x(5:8);traj_x(10:end)];
            %traj_ynew = [traj_y(1:3);traj_y(5:8);traj_y(10:end)];
            %traj = [traj_xnew,traj_ynew];
            
            % maybe include start and final point as last and first and last point
            % instead of the first and last cell point
            
            npt = length(traj);  % number of via points, (including initial and final)
            nvia = [0:1:npt-1];
            
            cs_x = csapi(nvia,traj_x);
            %cs_x = csapi(nvia,traj_xnew); % new smoother traj
            cs_y = csapi(nvia,traj_y);
            %cs_y = csapi(nvia,traj_ynew); % new smoother traj
            h= 0.01/(2*5);%0.01/2;
            time = [0:h:npt-1];
            sv.xx = fnval(cs_x, time); % x value in every time step (f(x))
            sv.yy = fnval(cs_y, time); % y value
     
            
        end
        
        function [z_ref, distance_to_end] = supervisor_high_level_controller(sv,x,y)
        
        for j = 1:length(sv.xx)
            distances(j) = (x - sv.xx(j))^2 + (y - sv.yy(j))^2 ; %no sqrt for performance
        end
        
        distance_to_end = distances(length(sv.xx));
        [shortest_distance, shortest_distance_index] = min(distances);
        
        %set z_ref to the n-th point after the closest point:
        n = 5;
        index = min(length(sv.xx) -1,shortest_distance_index + n);
        x_ref = sv.xx(index);
        y_ref = sv.yy(index); 
            
        if index+1<=length(sv.xx)
            theta_ref = atan2(sv.yy(index+1)-sv.yy(index), sv.xx(index+1)-sv.xx(index));
        else
            theta_ref = atan2(sv.yy(index)-sv.yy(index-1), sv.xx(index)-sv.xx(index-1));
        end
        
        z_ref = [x_ref, y_ref, theta_ref];
        
        end
   end
end