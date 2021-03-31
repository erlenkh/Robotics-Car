% input: * gui.m --> start, via, final points
%          start = [x_start, y_start]
%          via = [x_points, y_points]
%          final = [x_final, y_final]

%        * localization --> current localization of car
%          (x_loc,y_loc,theta_loc)

% output: trajectory [xx, yy]

function [xx, yy] = trajectory_gen(x,y,x_cp,y_cp,G)


% maybe in main instead
%lab2params % load lab2params
% load params from roadmap.m

% init values
x_loc= 0;
y_loc= 0;
theta_loc = 0;

[start,via, final]= gui(x_loc,y_loc,theta_loc); % get start via and final points

x_start = start(1);
y_start = start(2);
x_via = via(1: length(via)/2);
y_via = via((length(via)/2) +1 :end);
x_final = final(1);
y_final = final(2);

% find out which cell the points is inside
check_points =[x_start, x_via, x_final;y_start, y_via, y_final];

ref_route = [];
in_cell = 0;

for i = 1: length(check_points)
    for k = 1: length(x)
        % xq yq point xv yv polygon=x y
        in_cell = inpolygon(check_points(1,i),check_points(2,i),x(k,:),y(k,:));
        
        if in_cell == 1
            ref_route(i) =k;
            in_cell = 0;
        end
    end
end

if isempty(ref_route) == 1
    disp('No route was found, select other points.')
    return
end


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
route = up_route(diff([0 up_route])~=0)

%---- Get points/trajectory for chosen route from cell point (x_cp ,y_cp)

traj_x = zeros(length(route),1);
traj_y = zeros(length(route),1);

% save cellpoints of route to trajectory variable
for m = 1: length(route)
    traj_x(m) = x_cp(route(m));
    traj_y(m) = y_cp(route(m));
end
traj = [traj_x, traj_y];

if isempty(traj) == 1
    disp('No route was found, select other points.')
    return
end

npt = length(traj);  % number of via points, (including initial and final)
nvia = [0:1:npt-1];

cs_x = csapi(nvia,traj_x);
cs_y = csapi(nvia,traj_y);
h= 0.01/(2*5);%0.01/2;
time = [0:h:npt-1];
xx = fnval(cs_x, time); % x value in every time step (f(x))
yy = fnval(cs_y, time); % y value


disp('trajectory generated')

end






