% Lab2 params  -----RUN #1 and ONCE-----
% add parameters that is used
timestep = 0.01;


% params from roadmap.m
map = imread('ist_gmaps.png'); 
%calculate new origin
[rows, columns, numberOfColorChannels] = size(map);
origin = [712, 234];
x_org = origin(1);
y_org = origin(2);
xdata = -x_org : columns - x_org;
ydata = -y_org : columns - y_org;


% Car dimensions /given from prof
car_length = 3.332;
car_width = 1.508;
L = 2.2; %L =distance from back to front wheel
car_length_out_wheelbase = car_length - L; % egdes outside L


% assume that the length out-of-wheelbase is identical at front and back of
% the car
a1 = car_length_out_wheelbase / 2;
d = car_width/2;
%  car polygon constructed ccw
car_polygon = [ -a1, -d; %right back point
                L + a1, -d; %right front point
                L + a1, d; % left front point
                -a1, d ]; % left back point
    % add a row of first row values if we want to close up the car poly, 
    % but then don't know direction of car

%scale of map
x_scale = 0.18107;
%disp(['xx scale factor ', num2str(x_scale), ' meters/pixel']);

y_scale = 0.21394;
%disp(['yy scale factor ', num2str(y_scale), ' meters/pixel']);

disp('Lab2params.m runned!')


