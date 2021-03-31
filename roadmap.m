% Robotics Lab 2 - Roadmap  -----RUN #2 and ONCE to config map-----
lab2params % run params
% ------Add dijkstra folder to the path! --------
% METHOD: Decomposition in exact cells
% clear all
% clc

map = imread('ist_gmaps.png'); % given map

%calculate new origin
[rows, columns, numberOfColorChannels] = size(map);
origin = [712, 234];
x_org = 712;
y_org = 234;
xdata = -x_org : columns - x_org;
ydata = -y_org : columns - y_org;

%plot map with given axis
figure(1)
title('Original map')
image(map,'XData', xdata, 'YData', ydata) %map with origin fixed axis
%xlabel('x'); ylabel('y');
axis on; 
%grid on ; 
hold on;

j = 1; % counting rectangles =  # cells
button = 1 ; % 1 for left, 2 for middle, 3 for right, other numbers for keypresses.

disp('Draw cells on Figure 1')

% Plot cells as long as clicking left mouse button
while (button == 1)
    disp([('enter cell '), num2str(j)]);
    for i=1:4
        % click 4 points for one cell
        [x_c(j,i),y_c(j,i), button(j,i)] = ginput(1);
        disp([('Point '), num2str(i)]);
    end
    disp([('CELL '), num2str(j), 'DONE!']);
    plot([x_c(j,:) x_c(j,1)],[y_c(j,:) y_c(j,1)],'-b')
    %hold on
    j= j+1;
end
% nrcells =j-1;
hold off % Do not plot more on fig

%Saved x_c and y_c points in workspace, run code below to load them again
%load('work_map.mat') %uncomment and run to get x_c and y_c


%%
%Assign center point to each cell

nrpoints = length(x_c);

x_cp = zeros(1,nrpoints);
y_cp = zeros(1,nrpoints);
for k = 1:nrpoints
    x_cp(k) = mean(x_c(k,:));
    y_cp(k) = mean(y_c(k,:));
end

%% plot center points
figure(2) 
image(map,'XData', xdata, 'YData', ydata)
hold on
plot(x_cp, y_cp, 'r+')
set ( gca, 'ydir', 'reverse' ) %change direction of y-axis
hold off

%%
%------------------- set events--------------------------
figure(3)
title('Event map')
image(map,'XData', xdata, 'YData', ydata) %map with origin fixed axis
xlabel('x'); ylabel('y');
axis on; 
%grid on ; 
hold on;
%disp('Draw cells on Figure 1')
%msg_event = 'Use mouse to enter location of events';
%msg_start2 = 'Exit by pressing right mouse button ';
%uiwait(msgbox({msg_start;' ';msg_start2}, 'Instruction message','modal'));

disp('Enter points for events')
button = 1;
k = 1;
% Plot event point as long as clicking left mouse button
while (button == 1)
    [x_e(k), y_e(k), button] = ginput(1);
    if(button == 1) % Don't plot when clicking "exit" button
        plot(x_e,y_e,'+b')
    end
    k= k+1;
end
hold off

% remove last point pressed when exiting
x_event = x_e(1:end-1);
y_event = y_e(1:end-1);

% determine events at each point
eventpoints = [1 3 21 22 3 3];

% E1 Stop traffic sign, at a fixed location.
% E21 START -Speed limit traffic sign, at a fixed location.
% E22 END - Speed limit traffic sign, at a fixed location.
% E3 Pedestrian crossing traffic sign, at a fixed location.
% E4 Pedestrian crossing at a pre-specified location and time; the duration of the crossing is also pre-specified.


%-------------------end EVENT locations--------------------------
%% adjacency matrix

% include cost instead of 1s later when know where traffic signs etc will
% be
% no relation in diagonal
% 
% (from rownr, to colnr)
% full_map
G = zeros(63,63);
G1 = zeros(56,1); %56-57
G2 = eye(56);
G = [G1,G2];
G(4,6) = 1;
G(6,8) = 1;
G(11,17) = 1; G(11,18) = 1; 
G(12,17) = 1; G(12,18) = 1;
G(14,59) = 1; 
G(16,1) = 1;
G(17,1)=1; G(17,16) = 1; G(17,12) =1; G(17,13) = 1;
G(20,22) =1;
G(29,58) =1;
G(30,58) =1;
G(33,35) =1;
G(36,38) =1;
G(40,58) =1; G(40,42)=1;
G(41,58) =1;
G(44,46) =1;
G(48,50) =1;
G(50,52) =1;
G(54,56) =1;
G(55,62) =1;
G(57,46) =1;
G(58,30) =1; G(58,31) =1; G(58,41) = 1; 
G(59,60) =1; G(59,14) =1;
G(60,61) =1; G(60,59) =1;
G(61,60) =1; G(61,62) = 1;
G(62,55) =1; G(62,56) =1; G(62,61)=1;
G(63,63) =0;
 

%%
load('new_events.mat')

figure(8)
title('Roadmap of IST with cells and center points')
image(map,'XData', xdata, 'YData', ydata)
hold on
for j = 1: length(x_c)
    plot([x_c(j,:) x_c(j,1)],[y_c(j,:) y_c(j,1)],'-b');
    plot(x_cp(j),y_cp(j),'*r')
    hold on
end
% for k =1:length(eventpoints)
%     plot(x_event, y_event, 'og')
%     text(x_event,y_event,num2str(eventpoints(k)))
% end

set(gca, 'ydir', 'reverse' ) %yaxis
xlabel('x [pixels]')
ylabel('y [pixels]')

%% CODE above runned once %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Roadmap.m runned, now run trajectory.m file')










