% Graphical User Interface
function [start,via,final] = gui(x,y,theta)
% input: (x,y theta)

% output: start, via, final points
%          x_start, y_start
%          x_points, y_points
%          x_final, y_final

clear x_gui y_gui
clc
%Roadmap    % load map and events

% ---------------------Params--------------------------
%plot map with given axis
map = imread('ist_gmaps.png'); 
%calculate new origin
[rows, columns, numberOfColorChannels] = size(map);
origin = [712, 234];
x_org = origin(1);
y_org = origin(2);
xdata = -x_org : columns - x_org;
ydata = -y_org : columns - y_org;
 %-----------------------------------------------------
theta= 0;
load("events_final.mat")
figure(1)
title('Map over IST')
image(map,'XData', xdata, 'YData', ydata) %map with origin fixed axis
%xlabel('x'); 
%ylabel('y');
%axis on; 
%grid on ; 
hold on;
ev = plot(x_event,y_event, '+b')
legend(ev,'Events')


%----------------------Enter POINTS--------------------------
% Should be runned ones

k = 1; % counting points
button = 1 ; % 1 for left, 2 for middle, 3 for right, other numbers for keypresses.

%disp('Draw cells on Figure 1')
msg_start = 'Use mouse to enter start, via and final points';
msg_start2 = 'Exit by pressing right mouse button ';
uiwait(msgbox({msg_start;' ';msg_start2}, 'Instruction message','modal'));

% Plot point as long as clicking left mouse button
while (button == 1)
    % click start end and intermediate points
    [x_gui(k), y_gui(k), button] = ginput(1);
    if(button == 1) % Don't plot when clicking "exit" button
        plot(x_gui,y_gui,'*r')
    end
    k= k+1;
end
legend(ev,'Events')
% remove last point pressed when exiting
x_points = x_gui(1:end-1);
y_points = y_gui(1:end-1);

% start point
x_start = x_points(1);
y_start = y_points(1);

% final point
x_final = x_points(end);
y_final = y_points(end);

% via point
x_via = x_points(2:end-1);
y_via = y_points(2:end-1);


% output: [start, via ,final]
start = [x_start, y_start];
via   = [x_via, y_via];
final = [x_final, y_final];


end


%---------------------- Draw location of car------------------------

% RUNNED IN TRAJECTORY

% plot(xx,yy,'r-',traj_x,traj_y,'b*')
% set ( gca, 'ydir', 'reverse' )
% % draw the car at every cell     %%10 points of the trajectory
% tp = length(xx);
% sp = round(tp / 10);
% for k=1:sp:tp
%     if k+1<=tp
%         theta_car = atan2(yy(k+1)-yy(k), xx(k+1)-xx(k));
%     else
%         theta_car = atan2(yy(k)-yy(k-1), xx(k)-xx(k-1));
%     end
%     for p=1:4
%         rot_car_polygon(p,:) = ([cos(theta_car), -sin(theta_car); sin(theta_car), cos(theta_car)]*car_polygon(p,:)')';
%     end
%     plot(rot_car_polygon(:,1)/x_scale+xx(k), rot_car_polygon(:,2)/y_scale+yy(k))
% end



% % plot car following traj on map
% figure(2)
% image(map,'XData', xdata, 'YData', ydata)
% hold on
% plot(xx,yy)
% plot(z(1,:),z(2,:));
% xlabel('x')
% ylabel('y')
% set ( gca, 'ydir', 'reverse' ) %yaxis
% title('position on map')
% 


