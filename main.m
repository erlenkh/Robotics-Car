
% ********************* RUN THIS FILE ***********************
clear 
clc
close all

lab2params %load params

load('work_map.mat')% get map
load('events_final.mat') % x_event y_event, eventpoints
%% generate trajectory

% initially in origin

%[xx, yy] = supervisor_create_trajectory(0,0,0,x_c,y_c,x_cp,y_cp,G);
[xx, yy] = trajectory_gen(x_c,y_c,x_cp,y_cp,G);
%[start,via, final]= gui(0,0,0); 

%% simulation

theta_start = atan2(yy(2)-yy(1), xx(2)-xx(1));
z(:,1) = [xx(1); yy(1); theta_start; 0]; 

energy = 10000;
stop = 0;
stop_count1 = 0;
limit_speed_count2 = 0;
stop_count3 = 0;
stop_count4 = 0;
limit_speed = 0;
speed_limit = 15; 
ped_walk_simtime = 5e4;
event_radius = 10;
event_finished = zeros(1,length(x_event));

distance_to_end = 999;
i = 0;
while (distance_to_end > event_radius)
    i = i + 1;
    x = z(1,i);
    y = z(2,i);
    
     if i >50*length(xx) %car has clearly failed...
        disp('car has failed')
        break
    end
    
    % ---- supervisor: handle events ---- 
    act_on_event = 0; 
     
    for j= 1:length(x_event)
        dist(j) =sqrt((x-x_event(j))^2 + (y-y_event(j))^2);
        
        if dist(j) < event_radius && event_finished(j) == 0 % close to event
            act_on_event = eventpoints(j);            
        end
              
        if dist(j) < event_radius && eventpoints(j) == 4 && i == ped_walk_simtime == 5e4;
            act_on_event = 4;
            
        end
        
        if dist(j) > event_radius && event_finished(j) == 1 % not close to event
            event_finished(j) = 0;
        end
        
    end
    
    switch act_on_event
        case 1
            stop_count1 = stop_count1 + 1;
            if stop_count1 > 500
                stop = 0;
                event_finished(find(eventpoints==1)) = 1;
                disp(['Event 1 finished: "Stop traffic sign" at timestep: ',num2str(i)])
                stop_count1 =0;
            else
                stop = 1;
  
            end
        case 2
            limit_speed = 1;
           % disp(['Event 2: "Speed limit traffic sign" at time: ',num2str(i)])
        case 3
            stop_count3 = stop_count3 + 1;
            if stop_count3 > 600
                stop = 0;
                event_finished(find(eventpoints==3)) = 1;
                disp(['Event 3 finished: "Pedestrian crossing traffic sign" at timestep: ',num2str(i)])
                stop_count3 = 0;
            else
                stop = 1;
            end
        case 4 
            stop_count4 = stop_count4 + 1;
            if stop_count4 > 600
                stop = 0;
                event_finished(find(eventpoints==4)) = 1;
                disp(['Event 4 finished: "Pedestrian crossing" at timestep: ',num2str(i)])
                stop_count4 = 0;
            else
                stop = 1;
            end   
        otherwise
           
    end
    
    if limit_speed == 1
        limit_speed_count2 = limit_speed_count2 + 1;
            if limit_speed_count2 > 6000
                limit_speed = 0;
                event_finished(find(eventpoints==2)) = 1;
                disp(['Event finished 2: "Speed limit traffic sign" at timestep: ',num2str(i)])
                limit_speed_count2 = 0;
            end
    end
      
   % --- supervisor: find z_ref: ----
    
    for j = 1:length(xx)
        distances(j) = (x - xx(j))^2 + (y - yy(j))^2 ; %no sqrt for performance
    end
    
    distance_to_end = distances(length(xx));
    [shortest_distance, shortest_distance_index] = min(distances);
    
    %set z_ref to the n-th point after the closest point:
    n = 20;
    index = min(length(xx) -1,shortest_distance_index + n);
    x_ref = xx(index);
    y_ref = yy(index); 
        
    if index+1<=length(xx)
        theta_ref = atan2(yy(index+1)-yy(index), xx(index+1)-xx(index));
    else
        theta_ref = atan2(yy(index)-yy(index-1), xx(index)-xx(index-1));
    end
    z_ref(:,i) = [x_ref, y_ref, theta_ref];
    
    
    % ---- supervisor: run controller with z_ref ----
    if i > 2
        error_theta_prev = error_theta(i-1);
    else
        error_theta_prev = 0;
    end
     
    controls = controller(z(:,i),z_ref(:,i), error_theta_prev);
        
    v(i) = controls(1);
    omega_st(i) = controls(2);
    
    if limit_speed == 1
        v(i) = min(speed_limit, v(i)); 
    end
    
    if stop == 1
        v(i) = 0;
        omega_st(i) = 0;
    end
    
    % ---- supervisor: calculate energy ----
    energy = energy_calc(v(i),energy);
    
    if mod(i,100) == 0
        fprintf('Energy left at timestep %d: %d \n', i, energy);
    end 
    
    if energy < 0
        disp('no energy left')
        break
    end
    
    % ---- run simulation of robot -----
    
    z(:,i+1) = robot_sim(z(:,i),v(i),omega_st(i),timestep);
     
     %------extract error signals: ---------
    error_x(i) = controls(3);
    error_y(i) = controls(4);
    error_theta(i) = controls(5);
    omega_st_bs(i) = controls(6);
    error_theta_der(i) = controls(7);
end
disp('Simulation done!')


%% plotting


figure(1)
tiledlayout(6,1)
nexttile;
plot(z(4,:))
title('phi(t), steering wheel angle')
xlabel('time')
ylabel('phi(t)')

nexttile;
plot(omega_st)
title('omega_st(t), control input')
xlabel('time')
ylabel('omega_st(t)')
% 
% nexttile;
% plot(omega_st_bs)
% title('omega_st_bs(t), control input before shrinking')
% xlabel('time')
% ylabel('omega_st_bs(t)')

nexttile;
plot(v)
title('v(t), control input')
xlabel('time')
ylabel('v(t)')


% nexttile;
% plot(error_x)
% title('error_x(t), ERROR')
% xlabel('time')
% ylabel('error_x(t)')

nexttile;
plot(z_ref(3,:))
title('theta_ref(t), orientation')
xlabel('time')
ylabel('theta_ref(t)')

nexttile;
plot(z(3,:))
title('theta(t), orientation')
xlabel('time')
ylabel('theta(t)')
% 
nexttile;
plot(error_theta)
title('error_theta(t), orientation ERROR')
xlabel('time')
ylabel('error_theta(t)')


% ---------- plot car following trajectory on map
figure(4)
image(map,'XData', xdata, 'YData', ydata)
hold on
plot(xx,yy)
plot(z(1,:),z(2,:));
xlabel('x [pixel]')
ylabel('y [pixel]')
set ( gca, 'ydir', 'reverse' ) %yaxis
title('position on map')


% draw the car at every cell     %%10 points of the trajectory
tp = length(z);
sp = round(tp / 10*1/2); % stepsize 
for k=1:sp:tp
%     if k+1<=tp
% %       theta_car = atan2(yy(k+1)-yy(k), xx(k+1)-xx(k));
%         theta_car = atan2(z(2,k+1)-z(2,k), z(1,k+1)-z(1,k));
%     else
% %       theta_car = atan2(yy(k)-yy(k-1), xx(k)-xx(k-1));
%         theta_car = atan2(z(2,k)-z(2,k-1), z(1,k)-z(1,k-1));
%     end
    theta_car = z(3,k);
    for p=1:4
        rot_car_polygon(p,:) = ([cos(theta_car), -sin(theta_car); sin(theta_car), cos(theta_car)]*car_polygon(p,:)')';
    end
    plot(rot_car_polygon(:,1)/x_scale+z(1,k), rot_car_polygon(:,2)/y_scale+z(2,k))
end

hold off

