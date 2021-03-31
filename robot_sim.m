function [z_next] = robot_sim(z,v,omega_st,timestep)
    
% z = [x,y,theta,phi]

L = 2.2 *0.2; %distance between back wheels and front wheelz

M = [cos(z(3))*cos(z(4))    0; 
     sin(z(3))*cos(z(4))    0;
     tan(z(4))/L             0;
     0                      1;];
 
z_dot = M*[v; omega_st];
 
z_next = z + z_dot*timestep;

% bounding the steering wheel between -22.5 and 22.5 deg: 
z_next(4) = min(max(z_next(4),-pi/8),pi/8);


end