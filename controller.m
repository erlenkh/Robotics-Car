function controls = controller(z,z_ref,theta_error_prev)
    
    x = z(1);          y = z(2);          theta = z(3);
    x_ref = z_ref(1);  y_ref = z_ref(2);  theta_ref = z_ref(3);
    
    W_e = [x_ref - x; y_ref - y; theta_ref - theta];
    
    rot_mat = [cos(theta) sin(theta) 0;
               -sin(theta) cos(theta) 0;
               0               0      1;];
          
    b_e = rot_mat*W_e;
    
    %keep theta_error between - pi and pi: 
    
    if b_e(3) > pi
        b_e(3) = b_e(3) - 2*pi;
    end 
    
    if b_e(3) < - pi
        b_e(3) = b_e(3) + 2*pi;
    end 
          
    % error terms:
    x_error = b_e(1);
    y_error = b_e(2);
    theta_error = b_e(3);
    theta_error_der = (theta_error - theta_error_prev)/0.01; %timestep = 0.01
    
    %Controll parameters:
    
    scaling = 1;
    K_v = 7 *scaling;   
    K_s = 13 *scaling;
    K_l = 1 * scaling;
    K_sd = 4 * scaling;

    V = K_v * x_error;
          
    V = min(max(V,0.5),30);    
    
    omega_st = K_s * b_e(3) + K_l * y_error + K_sd*theta_error_der;
    
    omega_st_no_shrink = omega_st;
      
    omega_st = pi/4*tanh(0.05*omega_st); %shrinking omega_st
    
    controls = [V, omega_st,transpose(b_e), omega_st_no_shrink, theta_error_der];
    
end


