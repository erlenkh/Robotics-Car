function new_energy = energy_calc(v,energy)
    timestep = 0.01;
    P_0 = 1; % const
    M = 810; %kg
    
    dV = gradient(v(1)); %???
    acc = dV./timestep;
    
    del_E = (M.*acc+ P_0).*v(1).*timestep;
    
    new_energy = energy - del_E;
    
end