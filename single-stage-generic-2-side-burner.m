% This script is made for running with Octave. Run it with : octave filename.m
% This script is probably 99% MATLAB compatible.
% Author: Tom Van Braeckel

clear, clc      % Clear command window and workspace

global C = 0.6;                                % Drag coefficient
global Rho = 1.2;                              % Air density (kg/m^3) at sea level
global Gravity = 9.81;                         % Gravity (m/s^2) at sea level

function retval = Calculate_motor_wall_thickness(Motor_outside_diameter, Motor_burst_chamber_pressure, Motor_pressure_chamber_material_tensile_strength, Motor_pressure_chamber_safety_factor)
  retval = Motor_burst_chamber_pressure * Motor_outside_diameter/2 * Motor_pressure_chamber_safety_factor;
  retval = retval / Motor_pressure_chamber_material_tensile_strength;
endfunction

function retval = Simulate_stage(Motor_parameters)
  global Gravity

  Motor_length = Motor_parameters(1)
  Motor_outside_diameter = Motor_parameters(2)
  
  % GALCIT 61-C properties
  GALCIT_burn_rate_at_135_bar = 0.04      %	m/s
  GALCIT_density = 1780	                    % kg/m^3
  GALCIT_price = 4	                        % euro/kg

  % Motor properties
  Motor_burst_chamber_pressure= 20000000	% N/m^2	(1 bar = 100 000 N/m^2)
  Motor_pressure_chamber_material_tensile_strength=580000000	% N/m^2		(steel or aluminum)
  Motor_pressure_chamber_material_density = 2810	% kg/m^3		
  Motor_pressure_chamber_safety_factor	= 1.1			% extra pressure strength
  Motor_specific_impulse=180	                    % s		ISP: at 135 bar @ sea level: 186s (170-190)
  Motor_exhaust_velocity= 1798.32	                % m/s	according to DoD at 137.895146 bar	
  Motor_burning_sides=2
  
  % Derived motor properties
  Motor_wall_thickness = Calculate_motor_wall_thickness(Motor_outside_diameter, Motor_burst_chamber_pressure, Motor_pressure_chamber_material_tensile_strength, Motor_pressure_chamber_safety_factor)
  Motor_inside_diameter = Motor_outside_diameter - Motor_wall_thickness*2 - Motor_wall_thickness*4  % outside - wall - insulation

  % burning area of square propellant grain	12.53285812	m^2  
  Propellant_grain_square_side_length = sqrt(Motor_inside_diameter^2/2)
  Propellant_burning_surface_area_per_motor  = Motor_burning_sides * Motor_length * Propellant_grain_square_side_length
  
  % Propellant consumption a.k.a. propellant burn rate a.k.a m_dot	892.3394983	kg/s
  Propellant_burn_rate = Propellant_burning_surface_area_per_motor * GALCIT_burn_rate_at_135_bar * GALCIT_density
  
  Thrust_per_motor = Motor_specific_impulse * Propellant_burn_rate * Gravity
  
  % Parameters
  Delta = 0.1;                    % Time step 
  Memory_Allocation = 30000;      % Maximum number of time steps expected

  % Preallocate memory for arrays
  t = zeros(1, Memory_Allocation);
  Thrust = zeros(1, Memory_Allocation);
  Mass = zeros(1, Memory_Allocation);
  Theta = zeros(1, Memory_Allocation);
  Fn = zeros(1, Memory_Allocation);
  Drag = zeros(1, Memory_Allocation);
  Fx = zeros(1, Memory_Allocation);
  Fy = zeros(1, Memory_Allocation);
  Ax = zeros(1, Memory_Allocation);
  Ay = zeros(1, Memory_Allocation);
  Vx = zeros(1, Memory_Allocation);
  Vy = zeros(1, Memory_Allocation);
  x = zeros(1, Memory_Allocation);
  y = zeros(1, Memory_Allocation);
  Distance_x = zeros(1, Memory_Allocation);
  Distance_y = zeros(1, Memory_Allocation);
  Distance = zeros(1, Memory_Allocation);

  C = 0.4;                                % Drag coefficient
  Rho = 1.2;                              % Air density (kg/m^3)
  A = 4.9*10^-4;                          % Rocket projected area (m^2)
  Gravity = 9.81;                         % Gravity (m/s^2)
  Launch_Rod_Length = 1;                  % Length of launch rod (m)
  Mass_Rocket_With_Motor = 0.01546;       % Mass with motor (kg)
  Mass_Rocket_Without_Motor = 0.0117;     % Mass without motor (kg)

  Theta(1) = 60;                  % Initial angle (deg)
  Vx(1) = 0;                      % Initial horizontal speed (m/s)
  Vy(1) = 0;                      % Initial vertical speed (m/s)
  x(1) = 0;                       % Initial horizontal position (m)
  y(1) = 0.1;                     % Initial vertical position (m)
  Distance_x(1) = 0;              % Initial horizontal distance travelled (m)
  Distance_y(1) = 0;              % Initial vertical distance travelled (m)
  Distance(1) = 0;                % Initial  distance travelled (m)
  Mass(1) = Mass_Rocket_With_Motor;       % Initial rocket mass (kg)

  n = 1;                          % Initial time step

  Burn_time = 5

  while y(n) > 0                  % Run until rocket hits the ground
    n = n+1;                    % Increment time step

    t(n)= (n-1)*Delta;          % Elapsed time                     

    % Determine rocket thrust and mass based on launch phase
    if t(n) <= 0                              % Launch phase 1
        Thrust(n) = 0;
        Mass(n) = Mass_Rocket_With_Motor;
     elseif t(n) > 0 && t(n) < Burn_time            % Launch phase 2: boosting
        Thrust(n) = Thrust_per_motor;                          
        Mass(n) = Mass_Rocket_With_Motor;
    elseif t(n) >= 0.5 && t(n) < 3.5            % Launch phase 3: coasting
        Thrust(n) = 0;
        Mass(n) = Mass_Rocket_With_Motor;
    elseif t(n) >= 3.5                          % Launch phase 4                        
        Thrust(n) = 0;                                         
        Mass(n) = Mass_Rocket_Without_Motor;    % Rocket motor ejects
    end

    % Normal force calculations  
    if Distance(n-1) <= Launch_Rod_Length       % Launch rod normal force
        Fn(n) = Mass(n)*Gravity*cosd(Theta(1));
    else
        Fn(n) = 0;                              % No longer on launch rod
    end
    
    % Drag force calculation
    Drag(n)= 0.5*C*Rho*A*(Vx(n-1)^2+Vy(n-1)^2); % Calculate drag force
    
    % Sum of forces calculations 
    Fx(n)= Thrust(n)*cosd(Theta(n-1))-Drag(n)*cosd(Theta(n-1))...
        -Fn(n)*sind(Theta(n-1));                            % Sum x forces
    Fy(n)= Thrust(n)*sind(Theta(n-1))-(Mass(n)*Gravity)-...
        Drag(n)*sind(Theta(n-1))+Fn(n)*cosd(Theta(n-1));    % Sum y forces
        
    % Acceleration calculations
    Ax(n)= Fx(n)/Mass(n);                       % Net accel in x direction 
    Ay(n)= Fy(n)/Mass(n);                       % Net accel in y direction

    % Velocity calculations
    Vx(n)= Vx(n-1)+Ax(n)*Delta;                 % Velocity in x direction
    Vy(n)= Vy(n-1)+Ay(n)*Delta;                 % Velocity in y direction

    % Position calculations
    x(n)= x(n-1)+Vx(n)*Delta;                   % Position in x direction
    y(n)= y(n-1)+Vy(n)*Delta;                   % Position in y direction
    
    % Distance calculations    
    Distance_x(n) = Distance_x(n-1)+abs(Vx(n)*Delta);      % Distance in x 
    Distance_y(n) = Distance_y(n-1)+abs(Vy(n)*Delta);      % Distance in y 
    Distance(n) = (Distance_x(n)^2+Distance_y(n)^2)^(1/2); % Total distance

    % Rocket angle calculation
    Theta(n)= atand(Vy(n)/Vx(n));      % Angle defined by velocity vector
  end
  
  % The cost is currently the inverse of the max. altitude
  retval = 1/max(y(1:n))

endfunction

% Small test
Motor_parameters = [10, 1]
inverse_altitude = Simulate_stage(Motor_parameters);
max_altitude = 1/inverse_altitude

% The real GA
[x, fval] = ga(@Simulate_stage, 100)
