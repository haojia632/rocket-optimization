% This script is made for running with Octave. Run it with : octave filename.m
% This script is probably 99% MATLAB compatible.
% Author: Tom Van Braeckel

% Documentation:
% http://octave.sourceforge.net/ga/function/ga.html
% http://octave.sourceforge.net/ga/function/gaoptimset.html
% http://nl.mathworks.com/help/gads/examples/coding-and-minimizing-a-fitness-function-using-the-genetic-algorithm.html
% http://nl.mathworks.com/help/gads/gaoptimset.html

clear, clc      % Clear command window and workspace

global Gravity = 9.81;                         % Gravity (m/s^2) at sea level

function retval = Calculate_motor_wall_thickness(Motor_outside_diameter, Motor_burst_chamber_pressure, Motor_pressure_chamber_material_tensile_strength, Motor_pressure_chamber_safety_factor)
  retval = Motor_burst_chamber_pressure * Motor_outside_diameter/2 * Motor_pressure_chamber_safety_factor;
  retval = retval / Motor_pressure_chamber_material_tensile_strength;
endfunction

% This function gets called very often so any optimization here pays off
function retval = Simulate_stage(Motor_parameters)
  global Gravity

  Motor_length = Motor_parameters(1)		% Note, we are assuming Propellant_grain_length == Motor_length and that is not completely correct because we should substract the aft/fore wall thickness
  Motor_outside_diameter = Motor_parameters(2)

  % Do not waste time on impossible configurations (it would be better if the ga() respected the upper and lower bounds - TODO: check whether the UB/LB parameters of ga() are indeed broken)
  if (Motor_length <= 0 || Motor_outside_diameter <= 0)
	  retval = Inf
	  return
  end
  
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
  Motor_cylinder_volume = pi * Motor_length * (Motor_outside_diameter^2 - Motor_inside_diameter^2)/4
  Motor_mass_overhead = 1.1;	% To account for the mass of the nozzle, fore side flange - TODO: increase this?
  Motor_empty_mass = Motor_cylinder_volume * Motor_pressure_chamber_material_density * Motor_mass_overhead

  % Rocket properties
  % -----------------
  Number_of_motors = 1
  Rocket_mass_overhead_factor = 0.15	% To account for the mass of the fairing, steering mechanism, fins, electronics, recovery ballute
  Rocket_mass_overhead = ( Motor_empty_mass * Rocket_mass_overhead_factor ) * Number_of_motors/2	% Divide number of motors by 2 because there is some scale advantage
  Rocket_payload_mass = 10000		% 10T of payload
  Rocket_drag_coefficient = 0.6
  Rocket_frontal_area_max = (Motor_outside_diameter/2)^2*pi * Number_of_motors		% This is an upper bound, could be lowered by using a "how many motors can you fit" formula
  Rocket_empty_mass = Motor_empty_mass * Number_of_motors + Rocket_mass_overhead + Rocket_payload_mass

  % Derived values
  % --------------
  Propellant_grain_volume = Propellant_grain_square_side_length * Propellant_grain_square_side_length * Motor_length
  Propellant_grain_mass = Propellant_grain_volume * GALCIT_density
  Rocket_mass_at_liftoff = Rocket_empty_mass + Propellant_grain_mass

  % Parameters
  Delta = 1;                    % Time step - TODO: decrease this for more accuracy and altitude
  Memory_Allocation = 30000;      % Maximum number of time steps expected - TODO: calculate this dynamically

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

  Launch_Rod_Length = 1;                  % Length of launch rod (m)
  Mass_Rocket_With_Motor = 0.01546;       % Mass with motor (kg)

  Theta(1) = 89;                  % Initial angle (deg)
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
        Mass(n) = Rocket_mass_at_liftoff;
     elseif t(n) > 0 && t(n) < Burn_time            % Launch phase 2: boosting
        Thrust(n) = Thrust_per_motor;                          
        Mass(n) = Rocket_mass_at_liftoff;
    elseif t(n) >= 0.5 && t(n) < 3.5            % Launch phase 3: coasting
        Thrust(n) = 0;
        Mass(n) = Rocket_mass_at_liftoff;
    elseif t(n) >= 3.5                          % Launch phase 4                        
        Thrust(n) = 0;                                         
        Mass(n) = Rocket_empty_mass;    % Rocket motor ejects
    end

    % Normal force calculations  
    Fn(n) = 0;                              % Assume no launch rod
    
    % Drag force calculation
    % TODO: load the drag forces from the table used in the spreadsheet to verify we get identical results
    [rho,a,T,P,nu,z] = atmos(y(n-1));	% TODO: verify that this is really slow and speed it up (cache the result or use tropos.m when altitude is low)
    Drag(n)= 0.5*Rocket_drag_coefficient*rho*Rocket_frontal_area_max*(Vx(n-1)^2+Vy(n-1)^2); % Calculate drag force
    
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
    if (Vx(n) == 0)
	Theta(n) = 90;
    else
	Theta(n)= atand(Vy(n)/Vx(n));      % Angle defined by velocity vector
    end

  end
  
  % The cost is currently the inverse of the max. altitude
  Rocket_max_altitude = max(y(1:n))
  retval = 1/Rocket_max_altitude


endfunction

% Small tests
Motor_parameters = [5.2527, 3.2820]	% max altitude: 34021m with 10T payload
Motor_parameters = [17.2204, 2.4422]	% max altitude: 62865m with 10T payload
Motor_parameters = [79.2182, 3.9899]	% max altitude: 82947m with 10T payload
Motor_parameters = [79.2182, 2.4422]	% max altitude: 90701m with 10T payload
inverse_altitude = Simulate_stage(Motor_parameters);
max_altitude = 1/inverse_altitude

% The real GA
Population_initial_range = [0,0 ; 30, 30]
TimeLimit = 120

%options = gaoptimset ('Generations', 30)
%options = gaoptimset ('TimeLimit', 30 * 60)
options = gaoptimset ('TimeLimit', TimeLimit, 'PopInitRange', Population_initial_range)

[x, fval] = ga(@Simulate_stage, 2, [], [], [], [], [], [], [], options)

% Report the results
max_altitude = 1/fval

% TODO: add visualisations and graphs
