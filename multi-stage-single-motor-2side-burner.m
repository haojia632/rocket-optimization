% Genetic algorithm method of finding optimal rocket motor design dimentions
% --------------------------------------------------------------------------
% Made for GNU Octave. This script is probably 99% MATLAB compatible.
% Run it with : octave filename.m
% Author: Tom Van Braeckel

% NOTE: This currently simulates 2-side burners but at higher velocities an end burner might be more beneficial...

% Documentation:
% http://octave.sourceforge.net/ga/function/ga.html
% http://octave.sourceforge.net/ga/function/gaoptimset.html
% http://nl.mathworks.com/help/gads/examples/coding-and-minimizing-a-fitness-function-using-the-genetic-algorithm.html
% http://nl.mathworks.com/help/gads/gaoptimset.html

clear, clc      % Clear command window and workspace

global Gravity = 9.81;                   	% Gravity (m/s^2) at sea level
global Max_gravity = 15;			% Maximal G-forces during ascent
global Rocket_drag_coefficient = 0.6		% Might be slightly pessimistic

global Properties_per_stage = 2			% Length, diameter

function retval = Calculate_motor_wall_thickness(Motor_outside_diameter, Motor_burst_chamber_pressure, Motor_pressure_chamber_material_tensile_strength, Motor_pressure_chamber_safety_factor)
  retval = Motor_burst_chamber_pressure * Motor_outside_diameter/2 * Motor_pressure_chamber_safety_factor;
  retval = retval / Motor_pressure_chamber_material_tensile_strength;
endfunction

function cost = Simulate_rocket(Rocket_parameters)
  global Gravity
  global Max_gravity
  global Rocket_drag_coefficient 

  global Properties_per_stage

  Number_of_stages = length(Rocket_parameters)/Properties_per_stage

  % Parse and check arguments
  Motor_length = zeros(1, Number_of_stages);
  Motor_outside_diameter = zeros(1, Number_of_stages);
  for stage = 1:Number_of_stages
	  Motor_length(stage) = Rocket_parameters(stage*Properties_per_stage-1);
	  Motor_outside_diameter(stage) = Rocket_parameters(stage*Properties_per_stage);

	  if (Motor_length(stage) <= 0 || Motor_outside_diameter(stage) <= 0)
		  cost = Inf
		  return
	  end
  end

  printf("\nStarting simulation of rocket configuration:\n");
  printf("--------------------------------------------\n");
  Rocket_payload_mass = 10000		% 10T of payload
  %Rocket_payload_mass = 1000		% 10T of payload


  % GALCIT 61-C properties
  GALCIT_burn_rate_at_135_bar = 0.04      %	m/s
  GALCIT_density = 1780	                    % kg/m^3
  GALCIT_price = 4	                        % euro/kg

  % Motor properties
  Motor_burst_chamber_pressure= 20000000	% N/m^2	(1 bar = 100 000 N/m^2)
  Motor_pressure_chamber_material_tensile_strength=580000000	% N/m^2		(steel or aluminum)
  Motor_pressure_chamber_material_density = 2810	% kg/m^3
  Motor_pressure_chamber_material_price = 3		% dollar/kg
  Motor_pressure_chamber_safety_factor	= 1.1			% extra pressure strength
  Motor_specific_impulse=180	                    % s		ISP: at 135 bar @ sea level: 186s (170-190)
  Motor_exhaust_velocity= 1798.32	                % m/s	according to DoD at 137.895146 bar	
  Motor_burning_sides=2
  Motor_mass_overhead = 1.1;	% To account for the mass of the nozzle, fore side flange - TODO: increase this? Or calculate it!
  Number_of_motors = 1			% TODO: experiment with multiple motors per stage
 
  % Calculate all static properties of all stages
  Motor_wall_thickness = zeros(1, Number_of_stages);
  Motor_inside_diameter = zeros(1, Number_of_stages);
  Propellant_grain_square_side_length = zeros(1, Number_of_stages);
  Propellant_burning_surface_area_per_motor = zeros(1, Number_of_stages);
  Motor_propellant_burn_rate = zeros(1, Number_of_stages);

  Thrust_per_motor = zeros(1, Number_of_stages);
  Motor_cylinder_volume = zeros(1, Number_of_stages);
  Motor_empty_mass = zeros(1, Number_of_stages);

  Rocket_frontal_area_max = zeros(1, Number_of_stages);
  Rocket_empty_mass = zeros(1, Number_of_stages);
  Rocket_propellant_burn_rate = zeros(1, Number_of_stages);

  Motor_propellant_grain_volume = zeros(1, Number_of_stages);
  Motor_propellant_grain_mass = zeros(1, Number_of_stages);
  Rocket_propellant_mass = zeros(1, Number_of_stages);
  Rocket_mass_at_liftoff = zeros(1, Number_of_stages);

  Burn_time = zeros(1, Number_of_stages);
  Stage_payload_mass = zeros(1, Number_of_stages);


  for stage = Number_of_stages:-1:1

	  % Derived motor properties
	  Motor_wall_thickness(stage) = Calculate_motor_wall_thickness(Motor_outside_diameter(stage), Motor_burst_chamber_pressure, Motor_pressure_chamber_material_tensile_strength, Motor_pressure_chamber_safety_factor);
	  Motor_inside_diameter(stage) = Motor_outside_diameter(stage) - Motor_wall_thickness(stage)*2 - Motor_wall_thickness(stage)*4;  % outside - wall - insulation

	  % burning area of square propellant grain	12.53285812	m^2  
	  Propellant_grain_square_side_length(stage) = sqrt(Motor_inside_diameter(stage)^2/2);
	  Propellant_burning_surface_area_per_motor(stage)  = Motor_burning_sides * Motor_length(stage) * Propellant_grain_square_side_length(stage);
	  
	  % Propellant consumption a.k.a. propellant burn rate a.k.a m_dot	892.3394983	kg/s
	  Motor_propellant_burn_rate(stage) = Propellant_burning_surface_area_per_motor(stage) * GALCIT_burn_rate_at_135_bar * GALCIT_density;
	  Rocket_propellant_burn_rate(stage) = Motor_propellant_burn_rate(stage) * Number_of_motors;

	  Thrust_per_motor(stage) = Motor_specific_impulse * Motor_propellant_burn_rate(stage) * Gravity;
	  Motor_cylinder_volume(stage) = pi * Motor_length(stage) * (Motor_outside_diameter(stage)^2 - Motor_inside_diameter(stage)^2)/4;
	  Motor_empty_mass(stage) = Motor_cylinder_volume(stage) * Motor_pressure_chamber_material_density * Motor_mass_overhead;

	  % Rocket properties
	  % -----------------
	  Rocket_mass_overhead_factor = 0.15;	% To account for the mass of the fairing, steering mechanism, fins, electronics, recovery ballute
	  Rocket_mass_overhead = 0;
	  %Rocket_mass_overhead = ( Motor_empty_mass * Rocket_mass_overhead_factor ) * Number_of_motors/2	% Divide number of motors by 2 because there is some scale advantage - TODO: enable this

	  Rocket_frontal_area_max(stage) = (Motor_outside_diameter(stage)/2)^2*pi * Number_of_motors;		% This is an upper bound, could be lowered by using a "how many motors can you fit" formula
	  
	  % Payload mass
	  if (stage == Number_of_stages)
		 Stage_payload_mass(stage) = Rocket_payload_mass;
          else
		 Stage_payload_mass(stage) = Rocket_mass_at_liftoff(stage+1);
	  end
	  Rocket_empty_mass(stage) = Motor_empty_mass(stage) * Number_of_motors + Rocket_mass_overhead + Stage_payload_mass(stage);

	  % Derived values
	  % --------------
	  Motor_propellant_grain_volume(stage) = Propellant_grain_square_side_length(stage) * Propellant_grain_square_side_length(stage) * Motor_length(stage);
	  Motor_propellant_grain_mass(stage) = Motor_propellant_grain_volume(stage) * GALCIT_density;
	  Rocket_propellant_mass(stage) = Motor_propellant_grain_mass(stage) * Number_of_motors;
	  Rocket_mass_at_liftoff(stage) = Rocket_empty_mass(stage) + Rocket_propellant_mass(stage);

	  Burn_time(stage) = (Propellant_grain_square_side_length(stage) / GALCIT_burn_rate_at_135_bar) / Motor_burning_sides;


  end

  Total_rocket_cost = 0
  cost = 0;	% internal cost function
  Rocket_altitude = 0
  Stage_max_vertical_velocity = 0
  Stage_max_horizontal_velocity = 0
  Rocket_max_velocity = 0
  for stage = 1:Number_of_stages
	  Stage_parameters = [Motor_length(stage), Motor_outside_diameter(stage), Rocket_frontal_area_max(stage), Rocket_mass_at_liftoff(stage), Rocket_empty_mass(stage), Thrust_per_motor(stage), Rocket_propellant_burn_rate(stage), Burn_time(stage), Rocket_altitude, Stage_max_vertical_velocity, Stage_max_horizontal_velocity];
	  [Stage_max_altitude, Stage_max_accelleration, Stage_max_vertical_velocity, Stage_max_horizontal_velocity, Stage_altitude_at_max_velocity, Stage_time_at_max_velocity] = Simulate_stage(Stage_parameters);
	  % Keep track of altitude and velocity
	  if (stage < Number_of_stages)
		  % Normally we fire the next stage at maximal velocity
		  Rocket_altitude = Stage_altitude_at_max_velocity
	  else
		  % The last stage coasts all the way to zero velocity
		  Rocket_altitude = Stage_max_altitude
	  end
	  Rocket_max_velocity = sqrt(Stage_max_vertical_velocity^2 + Stage_max_horizontal_velocity^2);
	  
	  % Monetary cost function
	  % NOTE: this only includes the biggest costs (motors and propellant) and not the rocket cost (recovery, electronics, fairing, fins) but perhaps these will be negible if we reuse each rocket 5 times
	  if (Stage_max_accelleration < Max_gravity*Gravity)
		  Stage_total_cost = Motor_pressure_chamber_material_price * Motor_empty_mass(stage) * Number_of_motors + Rocket_propellant_mass(stage) * GALCIT_price
	  else
		  % Do not allow too much accelleration
		  Stage_total_cost = Inf
		  return;
	  end
	  Total_rocket_cost += Stage_total_cost

	  % Optimize for cost per payload (no matter the velocity or the altitude, so pretty much useless)
	  % Stage_total_cost_per_payload = Stage_total_cost / Stage_payload_mass(stage)

	  Stage_total_cost_per_payload_per_km = 1000 * Stage_total_cost / (Stage_payload_mass(stage) * Stage_max_altitude^3)
	  cost += Stage_total_cost_per_payload_per_km;
  end

endfunction

% This function gets called very often so any optimization here pays off
function [Stage_max_altitude, Stage_max_accelleration, Stage_max_vertical_velocity, Stage_max_horizontal_velocity, Stage_altitude_at_max_velocity, Stage_time_at_max_velocity] = Simulate_stage(Motor_parameters)
  global Gravity
  global Max_gravity
  global Rocket_drag_coefficient 

  printf("\nStarting simulation of stage configuration:\n");
  printf("-------------------------------------------\n");

  % We assume all input values here make sense
  Motor_length = Motor_parameters(1)		% Note, we are assuming Propellant_grain_length == Motor_length and that is not completely correct because we should substract the aft/fore wall thickness
  Motor_outside_diameter = Motor_parameters(2)
  Stage_frontal_area_max = Motor_parameters(3)
  Rocket_mass_at_liftoff = Motor_parameters(4)
  Rocket_empty_mass = Motor_parameters(5)
  Thrust_per_motor = Motor_parameters(6)
  Rocket_propellant_burn_rate = Motor_parameters(7)
  Burn_time = Motor_parameters(8)
  y(1) = Motor_parameters(9)                    % Initial vertical position (m)
  Vy(1) = Motor_parameters(10)                  % Initial vertical speed (m/s)
  Vx(1) = Motor_parameters(11)                  % Initial horizontal speed (m/s)
 

  V(1) = sqrt(Vx(1)^2 + Vy(1)^2); % Initial velocity (m/s)
  Theta(1) = 90;                  % Initial angle (deg)
  Vx(1) = 0;                      % Initial horizontal speed (m/s)
  A(1) = 0;			  % Initial accelleration (m/s^2)
  x(1) = 0;                       % Initial horizontal position (m)
  %Distance_x(1) = 0;              % Initial horizontal distance travelled (m)
  %Distance_y(1) = 0;              % Initial vertical distance travelled (m)
  %Distance(1) = 0;                % Initial  distance travelled (m)
  Mass(1) = Rocket_mass_at_liftoff;       % Initial rocket mass (kg)

  % Parameters
  Delta = 1;                    % Time step - TODO: decrease this for more accuracy and altitude

  n = 1;                          % Initial time step
  % This loop gets called very very often so it sure pays off to optimize it
  while Vy(n) >= 0                  % Run until rocket is slowing down or pointing downwards (at which point the next stage or the recovery mechanism should have been deployed
    n = n+1;                    % Increment time step

    t(n)= (n-1)*Delta;          % Elapsed time

    % Determine rocket thrust and mass based on launch phase
    if t(n) <= 0                              % Launch phase 1
        Thrust(n) = 0;
        Mass(n) = Rocket_mass_at_liftoff;
     elseif t(n) < Burn_time            % Launch phase 2: boosting
        Thrust(n) = Thrust_per_motor;                          
        Mass(n) = Rocket_mass_at_liftoff - Rocket_propellant_burn_rate * t(n);
    elseif t(n) > Burn_time             % Launch phase 3: coasting
        Thrust(n) = 0;
        Mass(n) = Rocket_empty_mass;
    end

    % Normal force calculations  
    Fn(n) = 0;                              % Assume no launch rod
    
    % Drag force calculation
    % TODO: load the drag forces from the table used in the spreadsheet to verify we get identical results
    % TODO: verify that this air density calculation matches other sources
    % TODO: precalculate the surface area and drag of each stage because otherwise we would get narrow lower stages and wide upper stages...
    [rho,a,T,P,nu,z] = atmos(y(n-1));	% TODO: verify that this is really slow and speed it up (cache the result or use tropos.m when altitude is low)
    Drag(n)= 0.5*Rocket_drag_coefficient*rho*Stage_frontal_area_max*(Vx(n-1)^2+Vy(n-1)^2); % Calculate drag force
    
    % Sum of forces calculations 
    Fx(n)= Thrust(n)*cosd(Theta(n-1))-Drag(n)*cosd(Theta(n-1))...
        -Fn(n)*sind(Theta(n-1));                            % Sum x forces
    Fy(n)= Thrust(n)*sind(Theta(n-1))-(Mass(n)*Gravity)-...
        Drag(n)*sind(Theta(n-1))+Fn(n)*cosd(Theta(n-1));    % Sum y forces
        
    % Acceleration calculations
    Ax(n)= Fx(n)/Mass(n);                       % Net accel in x direction 
    Ay(n)= Fy(n)/Mass(n);                       % Net accel in y direction
    A(n) = sqrt(Ax(n)^2 + Ay(n)^2);

    % Velocity calculations
    Vx(n)= Vx(n-1)+Ax(n)*Delta;                 % Velocity in x direction
    Vy(n)= Vy(n-1)+Ay(n)*Delta;                 % Velocity in y direction
    V(n) = sqrt(Vx(n)^2 + Vy(n)^2);

    % Position calculations
    x(n)= x(n-1)+Vx(n)*Delta;                   % Position in x direction
    y(n)= y(n-1)+Vy(n)*Delta;                   % Position in y direction
    
    % Distance calculations    
    %Distance_x(n) = Distance_x(n-1)+abs(Vx(n)*Delta);      % Distance in x 
    %Distance_y(n) = Distance_y(n-1)+abs(Vy(n)*Delta);      % Distance in y 
    %Distance(n) = (Distance_x(n)^2+Distance_y(n)^2)^(1/2); % Total distance

    % Rocket angle calculation
    if (Vx(n) == 0)
	Theta(n) = 90;
    else
	Theta(n)= atand(Vy(n)/Vx(n));      % Angle defined by velocity vector
    end

  end

  printf("\nResults of the simulation:\n");
  printf("--------------------------\n");
  % Return values:
  Stage_max_altitude = max(y(1:n))
  Stage_max_accelleration = max(A(1:n))
  [Stage_max_velocity, Max_velocity_index] = max(V(1:n))
  Stage_max_vertical_velocity = Vy(Max_velocity_index)
  Stage_max_horizontal_velocity = Vx(Max_velocity_index)
  Stage_altitude_at_max_velocity = y(Max_velocity_index)
  Stage_time_at_max_velocity = Max_velocity_index * Delta

endfunction


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% EXECUTION STARTS HERE %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulate a stage known stage
Motor_parameters = [12.2037, 1.8130, 2.5815, 5.8938e+04, 3.0901e+04, 3.4862e+06, 1974.3, 14.201, 0, 0, 0]	% max. alt. 43km with 10T payload for 175k dollar               % optimized for minimal cost / (kg of payload * km of altitude^2)		% Stage_max_altitude =    43km, Stage_max_vertical_velocity =  949.32, Stage_altitude_at_max_velocity =  6485.3, max accell: 88.857
[Stage_max_altitude, Stage_max_accelleration, Stage_max_vertical_velocity, Stage_max_horizontal_velocity, Stage_altitude_at_max_velocity, Stage_time_at_max_velocity] = Simulate_stage(Motor_parameters);

% Simulate a known rocket
%Rocket_parameters = [12.2037, 1.8130, 2, 0.5]
%Rocket_parameters = [10.6241  ,  8.0608  , 20.5753  ,  1.9338]
Rocket_parameters = [41.7625   , 1.6597   , 2.9920   , 2.0931 ,  13.0686  ,  1.8163]	% max altitude 178km, cost 746k, max vertical velocity 1704.5
Simulate_rocket(Rocket_parameters);

% The real GA
Number_of_stages = 3
%Population_initial_range = [0,0 ; 42,42]
%Population_initial_range = [0,0,0,0 ; 42, 42, 42,42]
Population_initial_range = [0,0,0,0,0,0 ; 42, 42, 42, 42, 42,42]
%{
	TODO: dynamically fill this array
Population_initial_range = zeros(1, Number_of_stages*2);
for stage = 1:Number_of_stages
	Population_initial_range(stage*2-1) = 0
	Population_initial_range(stage*2) = 42
end
%}
%PopulationSize = 100;
PopulationSize = 1000;

EliteCount = round(PopulationSize * 0.15);	% Needs integer
%TimeLimit = 60 * 60 * 8;	% one night
%TimeLimit = 30 * 60;		% 30 minutes
%TimeLimit = 600;		% 10 minutes
TimeLimit = 60;		% 1 minute
Generations = 10000;		% Keep running until we reach the timelimit

options = gaoptimset ('TimeLimit', TimeLimit, 'PopInitRange', Population_initial_range, 'PopulationSize', PopulationSize, 'EliteCount', EliteCount, 'Generations', Generations)
[solution, cost_of_solution, exitflag, output, population, scores] = ga(@Simulate_rocket, Number_of_stages * Properties_per_stage, [], [], [], [], [], [], [], options)

% Now recalculate the solution, verify the cost to display it nicely:
printf("\n\n\n");
printf("=============================================\n");
printf("|               SEARCH FINISHED             |\n");
printf("=============================================\n");
printf("\n");
printf("Solution:\n");
printf("---------\n");
solution
printf("\n");
printf("Evaluation report of solution:\n");
printf("------------------------------\n");
cost = Simulate_rocket(solution);
if (cost != cost_of_solution)
	printf("ERROR: cost of solution does not appear to be reproducable!");
end

% TODO: add visualisations and graphs
