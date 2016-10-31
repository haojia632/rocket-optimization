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

pkg load odepkg

source("atmostable.m")

global Gravity = 9.81;                   	% Gravity (m/s^2) at sea level
global Max_gravity = 15;			% Maximal G-forces during ascent
global Rocket_drag_coefficient = 0.6		% Might be slightly pessimistic

global Properties_per_stage = 2			% Length, diameter

% Barlow's formula: https://en.wikipedia.org/wiki/Barlow%27s_formula
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

  Motor_length = zeros(1, Number_of_stages);
  Motor_outside_diameter = zeros(1, Number_of_stages);
  cost = 0;	% internal cost function

  % Parse and check arguments
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
  Rocket_payload_mass = 2


  % GALCIT 61-C properties
  GALCIT_burn_rate_at_135_bar = 0.04      %	m/s
  GALCIT_density = 1780	                    % kg/m^3
  GALCIT_price = 4	                        % euro/kg

  % Motor properties
  %Motor_burst_chamber_pressure= 20000000	% N/m^2	(1 bar = 100 000 N/m^2)
  Motor_burst_chamber_pressure=18000000		% N/m^2	(1 bar = 100 000 N/m^2)
  %Motor_pressure_chamber_material_tensile_strength=580000000	% N/m^2		(steel or aluminum)
  Motor_pressure_chamber_material_tensile_strength=600000000	% N/m^2		(steel or aluminum)
  %Motor_pressure_chamber_material_density = 2810	% kg/m^3
  Motor_pressure_chamber_material_density = 7850	% kg/m^3
  Motor_pressure_chamber_material_price = 3		% dollar/kg
  Motor_pressure_chamber_safety_factor	= 1.1			% extra pressure strength
  Motor_specific_impulse=186	                    % s		ISP: at 135 bar @ sea level: 186s (170-190)
  Motor_exhaust_velocity=Motor_specific_impulse*Gravity	                % m/s	according to DoD at 137.895146 bar
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

  % Calculate all kinds of properties for all stages, most importantly the mass of the payload (which, for most stages equals the combined mass of the upper stages)
  for stage = Number_of_stages:-1:1

	  % Derived motor properties
	  Motor_wall_thickness(stage) = Calculate_motor_wall_thickness(Motor_outside_diameter(stage), Motor_burst_chamber_pressure, Motor_pressure_chamber_material_tensile_strength, Motor_pressure_chamber_safety_factor);
	  %Motor_inside_diameter(stage) = Motor_outside_diameter(stage) - Motor_wall_thickness(stage)*2 - Motor_wall_thickness(stage)*4  % outside - wall - insulation
	  Motor_inside_diameter(stage) = Motor_outside_diameter(stage) - Motor_wall_thickness(stage)*2   % outside - wall - insulation

	  % burning area of square propellant grain	12.53285812	m^2  
	  %Propellant_grain_square_side_length(stage) = sqrt(Motor_inside_diameter(stage)^2/2);
	  %Propellant_burning_surface_area_per_motor(stage)  = Motor_burning_sides * Motor_length(stage) * Propellant_grain_square_side_length(stage);
	  Propellant_burning_surface_area_per_motor(stage) = (Motor_inside_diameter(stage)/2)^2 * pi()
	  
	  % Propellant consumption a.k.a. propellant burn rate a.k.a m_dot	892.3394983	kg/s
	  Motor_propellant_burn_rate(stage) = Propellant_burning_surface_area_per_motor(stage) * GALCIT_burn_rate_at_135_bar * GALCIT_density;
	  Rocket_propellant_burn_rate(stage) = Motor_propellant_burn_rate(stage) * Number_of_motors;

	  Thrust_per_motor(stage) = Motor_specific_impulse * Motor_propellant_burn_rate(stage) * Gravity;
	  Motor_cylinder_volume(stage) = pi * Motor_length(stage) * (Motor_outside_diameter(stage)^2 - Motor_inside_diameter(stage)^2)/4;
	  Motor_empty_mass(stage) = Motor_cylinder_volume(stage) * Motor_pressure_chamber_material_density * Motor_mass_overhead;

	  % Rocket properties
	  % -----------------
	  Rocket_mass_overhead_factor = 0.2;	% To account for the mass of the fairing, steering mechanism, fins, electronics, recovery ballute
	  %Rocket_mass_overhead = ( Motor_empty_mass * Rocket_mass_overhead_factor ) * Number_of_motors/2	% Divide number of motors by 2 because there is some scale advantage - TODO: enable this
	  Rocket_mass_overhead = Motor_empty_mass(stage) * Rocket_mass_overhead_factor;

	  Rocket_frontal_area_max(stage) = (Motor_outside_diameter(stage)/2)^2*pi * Number_of_motors;		% This is an upper bound, could be lowered by using a "how many motors can you fit" formula
	  
	  % Payload mass
	  if (stage == Number_of_stages)
		 Stage_payload_mass(stage) = Rocket_payload_mass;
          else
		 Stage_payload_mass(stage) = Rocket_mass_at_liftoff(stage+1);
	  end
	  Rocket_empty_mass(stage) = Motor_empty_mass(stage) * Number_of_motors + Rocket_mass_overhead + Stage_payload_mass(stage)

	  % Derived values
	  % --------------
	  %Motor_propellant_grain_volume(stage) = Propellant_grain_square_side_length(stage) * Propellant_grain_square_side_length(stage) * Motor_length(stage);
	  Motor_propellant_grain_volume(stage) = Propellant_burning_surface_area_per_motor(stage) * Motor_length(stage)
	  Motor_propellant_grain_mass(stage) = Motor_propellant_grain_volume(stage) * GALCIT_density
	  Rocket_propellant_mass(stage) = Motor_propellant_grain_mass(stage) * Number_of_motors
	  Rocket_mass_at_liftoff(stage) = Rocket_empty_mass(stage) + Rocket_propellant_mass(stage)

	  %Burn_time(stage) = (Propellant_grain_square_side_length(stage) / GALCIT_burn_rate_at_135_bar) / Motor_burning_sides;
	  Burn_time(stage) = (Motor_length(stage) / GALCIT_burn_rate_at_135_bar)

	  Delta_v(stage) = Motor_exhaust_velocity * log(Rocket_mass_at_liftoff(stage)/Rocket_empty_mass(stage)) - Gravity * (Rocket_propellant_mass(stage)/Rocket_propellant_burn_rate(stage))
	  if (Delta_v(stage) < 0)
		  Delta_v(stage) = 0
	  end

	  %cost += 1/Delta_v(stage)

  end


  % Initial rocket conditions
  % Start at zero
  Rocket_altitude = 0;
  Stage_max_vertical_velocity = 0;
  Stage_max_horizontal_velocity = 0;

  % Simulate the rocket flight, stage by stage
  Total_rocket_cost = 0;
  Rocket_max_velocity = 0;
  for stage = 1:Number_of_stages
	  Stage_parameters = [Motor_length(stage), Motor_outside_diameter(stage), Rocket_frontal_area_max(stage), Rocket_mass_at_liftoff(stage), Rocket_empty_mass(stage), Thrust_per_motor(stage), Rocket_propellant_burn_rate(stage), Burn_time(stage), Rocket_altitude, Stage_max_vertical_velocity, Stage_max_horizontal_velocity];
	  [Stage_max_altitude, Stage_max_accelleration, Stage_max_vertical_velocity, Stage_max_horizontal_velocity, Stage_altitude_at_max_velocity, Stage_time_at_max_velocity] = Simulate_stage(Stage_parameters);

	  % Keep track of altitude and velocity
	  if (stage < Number_of_stages)
		  % Normally we fire the next stage at maximal velocity
		  Rocket_altitude = Stage_altitude_at_max_velocity;
	  else
		  % The last stage coasts all the way to zero velocity
		  Rocket_altitude = Stage_max_altitude;
	  end
	  Rocket_max_velocity = sqrt(Stage_max_vertical_velocity^2 + Stage_max_horizontal_velocity^2)
	  
	  % Monetary cost function
	  % NOTE: this only includes the biggest costs (motors and propellant) and not the rocket cost (recovery, electronics, fairing, fins) but perhaps these will be negible if we reuse each rocket 5 times
	  if (Stage_max_accelleration < Max_gravity*Gravity)
		  Stage_total_cost = Motor_pressure_chamber_material_price * Motor_empty_mass(stage) * Number_of_motors + Rocket_propellant_mass(stage) * GALCIT_price
	  else
		  % Do not allow too much accelleration
		  cost = Inf
		  return;
	  end
	  Total_rocket_cost += Stage_total_cost

	  % Optimize for cost per payload (no matter the velocity or the altitude, so pretty much useless)
	  % Stage_total_cost_per_payload = Stage_total_cost / Stage_payload_mass(stage)

	  Stage_total_cost_per_payload_per_km = 1000 * Stage_total_cost / (Stage_payload_mass(stage) * Stage_max_altitude^3)
	  cost += Stage_total_cost_per_payload_per_km;

	  % Maximize velocity
	  %cost = 1/Rocket_max_velocity;

	  % Maximize horiztonal velocity
	  %cost = 1/Stage_max_horizontal_velocity
  end

endfunction

% Take a vector r and derive it for time t
% Input: posx, posy, velx, vely
% Output: velx, vely, accellx, accelly
function dr = dr_gravi_friction(t,r,Motor_parameters)
	global Gravity
	global Max_gravity
	global Rocket_drag_coefficient

	global atmostable

	%t

	% We assume all input values here make sense
	Motor_length = Motor_parameters(1);		% Note, we are assuming Propellant_grain_length == Motor_length and that is incorrect because we should substract the aft/fore wall thickness
	Motor_outside_diameter = Motor_parameters(2);
	Stage_frontal_area_max = Motor_parameters(3);
	Rocket_mass_at_liftoff = Motor_parameters(4);
	Rocket_empty_mass = Motor_parameters(5);
	Thrust_per_motor = Motor_parameters(6);
	Rocket_propellant_burn_rate = Motor_parameters(7);
	Burn_time = Motor_parameters(8);

	%g = 9.81; %m / s^2
	%cdSphere = 0.45;
	%rhoAir = 1.20; %kg / m^3
	%frictioncoefficient = 1/2 * rhoAir * cdSphere * Stage_frontal_area_max / Rocket_mass_at_liftoff;

	% Easy stuff first
	Px = r(1);
	Py = r(2);
	Vx = r(3);
	Vy = r(4);

    % Drag force calculation
    % TODO: account for drag of all stages, not just the current stage, otherwise we might incorrectly find that narrow lower stages and wide upper stages are good
    % Determine air density using a precalculated and preloaded .m file in the matrix called atmostable
    if (Py <= 0)
	    rho = 1.22;
    else
	    rounded_position_y = floor(Py / 1000)+1;
	    rho = atmostable(rounded_position_y, 1);
    end
    Drag = 0.5*Rocket_drag_coefficient*rho*Stage_frontal_area_max*(Vx^2+Vy^2); % Calculate drag force

     % Determine rocket thrust, mass and angle based on launch phase
    if t < 0                              % Launch phase 1: initial conditions
        Thrust = 0;
        Mass = Rocket_mass_at_liftoff;
     elseif t <= Burn_time            % Launch phase 2: boosting
        Thrust = Thrust_per_motor;                          
        Mass = Rocket_mass_at_liftoff - Rocket_propellant_burn_rate * t;
	Theta = 45 * pi / 180;
    else % if t > Burn_time             % Launch phase 3: coasting
        Thrust = 0;
        Mass = Rocket_empty_mass;
	% When there is no more thrust, the angle is defined by the velocity vector
        Theta = atan2(Vy, Vx);
    end
    %printf('Theta = %0.5f \n', Theta);

    % Sum of forces calculations 
    Fx = Thrust*cos(Theta) - Drag*cos(Theta);
    Fy = Thrust*sin(Theta) - Drag*sin(Theta) - Mass*Gravity;

    % Acceleration calculations
    Ax = Fx/Mass;                       % Net accel in x direction 
    Ay = Fy/Mass;                       % Net accel in y direction

    dr = [Vx, Vy, Ax, Ay, Mass];
endfunction

% Stop when we reach altitude 0
function [value,isterminal,direction] = ode_events(t,y)
	% Locate the time when height passes through zero in a decreasing direction
	% and stop integration.
	value = y(2);	% position y
	isterminal = 1; % stop the integration
	direction = -1; % negative direction
endfunction

% function Simulate_stage(Motor_parameters)
% This function gets called very often so any optimization here would pay off
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

  % Initial state
  X0 = x(1) = 0
  Y0 = y(1) = Motor_parameters(9)                     % Initial altitude (m)
  VX0 = Vx(1) = Motor_parameters(11)                  % Initial horizontal velocity (m/s)
  VY0 = Vy(1) = Motor_parameters(10)                  % Initial vertical velocity (m/s)
  initialStateVector = [ X0; Y0; VX0; VY0; Rocket_mass_at_liftoff]

  printf("Drag coefficient: %0.5f\n", Rocket_drag_coefficient);

  A(1) = 0;			  % Initial accelleration (m/s^2)
  Distance_x(1) = 0;              % Initial horizontal distance travelled (m)
  Distance_y(1) = 0;              % Initial vertical distance travelled (m)
  Distance(1) = 0;                % Initial  distance travelled (m)

  StartT= 0 %s
  %StopT = Burn_time * 7 %s
  StopT = 1000

  % Ignored when using fixed timesteps: 'RelTol',0.001, 'AbsTol',.001
  options = odeset( 'InitialStep', 0.00001, 'MaxStep', .1, 'Events', @ode_events)

  % Solve a set of non–stiff Ordinary Differential Equations or non–stiff differential algebraic equations (non–stiff DAEs) with the well known explicit Runge–Kutta method of order (4,5)
  % Returns: an array of the times and an array of the results (position, velocity)
  % Note: to know the accellerations, we need to run dr_gravi_friction() on one of the solutions
  [t,Result] = ode45(@dr_gravi_friction, [StartT:.1:StopT], initialStateVector , options, Motor_parameters);

  n = length(Result);

  x = Result(:,1);
  y = Result(:,2);
  Vx = Result(:,3);
  Vy = Result(:,4);
  V = sqrt(Vx .^ 2 + Vy .^ 2);

  printf("\nResults of the stage simulation:\n");
  printf("--------------------------\n");
  [Stage_max_velocity, Max_velocity_index] = max(V(1:n));
  [Stage_max_y_velocity, Max_y_velocity_index] = max(Vy(1:n));

  % Return values:
  Stage_max_range = Stage_max_x_distance = max(x(1:n));
  Stage_max_altitude = Stage_max_y_distance = max(y(1:n));
  printf("Max distance x = range = %0.5f\n", Stage_max_x_distance);
  printf("Max distance y = altitude = %0.5f\n", Stage_max_y_distance);
  %printf("Length of distance vector = %0.5f\n", Distance(n));

  Stage_max_vertical_velocity = Vy(Max_y_velocity_index);
  Stage_max_horizontal_velocity = Vx(Max_y_velocity_index);	% This might not always be correct...
  printf("\n");

  Stage_altitude_at_max_velocity = y(Max_y_velocity_index)
  Stage_time_at_max_velocity = Burn_time
  printf("\n");

  % Calculate accelleration for each step
  % TODO: this can be optimized by calling dr_gravi_friction() with t, x, y, Vx, Vy in matrix form
  output = [zeros(1, n)' , zeros(1, n)' , zeros(1, n)' , zeros(1, n)', zeros(1, n)'];
  for step = 1:n
	time = t(step);
	input = [x(step), y(step), Vx(step), Vy(step), 0];
	output(step,:,:,:,:) = dr_gravi_friction(time, input, Motor_parameters);
  end
  Ax = output(:,3);
  Ay = output(:,4);
  A = sqrt(Ax .^ 2 + Ay .^ 2);
  Mass = output(:,5);
  Stage_max_accelleration = max(A(1:n))
  printf("\n");

  % Visualisations and graphs
  % =========================

  % Figure 1
  subplot(4,3,1)
  plot(x(1:n),y(1:n))
  grid on;
  xlabel({'x(n) - range (m)'})
  ylabel({'y(n) - altitude (m)'});
  %axis([0,100,0,100],'equal')
  %axis([-100,100,-100,100],'equal')
  axis('equal')
  title({'Trajectory'})

  % Figure 2
  subplot(4,3,2)
  plot(t(1:n),Vx(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Vx (m/s)'});
  title({'Horizontal Velocity'});

  % Figure 3
  subplot(4,3,3)
  plot(t(1:n),Vy(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Vy (m/s)'});
  title({'Vertical Velocity'});

  subplot(4,3,4)
  plot(t(1:n),A(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Accelleration(m/s^2)'});
  title({'Accelleration'});

  subplot(4,3,5)
  plot(t(1:n),Mass(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Mass (kg)'});
  title({'Rocket Mass'});

  % Figure 4
  %{
  subplot(4,3,4)
  plot(t(1:n),Theta(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Theta (Deg)'});
  title({'Theta'});

  % Figure 5
  subplot(4,3,5)
  plot(Distance(1:n),Theta(1:n));
  grid on;
  xlabel({'Distance (m)'});
  ylabel({'Theta (Deg)'});
  title({'Theta at Launch'});

  % Figure 7
  subplot(4,3,7)
  plot(t(1:n),Thrust(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Thrust (N)'});
  title({'Thrust'});

  % Figure 8
  subplot(4,3,8)
  plot(t(1:n),Drag(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Drag (N)'});
  title({'Drag Force'});

  % Figure 9
  subplot(4,3,9)
  plot(Distance(1:n),Fn(1:n));
  grid on;
  xlabel({'Distance (m)'});
  ylabel({'Normal Force (N)'});
  title({'Normal Force'});

  % Figure 11
  subplot(4,3,11)
  plot(t(1:n),Fx(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Force x (N)'});
  title({'Force x'});

  % Figure 12
  subplot(4,3,12)
  plot(t(1:n),Fy(1:n));
  grid on;
  xlabel({'Time (s)'});
  ylabel({'Force y (N)'});
  title({'Force y'});
  %}


endfunction


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% EXECUTION STARTS HERE %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulate a stage known stage
%{
Motor_parameters = [12.2037, 1.8130, 2.5815, 5.8938e+04, 3.0901e+04, 3.4862e+06, 1974.3, 14.201, 0, 0, 0]	% max. alt. 43km with 10T payload for 175k dollar               % optimized for minimal cost / (kg of payload * km of altitude^2)		% Stage_max_altitude =    43km, Stage_max_vertical_velocity =  949.32, Stage_altitude_at_max_velocity =  6485.3, max accell: 88.857
[Stage_max_altitude, Stage_max_accelleration, Stage_max_vertical_velocity, Stage_max_horizontal_velocity, Stage_altitude_at_max_velocity, Stage_time_at_max_velocity] = Simulate_stage(Motor_parameters);
%}

% Simulate a known rocket
%Rocket_parameters = [0.15, 0.042176]	% 
%Rocket_parameters = [0.15, 0.0393]	% 
Rocket_parameters = [0.15, 0.034954]
Simulate_rocket(Rocket_parameters);
%Rocket_parameters = [12.2037, 1.8130, 2, 0.5]
%Rocket_parameters = [10.6241  ,  8.0608  , 20.5753  ,  1.9338]
%Rocket_parameters = [41.7625   , 1.6597   , 2.9920   , 2.0931 ,  13.0686  ,  1.8163]	% max altitude 178km, cost 746k, max vertical velocity 1704.5
%Simulate_rocket(Rocket_parameters);

input("Simulation finished. Press [RETURN] to exit.");
exit

% The real GA
Number_of_stages = 1
Population_initial_range = [0,0 ; 42,42]
%Population_initial_range = [0,0,0,0 ; 42, 42, 42,42]
%Number_of_stages = 3
%Population_initial_range = [0,0,0,0,0,0 ; 42, 42, 42, 42, 42,42]
%{
	TODO: dynamically fill this array
Population_initial_range = zeros(1, Number_of_stages*2);
for stage = 1:Number_of_stages
	Population_initial_range(stage*2-1) = 0
	Population_initial_range(stage*2) = 42
end
%}
PopulationSize = 50+10^Number_of_stages;		% Large population size because 90% of individuals are inviable with these 3 stages...

EliteCount = round(PopulationSize * 0.15);	% Needs integer
TimeLimit = 60;		% 1 minute, this is really short but it's just for demo purposes
%TimeLimit = 60 * 60 * 8;	% one night
%TimeLimit = 60 * 60 * 1;	% 1 hour
%TimeLimit = 30 * 60;		% 30 minutes
%TimeLimit = 600;		% 10 minutes
Generations = 10000000;		% Keep running until we reach the timelimit

% This lower bound is chosen for initial population but the mutation function makes the chromosomes < 0
% TODO: use a custom mutation function that ensures that chromosomes are always within the [LB,UB] range
LB = zeros(1,Number_of_stages * Properties_per_stage);
options = gaoptimset('TimeLimit', TimeLimit, 'PopInitRange', Population_initial_range, 'PopulationSize', PopulationSize, 'EliteCount', EliteCount, 'Generations', Generations)
[solution, cost_of_solution, exitflag, output, population, scores] = ga(@Simulate_rocket, Number_of_stages * Properties_per_stage, [], [], [], [], LB, [], [], options)

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

