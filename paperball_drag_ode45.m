% Example differential equation solver for a paper ball without air resistance

pkg load odepkg

function dr = dr_gravi(t,r)
	g  = 9.81; %m/s^2

	vx = r(3);
	vy = r(4);
	ax = 0;
	ay = -g;

	dr = [vx;vy;ax;ay]
endfunction

% Take a vector and derive it
% Input: posx, posy, velx, vely
% Output: velx, vely, accellx, accelly
function dr = dr_gravi_friction(t,r,area,mass)
	g = 9.81; %m / s^2
	cdSphere = 0.45;
	rhoAir = 1.20; %kg / m^3
	frictioncoefficient = 1/2 * rhoAir * cdSphere * area / mass;  

	vx = r(3);
	vy = r(4);
	ax = - frictioncoefficient * vx^2;          % only friction
	ay = -( g + frictioncoefficient * vy^2 ) ;  % friction and gravitation

	dr = [vx,vy,ax,ay];
endfunction

% Initial point
X0 = 0
Y0 = 0
VX0 = 2
VY0 = 3
initialVector = [ X0; Y0; VX0; VY0]

mass1 = 0.1 %kg
area1 = pi * 0.02^2   %m^2

StartT= 0 %s
StopT = 0.7 %s

%t = linspace(0, 0.7)

options = odeset( 'RelTol',1e-4, 'AbsTol',1e-4, 'InitialStep',StopT/1e3, 'MaxStep',StopT/1e3)

% trajectory of heavy sphere
% Returns: an array of the times and an array of the results (position, velocity)
% Note: to know the accellerations, we need to run dr_gravi_friction() on one of the solutions
[T,Result] = ode45(@dr_gravi_friction,[StartT, StopT], initialVector , options, area1, mass1)

subplot(3,3,1)
% figure(1)
plot(Result(:,1),Result(:,2),'o');
xlabel 'x position / m'
ylabel 'y position / m'
title({'Trajectory'})

% figure(2)
subplot(3,3,2)
plot(T, Result(:,3),'.', T, Result(:,4),'x');
xlabel 'time / s'
ylabel 'velocity / (m /s)'
legend('vx','vy')
title({'Velocities'})

input("Done, press enter\n");
exit

