% Example differential equation solver for a paper ball without air resistance

function dr = dr_gravi(r,t)
	g  = 9.81; %m/s^2

	vx = r(3);
	vy = r(4);
	ax = 0;
	ay = -g;

	dr = [vx;vy;ax;ay]
endfunction

function dr = dr_gravi_friction(r,t,area,mass)
	g = 9.81; %m / s^2
	cdSphere = 0.45;
	rhoAir = 1.20; %kg / m^3
	frictioncoefficient = 1/2 * rhoAir * cdSphere * area / mass;  

	vx = r(3);
	vy = r(4);
	ax = - frictioncoefficient * vx^2;          % only friction

	ay = -( g + frictioncoefficient * vy^2 ) ;  % friction and  
	%  gravitation
	dr = [vx,vy,ax,ay];
endfunction

% Initial point
X0 = 0
Y0 = 0
VX0 = 2
VY0 = 3
initialVector = [ X0; Y0; VX0; VY0]

StartT= 0 %s
StopT = 0.7 %s

mass1 = 0.1 %kg
mass2 = 0.001 %kg
area1 = pi * 0.02^2   %m^2
area2 = area1;

t = linspace(0, 0.7)

[y,max_t] = lsode( @dr_gravi_friction, initialVector, t, [], area1, mass1)
figure(1)
plot(t,y)

figure(2)
plot(y(:,1),y(:,2),'o');
xlabel 'x position / m'
ylabel 'y position / m'

figure(3)
plot(t, y(:,4));
xlabel 'time / s'
ylabel 'y velocity / (m /s)'

input("Done, press enter\n");
exit

