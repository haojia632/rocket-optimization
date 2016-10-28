% Example differential equation solver for a paper ball without air resistance

function dr = dr_gravi(r,t)
	g  = 9.81; %m/s^2

	vx = r(3);
	vy = r(4);
	ax = 0;
	ay = -g;

	dr = [vx;vy;ax;ay]
endfunction

% Initial point
X0 = 0
Y0 = 0
VX0 = 2
VY0 = 3
initialVector = [ X0; Y0; VX0; VY0]

StartT= 0 %s
StopT = 0.7 %s

t = linspace(0, 0.7)

[y,max_t] = lsode( @dr_gravi, initialVector, t)
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

