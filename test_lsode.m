## oregonator differential equation
function xdot = f (x, t)

	xdot = zeros (3,1);

	xdot(1) = 77.27 * (x(2) - x(1)*x(2) + x(1) \
		- 8.375e-06*x(1)^2);
	xdot(2) = (x(3) - x(1)*x(2) - x(2)) / 77.27;
	xdot(3) = 0.161*(x(1) - x(3));

endfunction

t = linspace (0, 500, 1000);

%x0 = [ 1, 2, 3 ]
x0 = [ 4; 1.1; 4 ]

y = lsode ("f", x0, t)



%{
function xdot = f (x,t)
	xdot=-exp(-t)*x^2;
endfunction
t=linspace(0,5,50)
y=lsode("f",2,t)
%}

plot(t,y)
input("Done, press enter\n");
