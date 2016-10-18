pkg load odepkg

fvdb = @(vt,vy) [vy(2); (1 - vy(1)^2) * vy(2) - vy(1)];

%vopt = odeset ("RelTol", 1e-3, "AbsTol", 1e-3, "NormControl", "on", "OutputFcn", @odeplot);
%ode45(fvdb, [0 20], [2 0]);

vopt = odeset ("RelTol", 1e-3, "AbsTol", 1e-3, "NormControl", "on");
ode45(fvdb, [0 20], [2 0], vopt);

input("nice!");

