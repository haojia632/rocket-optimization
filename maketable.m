% run this script with: octave maketable.m > atmostable.m

printf("%% Precalculated atmospheric model\n");
printf("%% -------------------------------\n");
%
printf("%% density rho, a, speed of sound in m/s, temperature in K, pressure P in Pascal\n");
%printf("%% density rho, a, temperature T in Kelvin, pressure P in Pascal, nu, altitude z\n");

printf("global atmostable = [");

% LEO (400km altitude) should be enough for everyone ;-)
for altitude = 1:1000:400000
	%{
	rho =  1.2238
	a =  340.26
	T =  288.08
	P =    1.0120e+05
	nu =    1.4619e-05
	z =  10.000
	%}
	[rho,a,T,P,nu,z] = atmos(altitude);
	printf("%05d, %05d, %05d, %05d\n", rho, a, T, P);
end

printf("];");
