type vanderpoldemo
tspan = [0 160];
y0 = [2; 0];
Mu = 8;
ode = @(t,y) vanderpoldemo(t,y,Mu);
[t,y] = ode45(ode, tspan, y0);

% Plot solution
plot(t,y(:,1))
xlabel('t (in ms)')
ylabel('solution y')
title('van der Pol Equation, \mu = 8    ')