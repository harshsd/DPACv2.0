syms y(t);
ode = diff(y,t) == -160*2*3.14*y+0.6*sin(100*2*3.14*t);
%ode = diff(y,t,2) == diff(y,t,1)*(1-y*y)-y;
ySol(t) = dsolve(ode);
pretty(ySol)