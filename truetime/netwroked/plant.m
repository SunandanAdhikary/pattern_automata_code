function [y,xnew,Jk] = plant(data, x, u, sysid)

% odefun = data.odefun{i};
dt =0.01;
h = data.Pd{sysid}.Ts;
process = data.P{sysid};
process.C = eye(size(process.A,1));
process.D = zeros(size(process.A,1),size(process.B,2));
tspan = ttCurrentTime:dt:ttCurrentTime+h;
uspan = u.*ones(size(tspan));
xspan = lsim(process, uspan, tspan, x);
xnew = xspan(end,:)';
Jk= [xnew;u]'*data.Q{i}*[xnew;u];
y = process.C*xnew;


