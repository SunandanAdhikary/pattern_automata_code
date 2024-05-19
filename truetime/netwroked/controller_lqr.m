function [u, xhat] = controller_lqr(data, y, xhat, u, sysid)%ref)

A = data.Pd{sysid}.A;
B = data.Pd{sysid}.B;
C = data.Pd{sysid}.C;
K = -data.C{sysid};
L = data.O{sysid};
% F = data.F;
% estimator
% data.x_hat = A*data.x_hat + B*data.u + L*(y - C*data.x_hat);
xpred = A*xhat+ B*u;
r = y - C*xpred;
xhat = xpred+L*r;
% controller
data.u = - K*xhat ;%- F*ref;


