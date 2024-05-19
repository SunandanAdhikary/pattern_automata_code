function data = controller_lqr(data, y, ref)

A = data.A_d;
B = data.B_d;
C = data.C_d;
L = data.L;
K = data.K;
F = data.F;
% estimator
data.x_hat = A*data.x_hat+B*data.u+L*(y-C*data.x_hat);
% controller
data.u = - K*data.x_hat - F*ref;
% contol cost


