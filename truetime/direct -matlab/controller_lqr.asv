function data = controller_lqr(data, y, xhat, u)   %ref)
% disp('control calc')
hidx = data.hidx;
Pd = data.dsysmats{hidx};
A = Pd.A;
B = Pd.B;
C = Pd.C;
K = -data.ctrlgains{hidx};
L = data.estgains{hidx};
% F = data.F;
% estimator
% data.x_hat = A*data.x_hat + B*data.u + L*(y - C*data.x_hat);
xpred = A*xhat + B*u;
r = y - C*xpred;
xhatnew = xpred + L*r;
% controller
unew = K*xhatnew;%- F*ref;
Jnew = [xhatnew;unew]'*data.Q*[xhatnew;unew];
data.samp = data.samp+1;
% data.xhatvec{data.samp}= xhatnew;
% data.uvec{data.samp}= unew;
% data.Jvec(data.samp) = Jnew;
% disp('control calc end')
data = jtExecSys(data,2);
data = jtExecSys(data,3);
data = jtExecSys(data,4);
data.Jvec1(data.samp) = data.J;
% disp('control calc jt end')


