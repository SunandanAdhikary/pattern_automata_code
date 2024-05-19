function data = controller_lqr(data, y, xhat, u)   %ref)
% disp('control calc')
A = data.Pd.A;
B = data.Pd.B;
C = data.Pd.C;
K = -data.C;
L = data.O;
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
data.xhatvec{data.samp}= xhatnew;
data.uvec{data.samp}= unew;
data.Jvec1(data.samp) = Jnew;
% disp('control calc end')
data = jtExecSys(data,2);
data = jtExecSys(data,3);
data = jtExecSys(data,4);
data.Jvec(data.samp) = data.J;
disp('control calc jt end')


