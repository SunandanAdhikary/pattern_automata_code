% Cruise Control plant
% This function solves LMI to find out the number of maximum
% drops canbe allowed for ensuring aymptotic stability.
% It implements the concept provided in ICCPS-13 paper 
% "Co-design of control and platform with dropped signals"

 %%%%%%%%%%%%%%%% Plant-Controller Design %%%%%%%%%%%%%%%%%%%%

M = 0.4;
m = 0.1;
b = 0.25;
I = 0.03;
g = 9.8;
l = 0.26;

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0]; 

dimension=4;
sampling_period=0.07;

plant = ss(A, B, C, D);
[dim_s,dim_i]=size(B);       

%{
%%%%%%%%%%%% find optimal pole using LQR method %%%%%%%%%%%%

Q_1c_1=10^3*(C'*C);
Q_12c_1=[0;0;0;0];
Q_2c_1=10^-4;
R_1c_1=0.005*(B*B');
R_2_1=0.005;

Q=Q_1c_1;           
R=Q_2c_1; 
%}



%%%%%%%%%%%%%%%%%% controller %%%%%%%%%%%%%%%%%%%%

Ts=sampling_period;
ZOH = c2d(plant,Ts,'zoh');     % State space equation of plant-- discrete time
[Apd, Bpd, Cpd, Dpd] = ssdata(ZOH);

Q=(Cpd')*Cpd;
R=eye(size(B,2));
[K1,S,opt_pole_1] = dlqr(Apd, Bpd, Q, R); % Get Optimal S matrix

A1=[Apd Bpd;-K1 zeros(dim_i,dim_i)]; 
A0=[Apd Bpd;zeros(dim_i,dim_s) eye(dim_i)];

%{
%%%%%%%%%%--- controller design ---%%%%%%%%%%
      
QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
QWU=[R_1c_1 zeros(dimension,1);zeros(1,dimension) R_2_1];
  
lqg_reg=lqg(ZOH,QXU,QWU);

[Acd,Bcd,Ccd,Dcd]=ssdata(lqg_reg);  %  this K1 is same as -Ccd

Ic=eye(dimension);
Z=zeros(dimension,dimension);   

A1=[Apd Bpd*Ccd;Bcd*Cpd Acd];  
A0=[Apd Bpd*Ccd;Z Ic];
%}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% generate pattern of length n having k zeros.

pattern_length=9;
no_of_0s=3;
all_1_pattern='111111111';

all_combination=combnk(1:pattern_length,no_of_0s);
[total_comb,temp]=size(all_combination);
product=eye(dimension+1);
pattern_set=cell(1,total_comb);
  
   %%%%%%%%%%%%%%%%%%%%%%% Generate LMI %%%%%%%%%%%%%%%%%%%%%%%%%%%%

j=no_of_0s;
A0_j=A0^j;

%%========= lmi variables are gamma and P ================%%

setlmis([]);
%gmma=lmivar(1,[1,0]);
P=lmivar(1,[5,1]);

lmiterm([1 1 1 P],-1,1);                  % LMI # 1, element @ (1,1): -P
               
lmiterm([1 1 2 -P],A0_j',1);              % LMI # 1, element @ (1,2) : (P*A0_j)' = (A0_j)' P'

lmiterm([1 2 1 P],1,A0_j);                % LMI # 1, element @ (2,1) : P*A0_j

lmiterm([1 2 2 P],-1,1);                  % LMI # 1, element @ (2,2): -P  

lmiterm([-2 1 1 P],1,1);                  % LMI # 2  P       for P > 0

%lmiterm([2 1 1 0],1);                    % LMI # 2: I       for P > I

%%========= generate lmi for each pattern  ================%%

lmi_id=2;

for i=1:total_comb
    pattern=all_1_pattern;
    for j=1:no_of_0s
        pattern(all_combination(i,j))='0';
    end
    lmi_id=lmi_id+1;
    product=eye(dimension+1);
    for k=pattern_length:-1:1
    	if pattern(k)=='1'
			product=A1*product;
        else
			product=A0*product;
		end
    end
    
    lmiterm([lmi_id 1 1 P],product',product);   % LMI # lmi_id, element @ (1,1): product' P product
               
	lmiterm([lmi_id 1 2 P],-1,1);               % LMI # lmi_id, element @ (1,2) : -P
    
end            

lmisys = getlmis;                         % Create the LMI system
[tmin,rP_feas] = feasp(lmisys) ;          % Solve LMI

P = dec2mat(lmisys,rP_feas,P)             % display P matrix
eig(P)
% r = dec2mat(lmisys,rP_feas,r)           % display r

    
% plot(decay,rate,'r'),xlabel('Decay Rate'),ylabel('Min. Exe. Rate');grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



