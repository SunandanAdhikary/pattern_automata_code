% Cruise Control plant
% This function solves LMI to find out the number of maximum
% drops canbe allowed for ensuring aymptotic stability.
% It implements the concept provided in ICCPS-13 paper 
% "Co-design of control and platform with dropped signals"

 %%%%%%%%%%%%%%%% Plant-Controller Design %%%%%%%%%%%%%%%%%%%%

M = 0.4;
m = 0.1;
b = 0.26;
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
sampling_period=0.1;

plant = ss(A, B, C, D);
[dim_s,dim_i]=size(B);       

%%%%%%%%%%%%%%%%%% controller %%%%%%%%%%%%%%%%%%%%

Ts=sampling_period;
ZOH = c2d(plant,Ts,'zoh');     % State space equation of plant-- discrete time
[Apd, Bpd, Cpd, Dpd] = ssdata(ZOH);

Q=(Cpd')*Cpd;
R=eye(size(B,2));

[K1,S,opt_pole_1] = dlqr(Apd, Bpd, Q, R); 

A1=[Apd Bpd;-K1 zeros(dim_i,dim_i)]; 
A0=[Apd Bpd;zeros(dim_i,dim_s) eye(dim_i)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pattern_length=9;
epsilon=0.4926;

%{
d_max=0.35;            % maximum disturbance
desired_norm=0.005;

ST=5.5;         % settling time
q=6;

L=ST/sampling_period;
l=floor(L/q) % 9 

epsilon=(desired_norm/d_max)^(1/q) %.4926
%}

max_0s=3;
gamma=25;

% gamma=log(1/epsilon)/(pattern_length*sampling_period)  % 0.7867
% gamma=-2*log(1/epsilon)/(pattern_length*sampling_period)  % -2*0.7867=-1.5735;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%%========= lmi variables is P ================%%

setlmis([]);

% gamma=lmivar(1,[1,0]);
P=lmivar(1,[5,1]);

lmi_id=1;

%%%%%%%%%%%%%%%%%%%%%%% Generate LMI %%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=0:max_0s
    
	A0_j=A0^j;
    	
    lmiterm([lmi_id 1 1 P],-gamma,1);              % LMI # 1, element @ (1,1): -P
		       
	lmiterm([lmi_id 1 2 -P],A0_j',1);              % LMI # 1, element @ (1,2) : (P*A0_j)' = (A0_j)' P'

	lmiterm([lmi_id 2 1 P],1,A0_j);                % LMI # 1, element @ (2,1) : P*A0_j

	lmiterm([lmi_id 2 2 P],-1,1);                  % LMI # 1, element @ (2,2): -P  
    
    lmi_id=lmi_id+1;
end %for

lmiterm([-lmi_id 1 1 P],1,1);                  % LMI # 2  P       for P > 0

% lmi_id=lmi_id+1;
% lmiterm([-lmi_id 1 1 gamma],1,1);                  % LMI #        for gamma > 0

lmisys = getlmis;                         % Create the LMI system
[tmin,rP_feas] = feasp(lmisys) ;          % Solve LMI

P = dec2mat(lmisys,rP_feas,P);             % display P matrix
eig_P=eig(P)

% gamma = dec2mat(lmisys,rP_feas,gamma)           % display gamma
%}

%
%%========= Find Q generating lmi for each pattern for each drop count ================%%


all_1_pattern_large='111111111111111111111111111111111111111111111111111111111111';

for no_of_0=1:max_0s
    
    no_of_1=0;
    no_of_0
    Q=eye(dimension+1);
    eig_Q = eig(Q);
    Q_is_nd = all(eig_Q < 0);

    while(Q_is_nd ==0)
        no_of_1=no_of_1+1;
        pat_length=no_of_1+no_of_0;

        % generate pattern of length n having k zeros
        all_combination=combnk(1:pat_length,no_of_0);
        [total_comb,temp]=size(all_combination);
        product=eye(dimension+1);
        pattern_set=cell(1,total_comb);

        %%========= lmi variables is Q ================%%

        %setlmis([]);

        %Q=lmivar(1,[5,1]);

        %lmi_id=1;

        all_1_pattern=all_1_pattern_large(1:pat_length);

        for i=1:total_comb
            pattern=all_1_pattern;
            for k=1:no_of_0
                pattern(all_combination(i,k))='0';
            end
            %lmi_id=lmi_id+1;
            product=eye(dimension+1);
            for k=pat_length:-1:1
                if pattern(k)=='1'
                    product=A1*product;
                else
                    product=A0*product;
                end
            end
            Q=product'*P*product-P;
            eig_Q = eig(Q);
            Q_is_nd = all(eig_Q < 0);
            if Q_is_nd == 0
                break
            end
        end %for i
        
%         lmiterm([lmi_id 1 1 Q],product',product);   % LMI # lmi_id, element @ (1,1): product' P product
%         lmiterm([lmi_id 1 1 Q],product',product);   % LMI # lmi_id, element @ (1,1): product' P product
%         lmiterm([lmi_id 1 2 P],-1,1);              % LMI # lmi_id, element @ (1,2) : -P

     end  %while
 no_of_1 
 Q
end
%}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



