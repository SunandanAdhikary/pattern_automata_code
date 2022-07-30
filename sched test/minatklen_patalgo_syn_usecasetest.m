clc;
clear all;
yalmip("clear");
% {trajectory,esp,four_car_platoon,suspension_control,fuel_injection}
system = "trajectory"
vis = 0; % 1 to plot 0 to skip plotting
verbose = 0; % 1 for logs , 0 for running silently
ops = sdpsettings('verbose',verbose);
%% systems
% Four car platoon--from Optimal Control of Sets of Solutions to Formally Guarantee
% Constraints of Disturbed Linear Systems
if system=="four_car_platoon"
% states: position and velocities of 4 vehicles
% inputs: velocities
% output: positions
    Ts =0.1;
    A =[1.10517091807565,0.0110517091807565,0,0,0,0,0,0;0,1.10517091807565,0,0,0,0,0,0;0,0,1.10517091807565,0.0110517091807565,0,0,0,0;0,0,0,1.10517091807565,0,0,0,0;0,0,0,0,1.10517091807565,0.0110517091807565,0,0;0,0,0,0,0,1.10517091807565,0,0;0,0,0,0,0,0,1.10517091807565,0.0110517091807565;0,0,0,0,0,0,0,1.10517091807565];
    B =[5.34617373191714e-05,0,0,0;0.0105170918075648,0,0,0;5.34617373191714e-05,-5.34617373191714e-05,0,0;0.0105170918075648,-0.0105170918075648,0,0;0,5.34617373191714e-05,-5.34617373191714e-05,0;0,0.0105170918075648,-0.0105170918075648,0;0,0,5.34617373191714e-05,-5.34617373191714e-05;0,0,0.0105170918075648,-0.0105170918075648];
    C =[1,0,0,0,0,0,0,0;0,0,1,0,0,0,0,0;0,0,0,0,1,0,0,0;0,0,0,0,0,0,1,0];
    D =zeros(size(C,1),size(B,2));
    actuator_limit = [10;10;10;10];
    sensor_limit = [20;20;20;20];
    safex = [-20,-10,-20,-10,-20,-10,-20,-10;20,10,20,10,20,10,20,10];
    Q = C'*C;
    R = 1;
    [K,S,E]=dlqr(A,B,Q,R);
    sys_d =ss(A-B*K,B,C,D,Ts);
    L = [7.38472324606530e-09	1.10767378478111e-08	-3.69178068993179e-09	-2.35766404514692e-13;...
        -8.06607123878028e-08	-1.20987484468706e-07	4.03243462966293e-08	2.45231890733761e-12;...
        1.10767378620785e-08	2.21532418194634e-08	-1.47682827951273e-08	3.69154491818929e-09;...
        -1.20987484656173e-07	-2.41972543487752e-07	1.61309378729129e-07	-4.03218939403918e-08;...
        -3.69178073241593e-09	-1.47682827566559e-08	2.21530060221851e-08	-1.47685185520782e-08;...
        4.03243467777148e-08	1.61309378236735e-07	-2.41970090967650e-07	1.61311830983905e-07;...
        -2.35746720559148e-13	3.69154491801782e-09	-1.47685185240281e-08	1.84614611047725e-08;...
        2.45206180642458e-12	-4.03218938420706e-08	1.61311830610115e-07	-2.01648197004208e-07];
    perf = 0.1.*safex;
    threshold =4.35;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
    rate =0.5;
end
if system=="esp"
    Ts =0.04;
    A = [0.4450 -0.0458;1.2939 0.4402];
    B = [0.0550;4.5607];
    C = [0 1];
    K = [0.2826    0.0960];
    L = [-0.0390;0.4339];
    safex = [-2, -4; 2, 4];
    safer = 1;
    sensor_limit = 2.5;
    actuator_limit = 0.8125;
    perf_region_depth = 0.1;
    perf= perf_region_depth.*safex;
    threshold = 4.35;
    rate = 0.5;
end
if system=="trajectory"
    Ts = 0.1;
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    Ts_new = 0.2;
    Ts=Ts_new;
    old_sys = ss(A,B,C,D,Ts);
    new_sys = d2d(old_sys,Ts_new);
    [A,B,C,D] = ssdata(new_sys);
    Q= eye(size(A,2));
    R= eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    QN = 1500;
    RN = eye(1);
    sys_ss = ss(A,B,C,D,Ts);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [-25,-30;25,30];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 36;
    perf_region_depth = 0.3;
    perf = perf_region_depth.*safex;
    threshold = 4.35;
    rate = 0.7;
    pat_set = {[1 0 0 0 1 0],[1 0 0 0 1 1],[1 0 0 1 1 0],[1 0 0 1 1 1],[1 0 1 0 1 0],[1 0 1 0 1 1],[1 0 1 1 1 0],[1 0 1 1 1 1]};
end
% vehicle suspension control (Multi-Objective Co-Optimization of FlexRay-based Distributed Control System)--in CCM ECU
if system=="suspension_control"
%   states: car position, car velocity, suspension load position, suspension velocity
%   input suspension load force
%   output car position
    Ts = 0.04;
    Ac = [0 1 0 0;-8 -4 8 4;0 0 0 1;80 40 -160 -60];
    Bc = [0;80;20;1120];
    Cc = [1,0,0,0];
    Dc = [0];    
    sys_ct= ss(Ac,Bc,Cc,Dc);
    sys_dt= c2d(sys_ct,Ts);
    [A,B,C,D]= ssdata(sys_dt);  
    % safery margin
    safex = [-80,-800,-400,-2400;80,800,400,2400];
    ini = 0.5.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    threshold = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensor_limit = 20;  % columnwise range of each y
    actuator_limit = 100;   % columnwise range of each u
    qweight= 10000;
    rweight = 0.0001;
    Q= qweight*(C'*C);
    Q(2,2)=10;Q(3,3)=1;Q(4,4)=10;
    R= rweight;
    proc_dev= 0.01; meas_dev=0.001;
    QN = 0.005*(B*B');
    RN = 0.00005;
    [K,S,E] = dlqr(A,B,Q,R);
    L= [0.1298; 0.1642; 0.1312; -0.0622];
    s.uatkon=[1];   % attack on which u
    s.yatkon=[1];   % attack on which y
    rate =0.5;
end
% fuel_injection(Modeling and Control of an Engine Fuel Injection System)--in ECM ECU
if system=="fuel_injection"
    % states: 
    % inputs: inlet valve air flow, combustion torque
    % output: AFR
    Ts = 0.01;
    A = [0.18734,0.13306,0.10468;
        0.08183,0.78614,-0.54529;
        -0.00054 0.10877,0.26882];
    B = [0.00516,-0.0172;
        -0.00073,0.09841;
        -0.00011,0.13589];
    C = [158.16,8.4277,-0.44246];
    D = [0,0];
    Ts_new = 0.1;
    old_sys = ss(A,B,C,D,Ts);
    new_sys = d2d(old_sys,Ts_new);
    [A,B,C,D] = ssdata(new_sys);
    Q= blkdiag(0.1,0.1,1);%blkdiag(1,2.25,25);
    R= blkdiag(1,500);
    Pr= care(A,B,Q);
    K = -inv(R)*B'*Pr;
    qweight= 1;
    rweight = 1;
    proc_dev= 0.001; meas_dev=0.0001;
    QN = eye(size(B,1));%proc_dev^2*(B*B');
    RN = 1;%meas_dev^2;
    L=[0.00150311177912917;
        0.00200806159683866;
        0.000355570756765841];
    safex = [-0.66,-4.5,-15;0.66,4.5,15];
    % initial region of this system to safely start from
    ini = 0.7.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    threshold = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensor_limit = 80;  % columnwise range of each y
    actuator_limit = [20;20];   % columnwise range of each u
    % ref= 14.7;
    % F= [1; -1.5];
    settling_time = 6; 
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
    rate =0.5;
end
if system=="power_sys"
    Ts =1;
    A = [0.66 0.53;
       -0.53 0.13];
    B = [0.34;    %B1=Bp1
        0.53];
    C = eye(2);  
    D = [0 ;
        0];    
    K= [0.0556 0.3306];
    L= [0.36 0.27;  -0.31 0.08];  
    x0 = [1;1];                             % ini state
    x = x0;         
    t0 = [0];                               
    t = t0;
    k=0;
    n=0;    
    open_loop = ss(A,B,C,D,Ts);
    eig_open= eig(A);                   
%     eig_openZOH=eig(Ap1);   
    Q_1c_1=10^3*(C'*C);
    Q_12c_1=[0;0];
    Q_2c_1=10^-4;
    R_1c_1=0.005*(B*B');
    R_2_1=0.005*eye(2);
    p=100;
    %A1= (Ap1-Bp1*K);
    QXU = blkdiag(Q_1c_1,Q_2c_1);
    % QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
    dimension = size(A,1);
    QWV = blkdiag(R_1c_1,R_2_1);
    % QWV=[R_1c_1 zeros(dimension,1);zeros(1,dimension) R_2_1]
    safex = [-0.05,-0.1; 0.05,0.1];
    % initial region of this system to safely start from
    ini = 0.01.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    % th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensor_limit = 40;  % columnwise range of each y
    actuator_limit = [10;10];   % columnwise range of each u
    threshold = 4.35;
    rate =0.4;
end
n=2;
%% timer
allstart = tic;
%% opt
fprintf("Minimum-Length Attack Synthesis...\n");
constraints = [];
flag = 0;
cost = 1;
solved =1;
cost_prev = 0;
cost_now = 0;
pat_func = @(pat,ifexec,ifskip)(pat*(ifexec)+(1-pat)*ifskip); 
%% pattern repeat
pat = []
subseq = [];
while solved ~= 0
    if verbose
     fprintf("trying with minimum attack length "+num2str(n)+"\n")
    end
    pat = ones(1,n);
    subseq = [1 0 0 0 0 1 0 1 0 1 1 1 0 1 0 0 0 1 1 1 0 1 0 1 1 1 0 0 1 1 0];
    % ttc [1 0 0 0 0 0 0 1 1 1 1 1 1](18)
    % suspension [1 1 0 0](6),[1 0 0 1](7);
    % fuel [1,0,0,0,1,1,1,0](9), [1,1,1,1,1,0,0,0,0]
    % four-car [1  0  0  0  0  0  0  0  0  0  0  0  0  1  1  1  1  1  1  1  1  1  1  1  1]
    %[1];%[1     0     1     1     0     0     1     0];%[1     0     0     0     1     1     1];
    repeat = ceil(size(pat,2)/size(subseq,2));
    offset = 0;
    for i=1:repeat    
        for j=1:size(subseq,2)
            if offset+j< size(pat,2)
                pat(offset+j) = subseq(j);
            end
        end
        offset = offset + size(subseq,2);
    end
   %% init
    x0 = sdpvar(size(A,2),1);
    constraints = [perf(1,:).' <= x0, x0 <=  perf(2,:).',uncertain(x0)];    
    u = sdpvar(size(B,2),n);
    ua = sdpvar(size(B,2),n);
    uaa = sdpvar(size(B,2),n);
    x = sdpvar(size(A,2),n+1);
    xa = sdpvar(size(A,2),n+1);
    xhat = sdpvar(size(A,2),n+1);
    xhata = sdpvar(size(A,2),n+1);
    e = sdpvar(size(A,2),n+1);
    ea = sdpvar(size(A,2),n+1);
    y = sdpvar(size(C,1),n+1);
    ya = sdpvar(size(C,1),n+1);
    r = sdpvar(size(C,1),n+1);
    ra = sdpvar(size(C,1),n+1);
    
    up = sdpvar(size(B,2),n);
    uap = sdpvar(size(B,2),n);
    uaap = sdpvar(size(B,2),n);
    xp = sdpvar(size(A,2),n+1);
    xap = sdpvar(size(A,2),n+1);
    xhatp = sdpvar(size(A,2),n+1);
    xhatap = sdpvar(size(A,2),n+1);
    ep = sdpvar(size(A,2),n+1);
    eap = sdpvar(size(A,2),n+1);
    yp = sdpvar(size(C,1),n+1);
    yap = sdpvar(size(C,1),n+1);
    rp = sdpvar(size(C,1),n+1);
    rap = sdpvar(size(C,1),n+1);

    au = sdpvar(1,n);
    ay = sdpvar(1,n);

    x(:,1) = x0;
    xhat(:,1) = x0;
    u(:,1) = -K*x0;
    y(:,1) = C*x(:,1);
    r(:,1) = y(:,1)-C*xhat(:,1);
    e(:,1) = x(:,1)-xhat(:,1);

    ua(:,1) = u(:,1);
    uaa(:,1) = u(:,1);
    xa(:,1) = x(:,1);
    xhata(:,1) = xhat(:,1);
    ya(:,1) = y(:,1);
    ra(:,1) = r(:,1);
    ea(:,1) = e(:,1);
   
    xp(:,1) = x0;
    xhatp(:,1) = x0;
    up(:,1) = -K*x0;
    yp(:,1) = C*x(:,1);
    rp(:,1) = y(:,1)-C*xhat(:,1);
    ep(:,1) = x(:,1)-xhat(:,1);
    uap(:,1) = u(:,1);
    uaap(:,1) = u(:,1);
    xap(:,1) = x(:,1);
    xhatap(:,1) = xhat(:,1);
    yap(:,1) = y(:,1);
    rap(:,1) = r(:,1);
    eap(:,1) = e(:,1);
%% sim
    for i=1:n
        if i > 1 
            u(:,i) = pat_func(pat(i),(-K*xhat(:,i)),u(:,i-1));
            ua(:,i) = pat_func(pat(i),(-K*xhata(:,i)),ua(:,i-1));
            uaa(:,i) = pat_func(pat(i),(-K*xhata(:,i)) + au(i-1),uaa(:,i-1));
            
            up(:,i) = -K*xhatp(:,i);
            uap(:,i) = -K*xhatap(:,i);
            uaap(:,i) = -K*xhatap(:,i) + au(i-1);
        end

        x(:,i+1) = A*x(:,i) + B*u(:,i);
        xa(:,i+1) = A*xa(:,i) + B*uaa(:,i);
        
        xp(:,i+1) = A*xp(:,i) + B*up(:,i);
        xap(:,i+1) = A*xap(:,i) + B*uaap(:,i);

        y(:,i+1) = pat_func(pat(i),C*x(:,i+1),y(:,i));
        ya(:,i+1) = pat_func(pat(i),C*xa(:,i+1) + ay(i),ya(:,i));

        r(:,i+1) = pat_func(pat(i),y(:,i+1) - C*(A*xhat(:,i) + B*u(:,i)),r(:,1));%zeros(size(r(:,i),1),1));
        ra(:,i+1) = pat_func(pat(i),ya(:,i+1) - C*(A*xhata(:,i) + B*ua(:,i)),ra(:,1));%zeros(size(ra(:,i),1),1));

        yp(:,i+1) = C*xp(:,i+1);
        yap(:,i+1) = C*xap(:,i+1) + ay(i);
        
        rp(:,i+1) = yp(:,i+1) - C*(A*xhatp(:,i) + B*up(:,i));
        rap(:,i+1) = yap(:,i+1) - C*(A*xhatap(:,i) + B*uap(:,i));
    
        xhat(:,i+1) = A*xhat(:,i) + B*u(:,i) + L*r(:,i+1);
        xhata(:,i+1) = A*xhata(:,i) + B*ua(:,i) + L*ra(:,i+1);

        xhatp(:,i+1) = A*xhatp(:,i) + B*up(:,i) + L*rp(:,i+1);
        xhatap(:,i+1) = A*xhatap(:,i) + B*uap(:,i) + L*rap(:,i+1);
    
        e(:,i+1) = x(:,i+1) - xhat(:,i+1);
        ea(:,i+1) = xa(:,i+1) - xhata(:,i+1);
        
        ep(:,i+1) = xp(:,i+1) - xhatp(:,i+1);
        eap(:,i+1) = xap(:,i+1) - xhatap(:,i+1);

        constraints = [constraints,... 
                            norm(ra(:,i+1),inf) <= threshold-0.0001,...
                            (-1)*sensor_limit <= ya(:,i+1), ya(:,i+1) <= sensor_limit, ...
                            (-1)*sensor_limit <= ay(i), ay(i) <= sensor_limit, ...
                            (-1)*sensor_limit <= y(:,i+1), y(:,i+1) <= sensor_limit, ...
                            (-1)*actuator_limit <= au(i), au(i) <= actuator_limit,...
                            (-1)*actuator_limit <= u(:,i), u(:,i) <= actuator_limit,...
                            (-1)*actuator_limit <= ua(:,i), ua(:,i) <= actuator_limit,...
                            (-1)*actuator_limit <= uaa(:,i), uaa(:,i) <= actuator_limit];
        if i < n
            constraints = [constraints,norm(au(i+1),1)+0.01.*(actuator_limit)>=norm(au(i),1)];%,...
%                 || norm(au(i+1),1)+0.01.*(actuator_limit)<=norm(au(i),1)];
        end
        cost_prev = cost_now;
        cost_now =(2.*safex(2,:)' - abs(xa(:,i+1)))'*(2.*safex(2,:)' - abs(xa(:,i+1)));
        cost = cost + cost_now;
    end
    j =1;
    esterr_safety_lim =25;
    for j = 1: size(A,1)
        sol = optimize([constraints,safex(1,j)-0.001 >= xa(j,i+1)],[],ops);
        solved = sol.problem;
        if solved ~= 0
            if min(value(x0) == perf(1,:)') || min(value(x0) < 0)
                sol = optimize([constraints,safex(2,j)+0.001<= xa(j,i+1)],[],ops);
            end
            if min(value(x0) == perf(2,:)') || min(value(x0) >= 0)
                sol = optimize([constraints,safex(1,j)-0.001<= xa(j,i+1)],[],ops);
            end
            solved = sol.problem;
        else 
            break;
        end
    end
    n = n+1;
end
fprintf("Minimum attack length ")
n = n-1
fprintf("Minimum Length Attack on actuator data : ")
u_a = value(au)
fprintf("Minimum Length Attack on sensor data : ")
y_a = value(ay)
if length(u_a) < n
    u_a = [u_a,zeros(1,n-length(u_a))];
    y_a = [y_a,zeros(1,n-length(y_a))];
end
%% pattern syn algo
fprintf("Optimal Pattern Synthesis...\n")
patsynstart =tic;
% for calculation according to the proof (eq 10)
esterrlist = zeros(n,n);
difflist = zeros(n,n);
difflist1 = zeros(n,n);
patlist = num2cell(zeros(n,n));
AA = eye(size(A));
LL = eye(size(L));
lhs = 0;
rhs = 0;
l =1;
for k= 1:n-1
   for l = 1:n-k
       excessless = 0;
       AA = eye(size(A));
       LL = eye(size(L));
       lhs = 0;
       rhs = 0;
       pat = ones(1,n);
       for i = 1:l
          % for calculation according to the proof (eq 10)
          % execution at i and drop from i+1 (=j)
          AA = A^(i-1);
%           lhs = lhs + AA*B*(value(au(l-i+1))-value(au(k)));
            if k>1
              lhs = lhs + AA*B*(value(au(k+l-i))-value(au(k-1)));
            else
              lhs = lhs + AA*B*(value(au(k+l-i)));                
            end
          rhs = rhs + AA*L*(value(ra(k+l-i+1))-value(r(k+l-i+1)));
       end
       const=1;
       excess = norm(lhs,inf) - norm(rhs,inf);
       if norm(rhs,inf) ~= 0
            excess1 = (excess*100)/norm(rhs,inf);
       else
            excess1 = (norm(lhs,inf)*100)/(norm(rhs,inf)+const);
       end
        
      if min(excess1) > 0          
         pat(k+1:k+l)= 0;
         % not allowing subpats without rate sat
%          if sum(pat(1:k+l))>=rate*(k+l)
%              difflist(k,k+l)= min(excess);
%              patlist{k,k+l} = pat;
%          end
         % allowing subpats without rate sat
          if sum(pat)>=rate*n
             difflist(k,k+l)= min(excess);
             difflist1(k,k+l)= min(excess1);
             patlist{k,k+l} = pat;
         end
      else
          pat(k+1:k+l)= 1;
          patlist{k,k+l} = pat;
       end
    end
 end

% dp with memoization for optimal merging of subpatterns
finalpat = ones(1,n);
diffvals = [];
subpats = {};
subpatstart =[];
subpatend = [];

% storing subsequences in sorted order of starting index
% eg 1001111 is (1,3), then comes (1,4),(2,3),.. etc.
ii = 1;
for k = 1:n
    for l = 1:n
        if difflist(k,l)~=0 && l>k
           diffvals(ii) = difflist(k,l);
           subpats{ii} = patlist{k,l};
           subpatstart(ii) = k;
           subpatend(ii) = l;
           ii = ii+1;
        end
    end
end
M = zeros(length(diffvals),n);
resultpat = num2cell(ones(length(diffvals),n));

% dp for optimal pattern with max excess
for i = 1:length(diffvals)
    for j=1:n
        if verbose
            fprintf("----iteration for M("+num2str(i)+" ,"+num2str(j)+") i.e. "+strcat(num2str(subpats{i}))+" at length "+num2str(j)+"----\n");
        end
        currpat = subpats{i};
        % for the first subpattern row
        if i == 1
            if j >= subpatend(i) && sum(currpat(1:j))>=rate*j 
%               update j=l(i) onwards if satisfies length at current length else 0
                if verbose
                    fprintf("Updating for i =1 M("+num2str(i)+" ,"+num2str(j)+") if rate sat with del e = diffvals("+num2str(i)+")("+num2str(diffvals(i))+")\n");
                end
                M(i,j)= diffvals(i);
                resultpat{i,j} = subpats{i};
                if verbose
                    fprintf("Updating for i=1 resultpat("+num2str(i)+" ,"+num2str(j)+") with subpats("+num2str(i)+"),"+strcat(num2str(subpats{i}))+"\n");
                end    
            else
                if verbose
                    fprintf("Updating for i=1 M("+num2str(i)+" ,"+num2str(j)+") ,"+num2str(j)+") if rate not sat or j<l with del e = 0\n");
                end
                M(i,j)= 0;
                resultpat{i,j} = subpats{i};
                if verbose
                    fprintf("Updating for i=1 resultpat("+num2str(i)+" ,"+num2str(j)+") with subpats("+num2str(i)+"),"+strcat(num2str(ones(1,n)))+"\n");
                end
            end
         else
                % for j < l(i)
                if j < subpatend(i)
                    if verbose
                        fprintf("Updating M("+num2str(i)+" ,"+num2str(j)+") with M(i-1,j) ="+M(i-1,j)+" since "+num2str(j)+" < subpatend("+num2str(i)+")="+num2str(subpatend(i))+"\n"); 
                    end
                    M(i,j)= M(i-1,j);
                    if verbose
                        fprintf("Updating resultpat("+num2str(i)+" ,"+num2str(j)+") with pattern ="+strcat(num2str(resultpat{i-1,j}))+" ,and M("+num2str(i)+" ,"+num2str(j)+") with del e = (restnzdiffs(j)), i.e. "+num2str(M(i-1,j))+"\n");
                    end
                    resultpat{i,j} = resultpat{i-1,j};
                end
                % for j >= l(i)
                if j >= subpatend(i) 
                    curpat =subpats{i};
                    if subpatstart(i) > 1
                        mergedpat = resultpat{i,subpatstart(i)-1}.*subpats{i}
                        mergeddiff = M(i,subpatstart(i)-1)+diffvals(i)
                    else
                        mergedpat = subpats{i};
                        mergeddiff = diffvals(i);
                    end
                    if sum(mergedpat(1:j))>=rate*j
                       if verbose
                        fprintf("Updating if merged rate sat : "+strcat(num2str(subpats{i}))+" , "+num2str(j)+"\n");
                       end
                       M(i,j) = max(mergeddiff,M(i-1,j));
                       if M(i,j)==mergeddiff
                           if verbose
                            fprintf("Updating M("+num2str(i)+" ,"+num2str(j)+") with pattern ="+strcat(num2str(mergedpat))+" ,and del e = (mergeddiff), i.e. "+num2str(mergeddiff)+"\n");
                           end
                           resultpat{i,j} = mergedpat;
                       elseif M(i,j)==M(i-1,j)
                           if verbose
                            fprintf("Updating M("+num2str(i)+" ,"+num2str(j)+") with pattern ="+strcat(num2str(resultpat{i-1,j}))+" ,and del e = (M(i-1,j)), i.e. "+num2str(M(i-1,j))+"\n");
                           end
                           resultpat{i,j} = resultpat{i-1,j}; 
                       end  
                    elseif sum(curpat(1:j)) >= rate*j
                       if verbose
                           fprintf("Updating if merged rate not sat, current rate sat : "+strcat(num2str(subpats{i}))+" , "+num2str(j)+"\n");
                           fprintf("Updating M("+num2str(i)+" ,"+num2str(j)+") with del e = max(M("+num2str(i-1)+","+num2str(j)+",diffvals("+num2str(i)+")\n");
                       end
                       M(i,j) = max(M(i-1,j),diffvals(i)); 
                       if M(i,j)==diffvals(i)
                           if verbose
                            fprintf("Updating M("+num2str(i)+" ,"+num2str(j)+") with pattern ="+strcat(num2str(patlist{i}))+" ,and del e = (diffvals(i)), i.e. "+num2str(diffvals(i))+"\n");
                           end
                           resultpat{i,j} = subpats{i};
                       elseif M(i,j)==M(i-1,j)
                           if verbose
                            fprintf("Updating M("+num2str(i)+" ,"+num2str(j)+") with pattern ="+strcat(num2str(resultpat{i-1,j}))+" ,and del e = (M(i-1,j)), i.e. "+num2str(M(i-1,j))+"\n");
                           end
                           resultpat{i,j} = resultpat{i-1,j}; 
                       end  
                    else
                        if verbose
                           fprintf("Updating if any rate not sat : "+strcat(num2str(resultpat{i-1,j}))+" , "+num2str(j)+"\n");
                           fprintf("Updating M("+num2str(i)+" ,"+num2str(j)+") with del e = M("+num2str(i-1)+","+num2str(j)+")\n");
                        end
                       M(i,j) = M(i-1,j); 
                       if verbose
                        fprintf("Updating resultpat("+num2str(i)+" ,"+num2str(j)+") with pattern ="+strcat(num2str(resultpat{i-1,j}))+"\n");
                       end
                       resultpat{i,j} = resultpat{i-1,j};
                    end
                end       
        end
        if verbose
            fprintf("-- M("+num2str(i)+","+num2str(j)+")= "+num2str(M(i,j))+" --\n");
            fprintf("-- resultpat("+num2str(i)+","+num2str(j)+")= "+strcat(num2str(resultpat{i,j}))+" --\n");
        end
    end
end
% finalpat = max(resultpat(:,n))
finalpat=resultpat{M==max(M(:,n))};
fprintf("Optimal pattern of "+num2str(n)+" length for the system "+system+" is "+ strcat(num2str(finalpat))+"\n");
fprintf("Time required for Optimal pattern synthesis : ")
toc(patsynstart)
fprintf("Time required for minimum-length attack synthesis + Optimal Attack synthesis : ");
toc(allstart)
%% plot vars
if vis == 1
    xapi = value(xap);
    xai = value(xa);
    xhati = value(xhata);
    ui = value(ua);
    ri = value(ra);
    eai = value(ea);
    ei = value(e);
    eapi = value(eap);
    epi = value(ep);
    for i =1:n+1
        deleai(:,i) = norm(eai(:,i)-ei(:,i),1);
        deleapi(:,i) = norm(eapi(:,i)-epi(:,i),1);
        rnormpi (:,i) = norm(rap(:,i),inf);
        rnormi (:,i) = norm(ra(:,i),inf);
    end

    clf;
    figure(1)
    hold on;
    % plot(1:n,y_a,'b');
    % plot(1:n,u_a,'r');
    plot(1:n+1,xapi,'r');
    plot(1:n+1,xai,'--r');
    plot(1:n+1,deleai,'g');
    plot(1:n+1,deleapi,'--g');
    % plot(1:n+1,eai(1,:),'--c');
    % plot(1:n+1,eai(2,:),'--g');
    % plot(1:n+1,xi(1,:),'g');
    % plot(1:n+1,xi(2,:),'--g');
    plot(1:n+1,rnormi,'k');
    % plot(1:n+1,rnormpi,'--k');
    legend({'xapi1','xapi2','xai1','xai1','delta ei norm','delta epi norm','rnorm','rpnorm'})
    hold off;
end