 clc;
% clear all;
% yalmip("clear");
format long g
system = "pow_gen"
ops = sdpsettings('verbose',1);
%% systems
if system=="esp"
    A = [0.4450 -0.0458;1.2939 0.4402];
    B = [0.0550;4.5607];
    C = [0 1];
    K = [0.2826    0.0960];
    L = [-0.0390;0.4339];
    safex = [-1, -2;1, 2];
    safer = 1;
    sensor_limit = 2.5;
    actuator_limit = 0.8125;
    perf_region_depth = 0.1;
    perf= perf_region_depth.*safex;
    threshold = 4.35;
    nonatk_cov = 0.1169;
    safer_center = [-0.7258    1.7258   -0.7250    1.5611]';
    safer_scale = 0.2742;
    t=8;
    rate = 0.5;
end
if system=="trajectory"
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    Q= eye(size(A,2));
    R= eye(size(B,2));
%     [K,S,E] = dlqr(A,B,Q,R);
%     K =[16.0302    5.6622];
    K = [0.9171 1.6356];

    QN = 1500;
    RN = eye(1);
    sys_ss = ss(A,B,C,D,0.1);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
%     L =[0.9902;0.9892];
    L = [0.8327;2.5029];
    safex = [-25,-30;25,30];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 36; %36;
    perf_region_depth = 0.3;
    perf = perf_region_depth.*safex;
    threshold = 4.35;
    nonatk_cov = 12.6;
    safer_center = [18.1468   -9.5400   17.3949   -12.6147]';
    safer_scale = 3.3258;
    t=20;
    rate = 0.5;
    x0 = [0;0];%[0;3];%[0;-0.1820720546];%0.8225294341;0];%[15;-16.6096079397];%[-7.5;9];%perf(2,:)';
end
% linearized at [12.6,13]
if system=="quad_tank"
    A = [-0.0158,0,0.0256,0;0,-0.0109,0,0.0178;...
        0,0,-0.00256,0;0,0,0,-0.0178];
    B = [0.0482,0;0,0.035;...
        0,0.0775;0.0559,0];
    C = [0.5,0,0,0;0,0.5,0,0];
    D = [0];
%     Q= eye(size(A,2));
%     R= eye(size(B,2));
%     [K,S,E] = dlqr(A,B,Q,R);
%     QN = 1500;
%     RN = eye(1);
    sys_ss = ss(A,B,C,D,0.1);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [-25,-30;25,30];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 36; %36;
    perf_region_depth = 0.3;
    perf = perf_region_depth.*safex;
    threshold = 4.35;
    nonatk_cov = 12.6;
    safer_center = [18.1468   -9.5400   17.3949   -12.6147]';
    safer_scale = 3.3258;
    t=20;
    rate = 0.5;
end
% from Secure Control SYstems by Teixeira, single-machine infinite-bus power
% generator linearized at omega(theta_dot) and theta = 0, susceptance param b =1
% states: phase angle(theta), freq deviation(omega)
% output: electric power flow P = b Sin(theta) = theta for small theta
if system=="pow_gen"
    Ts =1;
    A = [0.66,0.53;-0.53,0.13];
    B = [0.34;0.53];
    C = eye(size(A,1));
    D = [0;0];
    K = [0.0556,0.3306];
    L = [0.36,0.27;-0.31,0.08];
    safex = [-0.1,-0.05;0.1,0.05];
    safer = 0.6;
    sensor_limit = 0.1;
    actuator_limit = 0.03;
    perf_region_depth = 0.3;
    perf = perf_region_depth.*safex;
    threshold = 0.01;
%     nonatk_cov = 12.6;
%     safer_center = [18.1468   -9.5400   17.3949   -12.6147]';
%     safer_scale = 3.3258;
    t=20;
    rate = 0.5;
    Da = [0 1;0,0];
    Ba = [0.34,0;0.53,0]; 
    [z,p,gain] = ss2zp(A-B*K,B,C,D);
    lambda =1;
    for i = 1:length(z)
        if abs(z(i)) >=lambda
            lambda=z(i);
        end
    end
    zero_init = sdpvar(size(A,2),1);
    zero_dir = sdpvar(size(B,2),1);
%     cons = [[(lambda.*eye(size(A))-A+B*K),-Ba;C,Da]*[zero_init;zero_dir]==0];
%     optimize(cons)
%     g = value(zero_dir);%[-0.01; 0.01];
%     x0 =value(zero_init);
    g = [-0.01; 0.01];
    x0 = [0;0];
    kappa = 1;
    ak = kappa*lambda*g;
    
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
    safex = [-20,-200,-100,-600;20,200,100,600]%[-15,-15,-20,-20;15,15,20,20];
    % safer region of this system to start from
    ini = 0.5.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    threshold = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [-20;20];  % columnwise range of each y
    actuatorRange = [-1000;1000];   % columnwise range of each u
    qweight= 1000;
    rweight = 0.00001;
    Q= qweight*(C'*C);
    Q(2,2)=10;Q(3,3)=1;Q(4,4)=10;
    R= rweight;
    proc_dev= 0.01; meas_dev=0.001;
    QN = 0.005*(B*B');
    RN = 0.00005;
%     % using lqg ragulator
%     K=[7.6110    0.3412    0.0186    0.0157];
%     L= [0.1298; 0.1642; 0.1312; -0.0622];
    settlingTime = 0.2800/Ts;
%     s.noisy_zvar= 2.6328;
%     s.noisy_zmean= 0.2719;
%     s.noisy_delta= 1.86;
%     s.nonatk_zvar= 12.6041;%15.8507
%     s.nonatk_zmean= 0.6064;
%     s.nonatk_delta= 0.0292;
    s.uatkon=[1];   % attack on which u
    s.yatkon=[1];   % attack on which y
end
%% opt
n =10
pat= ones(1,n);
subseq = [];

pat = ones(1,n);
subseq = [1];% finalpat;%[1];%[1 0 1 0 1 0 1 1 1 1];
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
% pat(2)=0;pat(5)=0;pat(6)=0;pat(17)=0;pat(22)=0;pat(23)=0;pat(25)=0;pat(26)=0;
% pat(3)=0;pat(8)=0;pat(6)=0;pat(10)=0;pat(16:20)=0;pat(22)=0;pat(24)=0;pat(25)=0;pat(27)=0;
% pat(2)=0;pat(10)=0;pat(15)=0;pat(23)=0;pat(26)=0;pat(22)=0;pat(24)=0;

% x0 = sdpvar(size(A,2),1);
% constraints = [constraints,perf(1,:).' <= x0, x0 <=  perf(2,:).']
% u = sdpvar(size(B,2),1);
% x = sdpvar(size(A,2),1);
% xhat = sdpvar(size(A,2),1);
% y = sdpvar(size(C,1),1);
% b = binvar(1,n-1);

x = x0;
% assign(xhat, x0);
xhat = x;
% assign(u,-K*x0);
u = -K*x0;
y = C*x;
r = y-C*xhat;
e = x-xhat;

ua = u;
uaa = u
xa = x;
xhata = xhat;
ya = y;
ra = r;
ea = e;

% au =[31.3200000000002,-31.6800000000002,-32.0400000000002,-32.4000000000001,-32.7600000000001,-33.1200000000002,-33.4800000000002,-33.8400000000002,-34.2000000000001,-34.5600000000001,-34.9200000000001,-35.2800000000001,-35.6400000000000,-36];
% ay =[-3.60365693685043,-30,-3.86716764407482,-3.36419876752477,-3.13558178450900,-2.44005016642648,-1.40861893886108,-0.0401492349047494,1.66644399505527,3.68311504695985,5.77836536562734,-30,-30,10.4635152123376];

% au = [-33.4800000000002, -33.8400000000002, -34.2000000000001, -34.5600000000001,...
%             -34.9200000000001, -35.2800000000001, -35.64, -36];
% ay = [4.34990000000001, 4.84620771268902, 6.36645755225488, 8.21069150382504,...
%         6.21491968791054, 12.0141176436976, 14.0727022043583, 12.8317957595551];
% au = [-35.99313719 -71.98627437 -71.98627437 -71.98627437   0   14.19167443   0.          37.33256927  13.43504349   1.60893935     0.41831677   0.           0.           0.           0      0.        ];
% au = [ 25.0492592   32.69405461  60.61712031  46.2788486   71.16365887    66.10628365  67.3659318   55.90577759   0.         -43.1503309 -22.09388527   0.           0.        ];
% au =[-35.98179279 -35.98179279   0.         -35.50716053 -35.98179279   0.           0.         -35.98179279  -6.64337146  35.98179279   0.           0.           0.        ];
% au =[ 35.95188398  15.21983364   0.          20.89314357  35.95188398   1.77598366  35.95188398  35.95188398  15.57175241  32.61966017 -29.98849737   0.           0.        ];
% au = [ -0.19580562   0.           0.          35.99902097  27.1272989   35.99902097   0.          35.99902097  35.99902097  35.99902097  -32.38020497   0.           0.        ];
% au = [0 0 -12.27734375  28.51861247  24.7136923   20.12764975  16.07710454  12.56830666   9.54731129   7.02119861  10.8498978    4.64257353  18.56893908   0.           0.     0     0   ];
% ay = [-15.01515367 -14.11742439 -13.05044433 -11.53980441  -8.93081496  -7.19985906  -3.65966581  -2.39903816  -1.30516755  -0.5442074  -0.1748743    0.           0.           0.           0.      0.];
% ay =[ -0.81446865   0.21962045  -0.28786589   0.45481024  -0.03475428     -0.60724119  -2.19860593  -6.56865347  -8.97036108 -11.64102807    -16.47363069   0.           0.        ];
% ay =[-1.82072054e-02  1.86147595e+00  1.60094150e+00  2.47635479e+00  5.41100848e+00  7.82392584e+00  8.84410585e+00  1.26548798e+01  1.50696373e+01  1.60034808e+01  1.82455083e+01  0.00000000e+00  0.00000000e+00];
% ay =[ -1.325633    -2.92067134  -3.73245406  -4.66022653  -7.65477417  -8.43301849  -9.40225911 -13.20630786 -15.31791476 -17.89934867 -20.76985566   0.           0.        ];
% ay = [-1.67638846  -2.61876081  -1.6872937   -2.99343667  -3.4030853   -6.15080711  -6.72278332  -8.66679519 -10.66992943 -12.88208587  -14.03330259   0.           0.        ];
% ay = [0 0 -1.22898125  -2.20268597  -3.55151945  -5.49072856  -7.95978554  -10.9204011  -14.32305977 -16.91981063 -22.10089557 -23.06213404 -25.26376068   -30.           -30.       0         0 ];
n = length(au);
pat=ones(1,n);
% au = au_per;%zeros(1,n);
% au(27:28) = [0,0];
% ay = ay_per;%zeros(1,n);
% ay(27:28) = [0,0];
% plot vars
ki = 1;
xi = [value(x0)];
xhati = [value(x0)];
ui = [value(u)];
uai = [value(ua)];
uaai = [value(uaa)];
yi = [value(ya)];
ri = [value(ra)];
ei = [value(ea)];
flag = 1;
cost = (safex(2,:)' - abs(xa))'*(safex(2,:)' - abs(xa));
obj1 = ea.'*ea;
obj2 = (ea-e).'*(ea-e);
trunc = @(x,x_limit)(max(min(x,x_limit),-x_limit));
pat_func = @(pat,ifexec,ifskip)(pat*(ifexec)+(1-pat)*ifskip);
%% for zero dyn
n =101;
N=10;
au =zeros(size(Ba,2),n);
ay =zeros(size(Da,2),n);
for i=1:n-1
    %% zero dyn atk
        kappa=ceil((i+1)/N)
        ak = -kappa*lambda*g;
        au(:,i) = Ba*ak;
        ay(:,i) = Da*ak;
        if min(au(:,i))> actuator_limit
            au(:,i) = actuator_limit;
        elseif au(:,i)< -actuator_limit
            au(:,i) = -actuator_limit;
        end
        if ay(:,i)> sensor_limit
            ay(:,i) = actuator_limit;
        elseif ay(:,i)< -sensor_limit
            ay(:,i) = -sensor_limit;
        end

        if i == 10*ki
            if mod(ki,2)~=0
                pat(i:10*(ki+1)) = 0;
%             pat(i) = 0;
%             if i < n
%                 pat(i+1) = 0;
%             end
            end
            ki = ki+1;
        end


    r = pat_func(pat(i),y - C*(A*xhat + B*u),r);
    ra = pat_func(pat(i),ya - C*(A*xhata + B*ua),0);
    x = A*x + B*u;%+normrnd(0,1/24,[2 1]);
    xa = A*xa + B*uaa;%+normrnd(0,1/24,[2 1]);
%     if pat(i)
    y = pat_func(pat(i),trunc(C*x,sensor_limit),y);%+normrnd(0,1/12,[1 1]);
    ya = pat_func(pat(i),trunc(C*xa + ay(i),sensor_limit),ya);%+normrnd(0,1/12,[1 1]);
%     end
    xhat = A*xhat + B*u + L*r;
    xhata = A*xhata + B*ua + L*ra;
    e = x - xhat;
    ea = xa - xhata;
    %     if pat(i)
        u = pat_func(pat(i),trunc(-K*xhat,actuator_limit),u);
        ua = pat_func(pat(i),trunc(-K*xhata,actuator_limit),ua);
        uaa = pat_func(pat(i),trunc(ua + au(i),actuator_limit),uaa);
%     end
%     constraints = [constraints,(-1)*sqrt(threshold*nonatk_cov) <= C*(xa-xhata),C*(xa-xhata) <= sqrt(threshold*nonatk_cov)];
%     constraints = [constraints, safex(1,:).' <= xa, xa <=  safex(2,:).', ...
%                    constraints, safex(1,:).' <= x, x <=  safex(2,:).', ...
%                         (-1)*sensor_limit <= ay(i), ay(i) <= sensor_limit, ...
%                         (-1)*sensor_limit <= ya, ya <= sensor_limit, ...
%                         (-1)*sensor_limit <= y, y <= sensor_limit, ...
%                         (-1)*actuator_limit <= au(i), au(i) <= actuator_limit...
%                         (-1)*actuator_limit <= ua, ua <= actuator_limit,...
%                         (-1)*actuator_limit <= u, u <= actuator_limit];
%     cost = cost + (safex(2,:)'-x)'*(safex(2,:)'-x);
%     cost = cost + (safex(2,:)'-x).'*(-safex(1,:)'+x);
%     cost = (safex(2,:)'-x).'*(-safex(1,:)'+x);
%     cost = [cost,(safex(2,:)' - abs(xa))'*(safex(2,:)' - abs(xa))];
%     obj_f = @(xa)cost;
%     obj1 = [obj1, ea.'*ea];
%     obj2 = [obj2, (ea-e).'*(ea-e)];
%     optimize(constraints,obj_f(xa));
%     attack = optimizer(constraints, obj_f(xa),[],[xa;xhata],[ay(i);au(i)]);
    xi = [xi,value(xa)];
    xhati = [xhati,value(xhata)];
    ui = [ui,value(ua)];
    uai = [uai,value(ua)];
    uaai = [uaai,value(uaa)];
    yi = [yi,value(ya)]
    ei = [ei,value(ea)];
%     ri = [ri,norm(ra,1)];
%     if i == 1
%         xi(:,1) = [value(x0)];
%         xhati(:,1) = [value(x0)];
%         ui(:,1) = [value(u)];
%         uai(:,1) = [value(ua)];
%         uaai(:,1) = [value(uaa)];
%         ri(:,1) = [abs(ra)];
%         ei(:,1) = [value(ea)];
%     end
end
th = threshold*ones(1,n);
% for i = 6:13
%     th(i) = th(i)-0.11*(i-6);
% end
% for i = 13:14
%     th(i) = th(i)+0.11*(15-i);
% end
% th(13)=th(13)+0.2;
% clf;
hold on;
% plot(1:n,value(ay),'b');
% plot(1:n,value(au),'r');
% plot(1:n+1,uaai,'--r');
plot(1:n,xi,'g');
plot(1:n,ei,'k');
plot(1:n,ri,'--b');
% legend({'ay1','ay2','au1','au2','ua','xa1','xa2','ea1','ea2','ra1','ra2'});
legend({'xa1','xa2','ea1','ea2','ra1','ra2'});
hold off;
% clf;
% figure("Name","A succesfull but stealthy FDI")
% hold on;
% plot(1:n,value(ay),'b');
% plot(1:n,value(au),'r');
% plot(1:n,uaai,'--r');
% plot(1:n,xi(1,:),'b');
% plot(1:n,safex(2,1)*ones(1,n),'--b');
% plot(1:n,xi(2,:),'r');
% plot(1:n,safex(2,2)*ones(1,n),'--r');
% plot(1:n,ri,'k');
% plot(1:n,th,'--k');
% plot(1:n+1,ri,'--b');
% plot(1:n,0.1*pat)
% legend({'ay1','ay2','au1','au2','ua','xa1','xa2','ea1','ea2','pat'});
% legend({'ua''x1','x2','r','th'})
% legend({'V','Safe V','residue','threshold'})
% hold off;
% hold on;
% plot(1:n,value(ay),'b');
% plot(1:n,value(au),'r');
% plot(1:n,100.*pat,'c');
% plot(1:n+1,cost,'g');
% plot(1:n+1,obj2,'k');
% legend({'ri'})
% hold off;
% pat
% figure;
% title("A Succesful but stealthy FDI on TTC")
% hold on;
% [t,y1,y2] = plotyy(1:n,[xi(2,:);safex(1,2)*ones(1,n)],1:n,[ri;th]);
% legend({'V','Safe V','residue','threshold'});
% xlabel('time (sec)')
% ylabel(y1,'Velocity (m/s)')
% ylabel(y2,'Residue')
% y1(1).LineStyle = '-';
% y1(2).LineStyle = '--';
% y2(1).LineStyle = '-';
% y2(2).LineStyle = '--';
% set(t,'FontSize',36);
% set(y1,{'LineWidth'},{3;3});
% set(y2,{'LineWidth'},{3;3});
% ylabel(y1,{'Velocity (m/s)','Velocity (m/s)'})
% ylabel(y2,{'residue','residue'});
% hold(t(1));
% [t1,z1] = plot(1:n,safex(2,2)*ones(1,n));
% legend({'Safe V','threshold'});
% z1.LineStyle = '--r';
% hold(t(2));
% [t1,z2] = plot(1:n,th);
% z1.LineStyle = '--b';
% plot(1:n,ri,'k');
% plot(1:n,th,'--k');