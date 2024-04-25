
t = 0: closed_loops_sc_40{1}.Ts : 0.2;
% t = 0: closed_loops_sc_40{2}.Ts : 0.2;

u = zeros(1,size(t,2));

u(6) = 0.1;
u(5) = 0.15;
x0 =[0.003;0.03;0.02;0.08];

[y_sc_0_200, tout, x_sc_0_200] = lsim(closed_loops_sc_40{1},u,t,[x0;x0]);
% [y_sc_0_200, tout, x_sc_0_200] = lsim(closed_loops_sc_40{2},u,t,[x0;x0]);

x00 = x_sc_0_200(end,:)';

t = 0.2: closed_loops_sc_30{1}.Ts : 0.4;
% t = 0.2: closed_loops_sc_40{1}.Ts : 0.4;

u = zeros(1,size(t,2));

[y_sc_200_400, tout1, x_sc_200_400] = lsim(closed_loops_sc_30{1},u,t,x00);
% [y_sc_200_400, tout1, x_sc_200_400] = lsim(closed_loops_sc_40{1},u,t,x00);


x000 = x_sc_200_400(end,:)';

t = 0.4: closed_loops_sc_40{1}.Ts : 0.5;
% t = 0.4: closed_loops_sc_40{2}.Ts : 0.5;

u = zeros(1,size(t,2));

[y_sc_400_500, tout11, x_sc_400_500] = lsim(closed_loops_sc_40{1},u,t,x000);
% [y_sc_400_500, tout11, x_sc_400_500] = lsim(closed_loops_sc_40{2},u,t,x000);


x0000 = x_sc_400_500(end,:)';

t = 0.5: closed_loops_sc_40{1}.Ts : 0.7;
% t = 0.5: closed_loops_sc_40{2}.Ts : 0.7;

u = zeros(1,size(t,2));

[y_sc_500_700, tout111, x_sc_500_700] = lsim(closed_loops_sc_40{1},u,t,x0000);
% [y_sc_500_700, tout111, x_sc_500_700] = lsim(closed_loops_sc_40{2},u,t,x0000);


y_sc_1 = [y_sc_0_200;y_sc_200_400;y_sc_400_500;y_sc_500_700]
x_sc_1 = [x_sc_0_200;x_sc_200_400;x_sc_400_500;x_sc_500_700];
t_sc_1 = [tout;tout1;tout11;tout111]
costmat_sc_1 = x_sc_1*x_sc_1';
cost_sc_1 = diag(costmat_sc_1);


% -------------esp--------------

t = 0: closed_loops_esp_20{2}.Ts : 0.2;
% t = 0: closed_loops_esp_40{1}.Ts : 0.2;

u = zeros(1,size(t,2));


x0 =[0.03;0.01];

[y_esp_0_200, tout, x_esp_0_200] = lsim(closed_loops_esp_20{2},u,t,[x0;x0]);
% [y_esp_0_200, tout, x_esp_0_200] = lsim(closed_loops_esp_40{1},u,t,[x0;x0]);

x00 = x_esp_0_200(end,:)';

t = 0.2: closed_loops_esp_20{2}.Ts : 0.4;
% t = 0.2: closed_loops_esp_40{1}.Ts : 0.4;

u = zeros(1,size(t,2));

u(end) = -1;
u(5) = -0.6;

[y_esp_200_400, tout1, x_esp_200_400] = lsim(closed_loops_esp_20{2},u,t,x00);
% [y_esp_200_400, tout1, x_esp_200_400] = lsim(closed_loops_esp_40{1},u,t,x00);




x000 = x_esp_200_400(end,:)';

t = 0.4: closed_loops_esp_20{1}.Ts : 0.5;
% t = 0.4: closed_loops_esp_20{1}.Ts : 0.5;

u = zeros(1,size(t,2));

[y_esp_400_500, tout11, x_esp_400_500] = lsim(closed_loops_esp_20{1},u,t,x000);
% [y_esp_400_500, tout11, x_esp_400_500] = lsim(closed_loops_esp_20{1},u,t,x000);

y_esp_1 = [y_esp_0_200;y_esp_200_400;y_esp_400_500]


x0000 = x_esp_400_500(end,:)';

% t = 0.5: closed_loops_esp_20{2}.Ts : 0.7;
t = 0.5: closed_loops_esp_40{1}.Ts : 0.7;

u = zeros(1,size(t,2));

% [y_esp_500_700, tout111, x_esp_500_700] = lsim(closed_loops_esp_20{2},u,t,x0000);
[y_esp_500_700, tout111, x_esp_500_700] = lsim(closed_loops_esp_40{1},u,t,x0000);

y_esp_1 = [y_esp_0_200;y_esp_200_400;y_esp_400_500;y_esp_500_700];
x_esp_1 = [x_esp_0_200;x_esp_200_400;x_esp_400_500;x_esp_500_700];
t_esp_1 = [tout;tout1;tout11;tout111]
costmat_esp_1 = x_esp_1*x_esp_1';
cost_esp_1 = diag(costmat_esp_1);


%
figure(1)
hold on;
plotyy(t_sc_1,y_sc_1(:,1),t_sc_1,cost_sc_1)
hold off;
figure(2)
hold on;
plotyy(t_esp_1,y_esp_1(:,1),t_esp_1,cost_esp_1);
hold off;








