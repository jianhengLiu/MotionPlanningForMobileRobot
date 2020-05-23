clc;clear;close all;
path = ginput() * 100.0;

n_order       = 7;% order of poly                七次幂多项式 因为每段函数要解p,v,a,j的起始与终点共4*2=8个未知量
n_seg         = size(path,1)-1;% segment number  函数段数
n_poly_perseg = (n_order+1); % coef number of perseg   每一个七次幂多项式有八个系数p0到p7

ts = zeros(n_seg, 1);     % 每一段多项式函数所需要走的时间分配
% calculate time distribution in proportion to distance between 2 points
% dist     = zeros(n_seg, 1);
% dist_sum = 0;
% T        = 25;
% t_sum    = 0;
% 
% for i = 1:n_seg
%     dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
%     dist_sum = dist_sum+dist(i);
% end
% for i = 1:n_seg-1
%     ts(i) = dist(i)/dist_sum*T;
%     t_sum = t_sum+ts(i);
% end
% ts(n_seg) = T - t_sum;

% or you can simply set all time distribution as 1
for i = 1:n_seg
    ts(i) = 1.0;    %%均分时间，每一段的运动时间均为1
end
tic
poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);   %%获取所有段的多项式函数的各项系数，注意是升幂排序，后面的polyval计算
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);   %%多项式在某点的值时需要降幂输入作为参数
toc                                                                     % 注：ploy_coef_x的值是[p10,p11,p12,p13,p14,p15,p16,p17,p20,p21,....]两个数字分别代表第几段和每一段的各幂项系数
%disp(['运行时间：',num2str(toc)]);

% display the trajectory
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;  %%每过tstep的时间就在多项式函数上取一点绘制在图上，最后把各点连起来，tstep越小，越近似还原原多项式曲线
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
    % and y-axis
    
    Pxi(1:8) = poly_coef_x(8*(i+1):-1:8*i+1,1)  %% 改升幂排列为降幂排列，同时将ploy_coef_x这个包含了所有段（每段八个系数）的大向量分化为8个一组的小向量
    Pyi(1:8) = poly_coef_y(8*(i+1):-1:8*i+1,1)   %% 并记录在每一段Pxi这个八维向量中，作为每一段各自的系数。   
                                                  %  例如Px1包含了第一段七次多项式的系数[p7,p6,p5,p4,p3,p2,p1,p0]
    for t = 0:tstep:ts(i+1)
        X_n(k)  = polyval(Pxi, t);
        Y_n(k)  = polyval(Pyi, t);
        k = k + 1;
    end
end
 
plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
hold on
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));

function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];        %起始点的p v a j条件
    end_cond   = [waypoints(end), 0, 0, 0];      %终点的p v a j条件
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);   % J = p'Qp, 希望使得J最小，J为一个二次型，Q为二次型矩阵
    %#####################################################
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);  % Aeq [p1,p2,...,pM] = beq ,系数向量即poly_coef_x/y,现在要求解系数向量
    f = zeros(size(Q,1),1);  % QP问题中，f为一次项的向量，本程序中没有一次项，故设为0
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);  %中间两个为A，b，是一个不等式约束条件，Ap <= b 。 最右边是等式约束条件 Aeq p = beq
end