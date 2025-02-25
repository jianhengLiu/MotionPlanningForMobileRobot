clc;clear;close all;
path = ginput() * 100.0;

n_order       = 7;% order of poly, 因为是minimum snap,N=2*4(snap)-1 = 7;4表示要满足首尾的pvaj约束
n_seg         = size(path,1)-1;% segment number
n_poly_perseg = (n_order+1); % coef number of perseg；每一段轨迹（多项式）的参数数目

ts = zeros(n_seg, 1);
% calculate time distribution in proportion to distance between 2 points
% 根据节点间的距离按比例分配节点间的时间
dist     = zeros(n_seg, 1);
dist_sum = 0;
% 总期望从起点到终点的时间为25
T        = 25;
t_sum    = 0;

for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum+dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;
    t_sum = t_sum+ts(i);
end
ts(n_seg) = T - t_sum;%为了保证期望时间的实现，消除计算小数导致的时间不准？

% or you can simply set all time distribution as 1
% for i = 1:n_seg
%     ts(i) = 1.0;
% end

% 求解各段的七阶多项式的系数
poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);


% display the trajectory
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;%画点的间隔，每过tstep代入时间画一点
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
    % and y-axis
    Pxi = poly_coef_x(i*n_poly_perseg+1:(i+1)*n_poly_perseg,1);
    Pxi = flipud(Pxi);
    Pyi = poly_coef_y(i*n_poly_perseg+1:(i+1)*n_poly_perseg,1);
    Pyi = flipud(Pyi);
    
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
% 首末的边界条件，p,v,a,j
start_cond = [waypoints(1), 0, 0, 0];
end_cond   = [waypoints(end), 0, 0, 0];
%#####################################################
% STEP 1: compute Q of p'Qp, 希望代价函数最小 J = p'Qp
Q = getQ(n_seg, n_order, ts);
%#####################################################
% STEP 2: compute Aeq and beq
[Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);

f = zeros(size(Q,1),1);
% matlab的求解二次优化问题
% min 0.5*x'*Q*x + f'*x
% 两个[],[]分别表示不等式约束A*x <= b的A和b
% Aeq，beq表示等式约束A*x = b
poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end