clc;clear;close all;
path = ginput() * 100.0;

n_order       = 7;% order of poly                �ߴ��ݶ���ʽ ��Ϊÿ�κ���Ҫ��p,v,a,j����ʼ���յ㹲4*2=8��δ֪��
n_seg         = size(path,1)-1;% segment number  ��������
n_poly_perseg = (n_order+1); % coef number of perseg   ÿһ���ߴ��ݶ���ʽ�а˸�ϵ��p0��p7

ts = zeros(n_seg, 1);     % ÿһ�ζ���ʽ��������Ҫ�ߵ�ʱ�����
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
    ts(i) = 1.0;    %%����ʱ�䣬ÿһ�ε��˶�ʱ���Ϊ1
end
tic
poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);   %%��ȡ���жεĶ���ʽ�����ĸ���ϵ����ע�����������򣬺����polyval����
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);   %%����ʽ��ĳ���ֵʱ��Ҫ����������Ϊ����
toc                                                                     % ע��ploy_coef_x��ֵ��[p10,p11,p12,p13,p14,p15,p16,p17,p20,p21,....]�������ֱַ����ڼ��κ�ÿһ�εĸ�����ϵ��
%disp(['����ʱ�䣺',num2str(toc)]);

% display the trajectory
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;  %%ÿ��tstep��ʱ����ڶ���ʽ������ȡһ�������ͼ�ϣ����Ѹ�����������tstepԽС��Խ���ƻ�ԭԭ����ʽ����
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
    % and y-axis
    
    Pxi(1:8) = poly_coef_x(8*(i+1):-1:8*i+1,1)  %% ����������Ϊ�������У�ͬʱ��ploy_coef_x������������жΣ�ÿ�ΰ˸�ϵ�����Ĵ������ֻ�Ϊ8��һ���С����
    Pyi(1:8) = poly_coef_y(8*(i+1):-1:8*i+1,1)   %% ����¼��ÿһ��Pxi�����ά�����У���Ϊÿһ�θ��Ե�ϵ����   
                                                  %  ����Px1�����˵�һ���ߴζ���ʽ��ϵ��[p7,p6,p5,p4,p3,p2,p1,p0]
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
    start_cond = [waypoints(1), 0, 0, 0];        %��ʼ���p v a j����
    end_cond   = [waypoints(end), 0, 0, 0];      %�յ��p v a j����
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);   % J = p'Qp, ϣ��ʹ��J��С��JΪһ�������ͣ�QΪ�����;���
    %#####################################################
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);  % Aeq [p1,p2,...,pM] = beq ,ϵ��������poly_coef_x/y,����Ҫ���ϵ������
    f = zeros(size(Q,1),1);  % QP�����У�fΪһ�������������������û��һ�������Ϊ0
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);  %�м�����ΪA��b����һ������ʽԼ��������Ap <= b �� ���ұ��ǵ�ʽԼ������ Aeq p = beq
end