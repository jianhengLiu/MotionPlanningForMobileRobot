function Ct = getCt(n_seg, n_order)
%#####################################################
% STEP 2.1: finish the expression of Ct
%
%
%
%
%状态变量数
d_order = 4;
%     起始点
Ct_start = zeros(d_order,d_order*(n_seg+1));
Ct_start(:,1:d_order) = eye(d_order);

%     中间点
Ct_mid = zeros(2*d_order*(n_seg-1),d_order*(n_seg+1));
for j= 0:n_seg-2
    Cj = zeros(d_order,d_order*(n_seg+1));
    Cj(1,d_order+j+1) = 1;
    start_idx_2 = 2*d_order+(n_seg-1)+3*j;
    Cj(2:d_order, start_idx_2+1:start_idx_2+3) = eye(d_order-1);
    start_idx_1 = 2*d_order*j;
    Ct_mid(start_idx_1+1:start_idx_1+2*d_order,:) = [Cj;Cj];
end

%     终止点
Ct_end = zeros(d_order,d_order*(n_seg+1));
% d_order+n_seg:起始点四个+中间点个数+1
% 2*d_order+n_seg-1；起始点和终止点各四个+中间点个数
Ct_end(:,d_order+n_seg:2*d_order+n_seg-1) = eye(d_order);

Ct = [Ct_start;Ct_mid;Ct_end];
end