function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
n_all_poly = n_seg*(n_order+1);
d_order = 4;
%#####################################################
% 贝塞尔曲线必经过始末点
% STEP 2.1 p,v,a,j constraint in start
Aeq_start = zeros(d_order,n_all_poly);
for k = 0:d_order-1
%     YanhTriangle
    Aeq_start(k+1,1:k+1) = factorial(n_order)/factorial(n_order-k)*YangHTriangle(k+1)*ts(1)^(1-k);
end
beq_start = start_cond';

%#####################################################
% STEP 2.2 p,v,a,j constraint in end
Aeq_end = zeros(d_order,n_all_poly);
for k = 0:d_order-1
    Aeq_end(k+1,n_all_poly-k:n_all_poly)= factorial(n_order)/factorial(n_order-k)*YangHTriangle(k+1)*ts(n_seg)^(1-k);
end
beq_end = end_cond';

%#####################################################
% STEP 2.3 p,v,a,j continuity constrain between 2 segments
Aeq_con = zeros((n_seg-1)*d_order,n_all_poly);
beq_con = zeros((n_seg-1)*d_order,1);
% v,a,j
for k = 0:d_order-1
    start_idx_1 = k*(n_seg-1);
    for j = 0:n_seg-2
        start_idx_2 = (n_order+1)*(j+1);
        Aeq_con(start_idx_1+j+1,start_idx_2-k:start_idx_2) = factorial(n_order)/factorial(n_order-k)*YangHTriangle(k+1)*ts(j+1)^(1-k);
        Aeq_con(start_idx_1+j+1,start_idx_2+1:start_idx_2+1+k) = -factorial(n_order)/factorial(n_order-k)*YangHTriangle(k+1)*ts(j+2)^(1-k);
    end
end
Aeq = [Aeq_start; Aeq_end; Aeq_con]
beq = [beq_start; beq_end; beq_con]
end