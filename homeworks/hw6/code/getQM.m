function [Q, M] = getQM(n_seg, n_order, ts)
Q = [];
M = [];
d_order = 4;
M_j = getM(n_order);
for j = 0:n_seg-1
    %#####################################################
    % STEP 2.1 calculate Q_k of the k-th segment
    Q_j = zeros(n_order+1,n_order+1);
    for i = d_order:n_order
        for l =i:n_order
            Q_j(i+1,l+1) = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)/(i+l-n_order)*ts(j+1)^(3-2*n_order);
            Q_j(l+1,i+1) =  Q_j(i+1,l+1);
        end
    end
    
    Q = blkdiag(Q, Q_j);
    M = blkdiag(M, M_j);
end
end