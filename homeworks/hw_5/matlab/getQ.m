function Q = getQ(n_seg, n_order, ts)
Q = [];
for k = 1:n_seg
    Q_k = [];
    %#####################################################
    % STEP 1.1: calculate Q_k of the k-th segment
    %
    %
    %
    %因为minimum snap要求J = p'Qp
    %因为snap为四阶导，所以没有p0,p1,p2,p3
    for i = 4:n_order+1
        for j =4:n_order+1
            Q_k(i,j) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(i+j-n_order)*ts(k)^(i+j-n_order);
        end
    end
    Q = blkdiag(Q, Q_k);
end
end