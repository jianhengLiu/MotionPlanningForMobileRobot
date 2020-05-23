function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    %
    %
    %
    %
    %
    Ct = zeros(8*n_seg,4*(n_seg+1));
    for j = 1:4
        Ct(j,j) = 1;
    end
    
    for i=1:n_seg-1
        Ct(i*8-3,4+i) = 1;
        Ct(i*8-2,(n_seg-1+8)+3*(i-1)+1) = 1;
        Ct(i*8-1,(n_seg-1+8)+3*(i-1)+2) = 1;
        Ct(i*8,(n_seg-1+8)+3*(i-1)+3) = 1;
        Ct(i*8+1,4+i) = 1;
        Ct(i*8+2,(n_seg-1+8)+3*(i-1)+1) = 1;
        Ct(i*8+3,(n_seg-1+8)+3*(i-1)+2) = 1;
        Ct(i*8+4,(n_seg-1+8)+3*(i-1)+3) = 1;
    end
    
    Ct(8*n_seg-3,4+n_seg) = 1;
    Ct(8*n_seg-2,4+n_seg+1) = 1;
    Ct(8*n_seg-1,4+n_seg+2) = 1;
    Ct(8*n_seg  ,4+n_seg+3) = 1;
    
end