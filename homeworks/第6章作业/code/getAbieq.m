function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    n_coef = n_order+1;
    %#####################################################
    % STEP 3.2.1 p constraint
    Aieq_p = [];
    bieq_p = [];
    Aeq_p = zeros(3,n_all_poly);
    beq_p = zeros(3,1);
    for i=1:n_seg-1
        tvec_p = calc_tvec(ts(i),n_order,0);
        tvec_p_next = calc_tvec(0,n_order,0);
        Aieq_p(2*i-1:2*i,n_coef*i+1:n_coef*(i+1)) = [tvec_p;-tvec_p_next];
        bieq_p(2*i-1:2*i) = [corridor_range(i,1) corridor_range(i,2)];
    end
        
    
    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v = [];
    bieq_v = [];
    for i=1:n_seg-1
        tvec_v = calc_tvec(ts(i),n_order,1);
        tvec_v_next = calc_tvec(0,n_order,1);
        Aieq_v(2*i-1:2*i,n_coef*i+1:n_coef*(i+1)) = [tvec_v;-tvec_v_next];
        bieq_v(2*i-1:2*i) = [0 0];
    end
    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = [];
    bieq_a = [];
    for i=1:n_seg-1
        tvec_a = calc_tvec(ts(i),n_order,2);
        tvec_a_next = calc_tvec(0,n_order,2);
        Aieq_a(2*i-1:2*i,n_coef*i+1:n_coef*(i+1)) = [tvec_a;-tvec_a_next];
        bieq_a(2*i-1:2*i) = [0 0];
    end
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
    Aieq = Aieq_p;
    bieq = bieq_p;
end