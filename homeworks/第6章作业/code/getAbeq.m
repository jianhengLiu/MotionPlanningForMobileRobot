function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    n_coef = n_order+1;
    v0 = 0.0;
    a0 = 0.0;
    %j0 = 0.0;
    ve = 0.0;
    ae = 0.0;
    %je = 0.0;
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = [];
    beq_start = [];
    Aeq_start = zeros(3,n_all_poly);
    beq_start = zeros(3,1);
   Aeq_start(1:3,1:n_coef) = [calc_tvec(0,n_order,0);
                               calc_tvec(0,n_order,1);
                               calc_tvec(0,n_order,2)];
    beq_start(1:3,1) = [start_cond(1),v0,a0]';    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = [];
    beq_end = [];
    Aeq_end = zeros(3,n_all_poly);
    beq_end = zeros(3,1);
    Aeq_end(1:3,1:n_coef) = [calc_tvec(ts(end),n_order,0);
                             calc_tvec(ts(end),n_order,1);
                             calc_tvec(ts(end),n_order,2)];
    beq_end(1:3,1) = [end_cond(1),ve,ae]';
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = [];
    beq_con_p = [];
    Aeq_con_p = zeros(n_seg-1,n_all_poly);
    beq_con_p = zeros(n_seg-1,1);
    for i=1:n_seg-1
        tvec_p = calc_tvec(ts(i),n_order,0);
        tvec_p_next = calc_tvec(0,n_order,0);
        Aeq_con_p(i,n_coef*(i-1)+1:n_coef*(i+1)) = [tvec_p,-tvec_p_next];
        beq_con_p(i,1) = 0.0;
    end
    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = [];
    beq_con_v = [];
    Aeq_con_v = zeros(n_seg-1,n_all_poly);
    beq_con_v = zeros(n_seg-1,1);
    for i=1:n_seg-1
        tvec_v = calc_tvec(ts(i+1),n_order,1);
        tvec_v_next = calc_tvec(0,n_order,1);
        Aeq_con_v(i,n_coef*(i-1)+1:n_coef*(i+1)) = [tvec_v,-tvec_v_next];
        beq_con_v(i,1) = 0.0;
    end
    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = [];
    beq_con_a = [];
    Aeq_con_a = zeros(n_seg-1,n_all_poly);
    beq_con_a = zeros(n_seg-1,1);
    for i=1:n_seg-1
        tvec_a = calc_tvec(ts(i),n_order,2);
        tvec_a_next = calc_tvec(0,n_order,2);
        Aeq_con_a(i,n_coef*(i-1)+1:n_coef*(i+1)) = [tvec_a,-tvec_a_next];
        beq_con_a(i,1) = 0.0;
    end
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end