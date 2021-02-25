function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
n_poly_perseg = n_order+1;
n_all_poly = n_seg*n_poly_perseg;%这里这么写是为把所有段要求的参数串起来

%#####################################################
% p,v,a,j constraint in start,
Aeq_start = zeros(4, n_all_poly);
beq_start = zeros(4, 1);
% STEP 2.1: write expression of Aeq_start and beq_start
%
%
%
%
Aeq_start(1,1:n_poly_perseg) = get_kderivate_constraint(0,n_order,0);
Aeq_start(2,1:n_poly_perseg) = get_kderivate_constraint(0,n_order,1);
Aeq_start(3,1:n_poly_perseg) = get_kderivate_constraint(0,n_order,2);
Aeq_start(4,1:n_poly_perseg) = get_kderivate_constraint(0,n_order,3);
beq_start(1:4,1) = [start_cond(1);start_cond(2);start_cond(3);start_cond(4)];


%#####################################################
% p,v,a constraint in end
Aeq_end = zeros(4, n_all_poly);
beq_end = zeros(4, 1);
% STEP 2.2: write expression of Aeq_end and beq_end
%
%
%
%
Aeq_end(1,(n_seg-1)*n_poly_perseg+1:n_all_poly) = get_kderivate_constraint(ts(end),n_order,0);
Aeq_end(2,(n_seg-1)*n_poly_perseg+1:n_all_poly) = get_kderivate_constraint(ts(end),n_order,1);
Aeq_end(3,(n_seg-1)*n_poly_perseg+1:n_all_poly) = get_kderivate_constraint(ts(end),n_order,2);
Aeq_end(4,(n_seg-1)*n_poly_perseg+1:n_all_poly) = get_kderivate_constraint(ts(end),n_order,3);
beq_end(1:4,1) = [end_cond(1);end_cond(2);end_cond(3);end_cond(4)];


%#####################################################
% position constrain in all middle waypoints
%     waypoints只有一个自由度（位置约束）
Aeq_wp = zeros(n_seg-1, n_all_poly);
beq_wp = zeros(n_seg-1, 1);
% STEP 2.3: write expression of Aeq_wp and beq_wp
%
%
%
%
for i = 1:n_seg-1
    Aeq_wp(i,(i-1)*n_poly_perseg+1:i*n_poly_perseg) = get_kderivate_constraint(ts(i),n_order,0);
    beq_wp(i,1) = waypoints(i+1,1);
end


%#####################################################
% position continuity constrain between each 2 segments
Aeq_con_p = zeros(n_seg-1, n_all_poly);
beq_con_p = zeros(n_seg-1, 1);
% STEP 2.4: write expression of Aeq_con_p and beq_con_p
%
%
%
%
for i = 1:n_seg-1
    Aeq_con_p(i,(i-1)*n_poly_perseg+1:(i+1)*n_poly_perseg) = [get_kderivate_constraint(ts(i),n_order,0),-get_kderivate_constraint(0,n_order,0)];
    beq_con_p(i,1) = 0;
end

%#####################################################
% velocity continuity constrain between each 2 segments
Aeq_con_v = zeros(n_seg-1, n_all_poly);
beq_con_v = zeros(n_seg-1, 1);
% STEP 2.5: write expression of Aeq_con_v and beq_con_v
%
%
%
%
for i = 1:n_seg-1
    Aeq_con_v(i,(i-1)*n_poly_perseg+1:(i+1)*n_poly_perseg) = [get_kderivate_constraint(ts(i),n_order,1),-get_kderivate_constraint(0,n_order,1)];
    beq_con_v(i,1) = 0;
end

%#####################################################
% acceleration continuity constrain between each 2 segments
Aeq_con_a = zeros(n_seg-1, n_all_poly);
beq_con_a = zeros(n_seg-1, 1);
% STEP 2.6: write expression of Aeq_con_a and beq_con_a
%
%
%
%
for i = 1:n_seg-1
    Aeq_con_a(i,(i-1)*n_poly_perseg+1:(i+1)*n_poly_perseg) = [get_kderivate_constraint(ts(i),n_order,2),-get_kderivate_constraint(0,n_order,2)];
    beq_con_a(i,1) = 0;
end


%#####################################################
% jerk continuity constrain between each 2 segments
Aeq_con_j = zeros(n_seg-1, n_all_poly);
beq_con_j = zeros(n_seg-1, 1);
% STEP 2.7: write expression of Aeq_con_j and beq_con_j
%
%
%
%
for i = 1:n_seg-1
    Aeq_con_j(i,(i-1)*n_poly_perseg+1:(i+1)*n_poly_perseg) = [get_kderivate_constraint(ts(i),n_order,3),-get_kderivate_constraint(0,n_order,3)];
    beq_con_j(i,1) = 0;
end

%#####################################################
% combine all components to form Aeq and beq
Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
beq = [beq_start; beq_end; beq_wp; beq_con];
end