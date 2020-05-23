% r = 0, 1, 2, 3 分别对应 p, v, a, j
% 最高次幂为n_order,自变量为t的时候，每一项幂的r阶导数，与系数向量相乘即为t处的r阶导的值，例如p(t),v(t),a(t),j(t)
function tvec = calc_tvec(t,n_order,r)
    tvec = zeros(1,n_order+1);
    for i=r+1:n_order+1
        tvec(i) = prod(i-r:i-1)*t^(i-r-1);
    end
end