% r = 0, 1, 2, 3 �ֱ��Ӧ p, v, a, j
% ��ߴ���Ϊn_order,�Ա���Ϊt��ʱ��ÿһ���ݵ�r�׵�������ϵ��������˼�Ϊt����r�׵���ֵ������p(t),v(t),a(t),j(t)
function tvec = calc_tvec(t,n_order,r)
    tvec = zeros(1,n_order+1);
    for i=r+1:n_order+1
        tvec(i) = prod(i-r:i-1)*t^(i-r-1);
    end
end