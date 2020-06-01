function coef_list = YangHTriangle(n)
    coef_list = zeros(1,n);
    for i = 1:n
%         nchoosek为组合bai，用法nchoosek(N,M)，代表从N在选M个数进行du组合，N>M。数学定义：N!/(N-M)!/M!
        coef_list(i) = (-1)^(i+n)*nchoosek(n-1,i-1);
    end
end