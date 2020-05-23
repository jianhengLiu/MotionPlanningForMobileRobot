function constraintVector = get_kderivate_constraint(T, n_order, k)
constraintVector = zeros(1,n_order+1);

for i = k+1:n_order+1
    constraintVector(i) = factorial(i-1)/factorial(i-k-1)*T^(i-k-1);
end
end