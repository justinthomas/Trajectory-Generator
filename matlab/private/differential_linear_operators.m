function D = differential_linear_operators(n)

LinDeriv = zeros(n + 1);

for idx = 1:n
    LinDeriv(:,idx) = [zeros(1,idx) (n - idx + 1) zeros(1,n-idx)]';
end
D = cell(4,1);
D{1} = LinDeriv;
D{2} = LinDeriv*D{1};
D{3} = LinDeriv*D{2};
D{4} = LinDeriv*D{3};
D{5} = LinDeriv*D{4};

end