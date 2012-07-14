function vec = basis(t, deriv, n, D)
% This function generates a basis vector of order n at times t.  It has a
% dimension of n+1 by 1 and takes the form [t^n; t^(n-1); ... t; 1];

if nargin < 4
    D = differential_linear_operators(n);
end

if isequal(deriv, 0)
    % If there is no derivative, then the coefficients are all unity
    coeffs = ones(1,n+1);
else
    % We can simply extract the coefficients from our differential operator
    coeffs = sum(D{deriv});
end

% We have to enforce that the powers are >= 0 because if t = 0, t^(-1) is
% undefined.
powers = max(0,(n:-1:0)-deriv);

% Our vector is simply
vec = coeffs.*(t.^powers);

end