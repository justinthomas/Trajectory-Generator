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

% If t is a vector, we need to reshape things so that each time generates
% a basis row.  This will create a 2-d matrix;
if numel(t) > 1

    % If t is a vector, then we want to generate a basis vector for each
    % element of t
    if size(t,1) > 1
        t = repmat(t,[1 n+1]);
    else
        t = repmat(t',[1 n+1]);
    end
    
    powers = repmat(powers, [size(t,1), 1]);
    coeffs = repmat(coeffs, [size(t,1),1]);
end

vec = coeffs.*(t.^powers);

end
