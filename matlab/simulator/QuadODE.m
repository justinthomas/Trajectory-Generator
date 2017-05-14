function [sd, s] = QuadODE(t, s, m, J, traj)
% sdot = QuadODE(time, s, mass, inertia, trajectory)

% s = [x y z xd yd zd R11 R12 R13 R21 R22 R23 R31 R32 R33 Om1 Om2 Om3];
sd = zeros(length(s),1);

g = 9.81;
e3 = [0; 0; 1];

R = reshape(s(7:15), [3,3]);
[U, ~, V] = svd(R);
R = U*V';
assert(abs(det(R) - 1) <= 100*eps, 'det(R) ~= 1');

[f, M] = ComputeControl(m, J, traj, t);

Omega = s(16:18);

sd(1:3) = s(4:6);
sd(4:6) = f / m * R * e3 - g * e3;
sd(7:15) = R * hat(Omega);
sd(16:18) = J \ (M - cross(Omega, J * Omega));

ProgressBar(t / traj.keytimes(end));

end

function c = cross(a, b)
% function c = cross(a, b)
% This is simply a fast cross product of vectors a and b.  Note, a and b
% can also be of the form a = [a1, a2, ..., an] and b = [b1, b2, ..., bn]
% where a1, a2, ..., an represent column vectors that are crossed with b1,
% b2, ..., bn, respectively.

if size(a,2) > 1
    c = [...
        a(:,2).*b(:,3) - a(:,3).*b(:,2), ...
        a(:,3).*b(:,1) - a(:,1).*b(:,3), ...
        a(:,1).*b(:,2) - a(:,2).*b(:,1)]';
else
    c = [...
        a(2).*b(3) - a(3).*b(2)
        a(3).*b(1) - a(1).*b(3)
        a(1).*b(2) - a(2).*b(1)];
end
end

function ss = hat(vec)
% function ss = hat(vec)
%   The hat function maps a vector in R^3 to its skew symmetric matrix

ss = [...
          0, -vec(3),  vec(2);...
     vec(3),       0, -vec(1);...
    -vec(2),  vec(1),       0;];

end