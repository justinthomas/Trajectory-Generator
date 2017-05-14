%function EulerConventions()
% EulerConventions
%
% Note: this depends on MATLAB-tools from
% https://github.com/justinthomas/MATLAB-tools

clc

%% Useful Shorthands

e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];

%% Analysis

% syms phi theta real;

psi = 0;%pi() * (2*rand(1) - 1);

% R_ZXY = Rot(psi, 'Z') * Rot(phi, 'X') * Rot(theta, 'Y'); % Daniel
% R_ZYX = Rot(psi, 'Z') * Rot(theta, 'Y') * Rot(phi, 'X'); % Ours?

% Choose a random b3 vector
b3 = 2*rand(3,1) - 1;
b3 = - b3 * sign(b3(3)); % Ensure b3.' * e3 > 0
b3 = b3 / norm(b3);    % Ensure unit vector

% We can show that this works out symbolically for ZXY
b1c = Rot(psi, 'Z') * e1;
b2 = cross(b3, b1c) / norm(cross(b3, b1c));
b1 = cross(b2, b3);
R_ZXY = [b1, b2, b3];

% Now, convert the rotation back to euler angles
[phi_ZXY, theta_ZXY, psi_ZXY] = RotToEulZXY(R_ZXY);
disp('ZXY');
[phi_ZXY, theta_ZXY, psi_ZXY] %#ok<NOPRT>
R_ZXY

%%
b1c = Rot(psi, 'Z') * e1;
b2c = Rot(psi, 'Z') * e2;
b1 = cross(b2c, b3) / norm(cross(b2c, b3));

if b1c.' * b1 < 0
    b2c = Rot(psi + pi, 'Z') * e2;
    b1 = cross(b2c, b3) / norm(cross(b2c, b3));
end
b2 = cross(b3, b1);

R_ZYX = [b1, b2, b3];
[phi_ZYX, theta_ZYX, psi_ZYX] = RotToEulZYX(R_ZYX);
disp('ZYX');
[phi_ZYX, theta_ZYX, psi_ZYX] %#ok<NOPRT>
R_ZYX

% end