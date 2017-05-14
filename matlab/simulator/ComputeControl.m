function [f, M, Omega, Omegad, xdd] = ComputeControl(m, J, traj, t)
% [f, M] = ComputeControl(mass, intertia, trajectory, time)

assert(length(t) == 1, 'ComputeControl requires length(time) = 1');

g = 9.81;
e3 = [0; 0; 1];

ntraj = TrajEval(traj, t);

if size(ntraj, 2) == 2
  % Assume we just have X and Z dimensions
  xdd   = [ntraj(1,1,3); 0; ntraj(1,2,3)];
  xddd  = [ntraj(1,1,4); 0; ntraj(1,2,4)];
  xdddd = [ntraj(1,1,5); 0; ntraj(1,2,5)];
  psi   = 0;
  psid  = 0;
  psidd = 0;
else
  xdd   = ntraj(1, 1:3, 3)';
  xddd  = ntraj(1, 1:3, 4)';
  xdddd = ntraj(1, 1:3, 5)';
  psi   = ntraj(1,   4, 1);
  psid  = ntraj(1,   4, 2);
  psidd = ntraj(1,   4, 3);
end

f = m * norm(xdd + g*e3);
b3 = (xdd + g*e3) / norm(xdd + g*e3);

convention = 'ZYX';
switch convention
    
    case 'ZXY'
        
        % Using ZXY (Daniel's convention)
        
        bc = [cos(psi); sin(psi); 0];
        b2 = cross(b3, bc) / norm(cross(b3, bc));
        b1 = cross(b2, b3);
        R = [b1, b2, b3];
        
        assert(abs(det(R) - 1) < 10*eps, 'det(R) ~= 0');
        
        fd = m * b3.' * xddd;
        
        Om1 = m/f * -b2.' * xddd;
        Om2 = m/f *  b1.' * xddd;
        
        % Note: this differs based on the Euler angle convention
        Om3 = ((1 - R(3,2)^2) * psid - R(3,1)*Om1) / R(3,3);
        
        Omega = [Om1; Om2; Om3];
        
        Rd = R * hat(Omega);
        
        Omd1 = m/f * -b2.' * xdddd - 2*fd/f * Om1 + Om3*Om2;
        Omd2 = m/f *  b1.' * xdddd - 2*fd/f * Om2 - Om3*Om1;
        
        % Note: this differs based on the Euler angle convention
        Omd3 = 1/R(3,3)^2 * (Rd(3,3)*(R(3,1)*Om1 + (R(3,2)^2-1)*psid) - R(3,3)*(Om1*Rd(3,1) + 2*R(3,2)*Rd(3,2)*psid + R(3,1)*Omd1 + (R(3,2)^2-1)*psidd));
        
        Omegad = [Omd1; Omd2; Omd3];
        
        M = J*Omegad + cross(Omega, J*Omega);
        
    case 'ZYX'
        
        % Using ZYX

        
        % Current approach
%         b1c = [cos(psi); sin(psi); 0];
%         b2 = cross(b3, b1c) / norm(cross(b3, b1c));
%         b1 = cross(b2, b3);
        
        % JMR Approach
%         bc = [cos(psi); sin(psi); 0];
%         b2 = cross(b3, bc) / norm(cross(b3, bc));
%         b1 = cross(b2, b3);
%         R = [b1, b2, b3];
        
        % New approach
        b2c = [-sin(psi); cos(psi); 0];
        b1 = cross(b2c, b3) / norm(cross(b2c, b3));
        b2 = cross(b3, b1);
        
        R = [b1, b2, b3];
        assert(abs(det(R) - 1) < 10*eps, 'det(R) ~= 0');
        
        fd = m * b3.' * xddd;
        
        Om1 = m/f * -b2.' * xddd;
        Om2 = m/f *  b1.' * xddd;
        Om3 = ((1 - R(3,1)^2) * psid - R(3,2)*Om2) / R(3,3);
        Omega = [Om1; Om2; Om3];
        
        Rd = R * hat(Omega);
        
        Omd1 = m/f * -b2.' * xdddd - 2*fd/f * Om1 + Om3*Om2;
        Omd2 = m/f *  b1.' * xdddd - 2*fd/f * Om2 - Om3*Om1;
        Omd3 = 1/R(3,3)^2 * (Rd(3,3)*(R(3,2)*Om2 + (R(3,1)^2-1)*psid) - R(3,3)*(Om2*Rd(3,2) + 2*R(3,1)*Rd(3,1)*psid + R(3,2)*Omd2 + (R(3,1)^2-1)*psidd));
        
        Omegad = [Omd1; Omd2; Omd3];
        
        M = J*Omegad + cross(Omega, J*Omega);
        
  case 'ZYX_old'
    
        b1c = [cos(psi); sin(psi); 0];
        b2 = cross(b3, b1c) / norm(cross(b3, b1c));
        b1 = cross(b2, b3);
    
    % JMR Approach
    %         bc = [cos(psi); sin(psi); 0];
    %         b2 = cross(b3, bc) / norm(cross(b3, bc));
    %         b1 = cross(b2, b3);
    %         R = [b1, b2, b3];
        
        R = [b1, b2, b3];
        assert(abs(det(R) - 1) < 10*eps, 'det(R) ~= 0');
        
        fd = m * b3.' * xddd;
        
        Om1 = m/f * -b2.' * xddd;
        Om2 = m/f *  b1.' * xddd;
        Om3 = ((1 - R(3,1)^2) * psid - R(3,2)*Om2) / R(3,3);
        Omega = [Om1; Om2; Om3];
        
        Rd = R * hat(Omega);
        
        Omd1 = m/f * -b2.' * xdddd - 2*fd/f * Om1 + Om3*Om2;
        Omd2 = m/f *  b1.' * xdddd - 2*fd/f * Om2 - Om3*Om1;
        Omd3 = 1/R(3,3)^2 * (Rd(3,3)*(R(3,2)*Om2 + (R(3,1)^2-1)*psid) - R(3,3)*(Om2*Rd(3,2) + 2*R(3,1)*Rd(3,1)*psid + R(3,2)*Omd2 + (R(3,1)^2-1)*psidd));
        
        Omegad = [Omd1; Omd2; Omd3];
        
        M = J*Omegad + cross(Omega, J*Omega);
    
end

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