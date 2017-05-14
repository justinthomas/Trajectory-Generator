function traj = SimulateTraj(traj)  
% SimulateTraj(traj)

if ~exist('traj', 'var')
    
    dt = 5;
    
    wp0 = ZeroWaypoint(0,4);
    wp0.pos = [10; 10; 2; pi] .* (2*rand(4,1)-1);
    
    wp1 = ZeroWaypoint(dt,4);
    wp1.pos = wp0.pos + [10; 10; 1; pi] .* (2*rand(4,1)-1);
    
    % Determine boundary conditions
    b = [wp0.pos'; wp1.pos'; wp0.vel'; wp1.vel'; wp0.acc'; wp1.acc'; wp0.jerk'; wp1.jerk'; wp0.snap'; wp1.snap'];
    
    n = size(b,1)-1;
    syms t;
    
    % Our system is:
    % A c = b;
    % where c = [...; c3; c2; c1; c0];
    
    A = NaN(n+1);
    %A = sym('a', [n+1, n+1]);
    for idx = 1:size(b,1)
        
        deriv = floor((idx-1) / 2);
        
        % Scale the boundary condition
        b(idx,:) = b(idx,:) * dt^deriv;
        
        % Note, to use polyder, matlab requires that p represents a polynomial:
        % [p0 t^n, p1 t^(n-1), p2 t^(n-2), ..., pn]
        p = ones(1, n+1);
        for idx2 = 1:deriv
            p = polyder(p);
        end
        p = [p .* t.^(length(p)-1:-1:0), zeros(1,deriv)];
        
        % Switch to our convention
        p = fliplr(p);
        
        A(idx,:) = subs(p, mod(idx+1,2));
    end
    
    Ainv = inv(sym(A));
    
    %% Compute dimension coefficients
    c = double(Ainv * b); %#ok<MINV>
    
    % Change to matlab convention
    c = flipud(c);
    
    %% Create traj struct
    clear traj;
    traj.keytimes = [0, dt];
    traj.durations = diff(traj.keytimes);
    traj.poly = NaN(n+1,size(b,2),1,5);
    for dim = 1:size(b,2)
        for deriv = 0:4
            p = c(:,dim);
            for idx = 1:deriv
                p = polyder(p);
            end
            p = [zeros(n+1-length(p),1); p'];
            traj.poly(:,dim,1,deriv+1) = p;
        end
    end
end

PlotTraj(traj);

%% Parameters

m = 0.5; % mass
J = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3]; % inertia

t_step = 0.01;

%% Initial Conditions

to = traj.keytimes(1);
tf = traj.keytimes(end);
tts = 0:0.1:tf;
ntraj = TrajEval(traj, tts);

R0 = Rot(ntraj(1,4,1), 'z');
% s = [x y z xd yd zd R11 R12 R13 R21 R22 R23 R31 R32 R33 Om1 Om2 Om3];
s0 = [ntraj(1,1:3,1)'; zeros(3,1); R0(:); zeros(3,1)];

%% Simulate

options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[ts, ss] = ode45( @(t,s) QuadODE(t, s, m, J, traj), [to, tf], s0, options);

% ts = [];
% ss = [];
% while to < tf
%     
%     % Integrate for this step
%     [ts_step, ss_step] = ode45( @(t,s) QuadODE(t, s, m, J, traj), [to, to + t_step], s0, options);
%     ts = [ts; ts_step];
%     ss = [ss; ss_step];
%     
%     % Reorthonormalize R and load back into the state vector
%     R = reshape(ss_step(end, 7:15), [3,3]);
%     [U, ~, V] = svd(R);
%     R = U*V';
%     assert(abs(det(R) - 1) <= 10*eps, 'det(R) ~= 1');
%     s0 = ss_step(end, :);
%     s0(7:15) = R(:);
%     
%     % Increment to
%     to = to + t_step;
% end

close all
hold on
plot(tts, ntraj(:,1,1), 'b-', ts, ss(:,1), 'bx');
plot(tts, ntraj(:,2,1), 'r-', ts, ss(:,2), 'rx');
plot(tts, ntraj(:,3,1), 'g-', ts, ss(:,3), 'gx');
legend('Desired X', 'Actual X', 'Desired Y', 'Actual Y', 'Desired Z', 'Actual Z');

fprintf('\nFinal Errors:');
ntraj(end,1:3,1) - ss(end,1:3)

PlotControl(m, J, traj);

figure();
quiver3(ntraj(:,1,1), ntraj(:,2,1), ntraj(:,3,1), ntraj(:,1,3), ntraj(:,2,3), ntraj(:,3,3)+9.81);
hold all;
plot3(ntraj(:,1,1), ntraj(:,2,1), ntraj(:,3,1), '.', 'MarkerSize', 30);

end

function PlotControl(m, J, traj)

ts = 0:0.001:traj.keytimes(end);

f = nan(length(ts),1);
M = nan(length(ts),3);

for idx = 1:length(ts)
    [fnew, Mnew] = ComputeControl(m, J, traj, ts(idx));
    f(idx) = fnew;
    M(idx,:) = Mnew';
end

figure();
hold all;
plot(ts, (f - m*9.81)/100, ts, M(:,1), ts, M(:,2), ts, M(:,3));
legend('(thrust - m*g) / 100', 'M1', 'M2', 'M3');

end