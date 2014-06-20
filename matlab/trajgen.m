function [traj, durations, problem, exitflag] = trajgen(waypoints, options, bounds)
% function traj = trajgen(waypoints, options, bounds, varargin)
%
% options is a cell array formatted {'parameter1', value1, 'parameter2', value2, ...}
%
% Available parameters for the options cell array:
%   order (integer)
%       Defines the order of polynomials to use
%   minderiv (vector)
%       Defines which derivative to minimize for each dimension
%   constraints_per_seg (integer)
%       The number of constraints to place for a bound over each segment
%   numerical (boolean)
%       Use the numerical optimization?  Alternative is analytical, but
%       can only be used if there are no bounds.
%   convergetol (double)
%       The tolerance between the primal and dual costs before calling the
%       solution optimal.
%   contderiv (vector)
%       Similar to minderiv, but allows for a different level of required
%       continunity constraints.  For example, [3, 3], would mean that up
%       through the 2nd derivative must be continuous for both dimensions.
%
% The output traj contains a field poly such that
% The first dimension indexes the polynomial coefficients
% The second dimension indexes the dimension of the system (e.g. x, y, z, psi, ...)
% The third dimension indexes the segment
% The fourth dimension indexes the derivative.  
%
% So, traj.poly(:, b, c, d) will return the polynomial defining the b^th
% dimension, the c^th segment, and the (d-1)^th derivative.
%
% A more detailed explanation of this program will go here

warning off MATLAB:nearlySingularMatrix
ticker = tic; %#ok<NASGU>

% Initalize return variables
traj = [];
problem = [];

%% Defaults

if nargin < 3 || isempty(bounds)
    % Use the analytical solver
    bounds = [];
    numerical = false;
else
    % Use a numerical solver
    numerical = true;
end

n = 12;      % Polynomial order
constraints_per_seg = 2*(n+1);   % Number of inequality constraints to enforce per segment
convergetol = 1e-08;    % Tolernce

%% Process varargin
for idx = 1:2:length(options)
    
    switch options{idx}
        case 'order'
            n = options{idx+1};
        case 'minderiv'
            % What derivative are we minimizing
            % minderiv = 4 corresponds to snap
            % minderiv = 2 corresponds to acceleration
            % minderiv = 0 corresponds to position
            minderiv = max(0,options{idx+1});
            
            % The first derivative which can be discontinuous
            if ~exist('contderiv', 'var')
                contderiv = minderiv;
            end
            
            if max(minderiv) > 4; warning('This program can only support up to 4 derivatives at this time.'); end; %#ok<WNTAG>
            
            % We can also determine the number of dimensions
            d = length(minderiv);
            
        case 'constraints_per_seg' % #### This should be a shorter argument...
            constraints_per_seg = options{idx+1};
        case 'numerical'
            numerical = options{idx+1};
        case 'convergetol'
            convergetol = options{idx+1};
        case 'contderiv'
            contderiv = max(0,options{idx+1});
    end
end

%% Keytimes, Segments

% We have one less segment than we have waypoints
N = size(waypoints,2) - 1;

keytimes = [waypoints.time]; % Keytimes
durations = diff(keytimes);

%% Generate our linear differential operator

D = differential_linear_operators(n);

%% Equality constraints

% This will make sure that all the waypoints are column vectors
for idx = 1:length(waypoints)
    waypoints(idx).pos = waypoints(idx).pos(:);
    waypoints(idx).vel = waypoints(idx).vel(:);
    waypoints(idx).acc = waypoints(idx).acc(:);
    waypoints(idx).jerk = waypoints(idx).jerk(:);
end

% Determine the size of E for preallocation.  There will be a row for every
% non-NaN and non-empty constraint.  This will also generate an error if
% the dimensions of the waypoints are not consistent
% (i.e. d is the not the same size for all waypoints)
nrows = sum(sum(~isnan([...
    [waypoints.pos],...
    [waypoints.vel],...
    [waypoints.acc],...
    [waypoints.jerk]])));

% Preallocate E and Ebeq
E = zeros(nrows,d*N*(n+1));
Ebeq = zeros(nrows,1);

% Initalize a row index for E and Ebeq
row = 1;

% And now populate E with the appropriate bases
for pt = 1:N+1
    % The matrix E will have 1 row for every constraint.
    % Additionally, it will have d*N*(n+1) columns.
    % Before each basis group (bgroup) in each row, there will be
    % ((idx - 1) + d*(seg-1))*(n+1) zeros where idx indexes the output (d)
    % and seg indexes the polynomial segment (N).  Then, we know the basis
    % will occupy (n+1) columns.  Finally, the resulting number of zeros is
    % simply the difference of the total columns in the row and the already
    % occupied columns.
    
    % We need to make the last waypoint fall on the end of the last
    % segment, not the beginning of the next segment.
    bgroup = min(pt,N);
    
    % Determine the time duration of the segment
    dt = durations(bgroup);
    
    % We want to scale the time so that each segment is parametrized by t = 0:1.
    % Then, at the beginning of each segment, t = 0, and at the end, t = 1.
    t = pt - bgroup;
    
    % Now, establish the constraints
    if ~isempty(waypoints(pt).pos)
        deriv = 0;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).pos(idx)) && (deriv < contderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).pos(idx);
                row = row+1;
            end
        end
    end
    
    if ~isempty(waypoints(pt).vel)
        deriv = 1;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).vel(idx)) && (deriv < contderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).vel(idx)*dt;
                row = row+1;
            end
        end
    end
    
    if ~isempty(waypoints(pt).acc)
        deriv = 2;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).acc(idx)) && (deriv < contderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).acc(idx)*dt^2;
                row = row+1;
            end
        end
    end
    
    if ~isempty(waypoints(pt).jerk)
        deriv = 3;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).jerk(idx)) && (deriv < contderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).jerk(idx)*dt^3;
                row = row+1;
            end
        end
    end
end

%% Continuity constraints

% There will be the same number of columns as in the matrix E, but now
% there will be a continunity constraint for each output that has a
% derivative below the one we are minimizing.

% There will be a continunity constraint (except for the first and last points)
% for each output and its derivatives below the one being minimized.
nrows = sum((N-1)*contderiv);
C = zeros(nrows,d*N*(n+1));

% This will be zeros since we have
% basis' * coeffs1 - basis' * coeffs2 = 0
Cbeq = zeros(nrows,1);

% Initalize our row counter
row = 1;

% And now populate C with the appropriate bases
for pt = 2:N
    % The matrix C will have 1 row for every constraint.
    % Additionally, it will have d*N*(n+1) columns.
    % Before each basis group (bgroup) in each row, there will be
    % ((idx - 1) + d*(pt-1))*(n+1) zeros where idx indexes the output (d)
    % and pt indexes the polynomial segment (N).  Finally, we know the basis
    % will occupy (n+1) columns.
    
    % The first group will be one less than the point.  For example, for
    % the continunity constraints at waypoint 2, we will require
    % equivalency between groups 1 and 2.
    bgroup = pt - 1;
    
    % Extract the durations of the two segments
    dt1 = durations(bgroup);
    dt2 = durations(bgroup + 1);
    
    % Loop through the derivatives
    for deriv = 0:(max(contderiv)-1)
        
        % Determine our bases at this timestep and for this derivative
        % basis1 corresponds to the end of the first segment and basis0
        % corresponds to the beginning of the next segment
        basis1 = basis(1, deriv, n, D);
        basis0 = basis(0, deriv, n, D);
        
        % Since we can't scale our constraints, we need to scale our bases
        % since they are on different timescales
        basis1 = basis1./(dt1^(deriv));
        basis0 = basis0./(dt2^(deriv));
        
        % Now loop through the dimensions
        for idx = 1:d
            
            % We don't want to impose constraints on derivatives higher
            % than or equal to what we are minimizing
            if deriv < contderiv(idx)
                
                % The first basis group starts here
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                C(row,startidx:startidx+n) = basis1;
                
                % The second basis group starts (n+1)*d columns later
                startidx = startidx + (n+1)*d;
                C(row,startidx:startidx+n) = -basis0; % Note the negative sign
                
                % Advance to the next row
                row = row+1;
            end
        end
    end
end

%% The H Matrix

% Initalize H
H = zeros(N*d*(n+1));

% Determine the powers
powers = (n:-1:0)';

% Determine the Base Hpow matrix (this does not need to be recalculated for
% each flat output and segment)
Hpow_base = repmat(powers,1,n+1)+repmat(powers',n+1,1);

% Initalize our row indexes
rows = 1:(n+1);

% Loop through the segments
for seg = 1:N
    
    % Loop through the dimensions
    for dim = 1:d
        
        % Generate a matrix which represents the powers of H
        Hpow = Hpow_base-2*minderiv(dim);

        % Determine the coefficients
        if ~isequal(minderiv(dim),0)
            Hcoeffs = (sum(D{minderiv(dim)}).'); %./(durations(seg).^minderiv(dim))
            Hcoeffs = Hcoeffs*Hcoeffs';
        else
            Hcoeffs = ones(n+1);
        end
        
        % Now integrate
        Hpow(Hpow >= 0) = Hpow(Hpow >= 0) + 1;
        Hcoeffs(Hpow > 0) = Hcoeffs(Hpow > 0)./Hpow(Hpow > 0);
        
        % Things with negative powers are actually zero.
        Hcoeffs(Hpow < 0) = 0;
        Hpow(Hpow < 0) = 0;
        
        % And store this block in H
        H(rows,rows) = Hcoeffs; %.*(durations(seg).^Hpow);
        
        % Now increment to the next diagonal block
        rows = rows + (n+1);

    end
end

%% Inequality constraints
% A*x <= b

% Determine number of rows for preallocation and determine our times.  We
% will also split up bounds by the segment.  For example, a bound that
% lasts the entire duration will be split up into N bounds so that they can
% be more easily processed later.
nrows = 0;
idx = 0;

while (1)
    idx = idx + 1;
    
    % If we have exceeded the length of bounds, then exit this loop
    if idx > length(bounds)
        break;
    end
    
    % Let's work with a nicer variable
    t = bounds(idx).time;
    
    % If the time is empty, then apply the bound for the entire duration
    if isempty(t)
        t = keytimes([1 end]);
    end
    
    % Determine the segments which the bound starts and finishes in
    start_seg = find(keytimes <= t(1), 1, 'last');
    end_seg = find(keytimes < t(2), 1, 'last');
    
    % If the bound spans more than one segment, split it up
    if ~isequal(start_seg, end_seg)
        
        % Copy the current constraint to the end of our bound array
        bounds(end+1) = bounds(idx); %#ok<AGROW>
        
        % Remove the current segment from the time span
        bounds(end).time = [keytimes(start_seg+1) t(2)];
        
        % Only consider the current segment for the current bound
        t = [t(1) keytimes(start_seg + 1)];
        
    end
    
    % Now generate the times at which this constraint will be applied
    tstep = (keytimes(start_seg+1) - keytimes(start_seg)) / constraints_per_seg;
    t = t(1):tstep:t(2);
    
    % And store it back in bounds
    bounds(idx).time = t;
    
    % And store the segment which contains this bound
    bounds(idx).seg = start_seg;
    
    % The number of dimensions bounded by this bound over this duration
    if ~isequal(length(bounds(idx).arg),d)
        warning('You must specify NaNs for a dimension even if you are not using it in your bound'); %#ok<WNTAG>
    end
    ndim = sum(~isnan(bounds(idx).arg));
    
    % Keep track of the number of rows we will need
    nrows = nrows + length(t)*ndim;
end

% The powers which we divide the basis by to scale it to the full scale
tpow = (n:-1:0);

% Initalize Aineq and bineq
Aineq = zeros(nrows,N*(n+1)*d);
bineq = zeros(nrows,1);

% Initalize our row counter
rows = 0;

% Loop through the bounds
for bidx = 1:length(bounds)

%     if isequal(bounds(bidx).deriv,1); keyboard; end;
    
    % The times will be
    t = bounds(bidx).time - waypoints(bounds(bidx).seg).time;

    % Now, bounds(bidx).time is a vector of times when we wish to enforce
    % the constraint.  So, we willl generate a basis for each time and use
    % the basis block where we need it.  It will have dimensions of
    % length(t) by (n+1) where t is the time vector for this bound
    basis_block = basis(t, bounds(bidx).deriv, n, D);

    % Now scale it so the constraints are in the correct space (not
    % nondimensionalized)
    basis_block = basis_block./repmat(durations(bounds(bidx).seg).^tpow,[length(t), 1]);
    
    % idx indexes d, bgroup indexes the segment
    bgroup = bounds(bidx).seg;
    
    switch bounds(bidx).type
        
        case {'lb', 'ub'}
            
            % Loop through the dimensions
            for idx = 1:d

                % Only impose non-NaN constraints
                if ~isnan(bounds(bidx).arg(idx))
                    
                    % Determine the rows which our basis_block will occupy
                    rows = (rows(end) + 1):(rows(end) + size(basis_block,1));
                    
                    % Determine the column which the basis block will start
                    startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                    
                    % Determine the sign of the bound
                    if isequal(bounds(bidx).type, 'ub')
                        s = 1;
                    elseif isequal(bounds(bidx).type, 'lb')
                        s = -1;
                    end
                    
                    % The basis block will end n columns later
                    Aineq(rows, startidx:(startidx + n)) = s*basis_block;
                    
                    % And save the bound in bineq
                    bineq(rows) = s*bounds(bidx).arg(idx);
                end
            end
            
        case '1norm'
            % This is in progress, but I don't wan't to waste too much time
            % on this right now
            
%             % NaN constraints also imply 0 weighting
%             bounds(bidx).arg(isnan(bounds(bidx).arg)) = 0;
%             
%             % Which are the non-zero weightings?
%             d_logical_idx = ~isequal(0,bounds(bidx).arg);
%             nonzero_d = sum(d_logical_idx);
%             
%             % Determine the rows which our basis_block will occupy
%             rows = (rows(end) + 1):(rows(end) + (2^nonzero_d)*size(basis_block,1));
%             
%             % Determine the column which the basis blocks will start
%             idx = 1;
%             startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
%             
%             % Determine the sign of the bound
%             if isequal(bounds(bidx).type, 'ub')
%                 s = 1;
%             elseif isequal(bounds(bidx).type, 'lb')
%                 s = -1;
%             end
%             
%             % The basis block will end n columns later
%             Aineq(rows, startidx:(startidx + n)) = s*basis_block;
%             
%             % And save the bound in bineq
%             bineq(rows) = s*bounds(bidx).arg(idx);
            
        case 'infnorm'
            
    end
end
%% Construct the problem

problem.H = H;
problem.Aeq = [E; C];
problem.beq = [Ebeq; Cbeq];
problem.Aineq = Aineq;
problem.bineq = bineq;

%% Determine the solution

% min x'*H*x  subject to:  A*x <= b and Aeq*x = beq
if ~numerical
    
    temp = [2*problem.H, problem.Aeq';...
        problem.Aeq, zeros(size(problem.Aeq,1)')];
    
    % Analytic Solution
    x = temp\[zeros(size(temp,1)-length(problem.beq),1); problem.beq];
    
    % Now, extract the coefficients
    x = x(1:size(problem.H));
    
    fprintf('Solved analytically\n');
    
else
    
    % If we have the cplex solvers in our path, use them.  Otherwise, we
    % will default to MATLAB's optimization toolbox and use quadprog.
    if exist('cplexqp.p', 'file')
        
        problem.f = zeros(size(H,2),1);
        problem.options = cplexoptimset('cplex');
        % If primal obj and dual obj are this close, the solution is considered optimal
        problem.options.barrier.convergetol = convergetol;

        ticker2 = tic;
        [x, fval, exitflag, output] = cplexqp(problem); %#ok<NASGU,ASGLU>
        temp = toc(ticker2);
        fprintf('CPLEX solve time: %2.3f seconds\n', temp);
        
    else
        
        % Set up the problem
        problem.options = optimset('MaxIter',1500,'Display','on','Algorithm','active-set');
        problem.solver = 'quadprog';

        % Numerical Solution
        ticker2 = tic;
        [x, fval, exitflag, output] = quadprog(problem); %#ok<NASGU,ASGLU>
        temp = toc(ticker2);
        fprintf('QuadProg solve time: %2.3f seconds\n', temp);
    end
    
end

%% Package the solution
% The first dimension will index the polynomial coefficients
% The second dimension will index the dimension (e.g. x, y, z, psi, ...)
% The third dimension will index the segment
% The fourth dimension will index the derivative.  
% So, traj.poly(:, b, c, d) will return the polynomial defining the b^th
% dimension, the c^th segment, and the (d-1)^th derivative.
traj.poly = zeros(n+1, d, N);
traj.poly(:) = x;

traj.durations = durations;
traj.keytimes = keytimes;

%% Unnormalize the coefficients

% To scale the bases, we will need the powers of the segment
% duration
tpow = (n:-1:0)';

for seg = 1:N
    
    % Renormalize the segments
    t = durations(seg);
    traj.poly(:,:,seg) = traj.poly(:,:,seg)./repmat((t.^tpow),[1 d]);
    
    % Differentiate the polynomials
    for deriv = 1:max(minderiv)
        traj.poly(:,:,seg,deriv+1) = D{deriv}*traj.poly(:,:,seg, 1);
    end
    
end

end