function [traj problem] = trajgen(waypoints, options, bounds, varargin)
% function traj = trajgen(waypoints, options, bounds, varargin)
%
% A more detailed explanation of this program will go here

warning off MATLAB:nearlySingularMatrix
ticker = tic; %#ok<NASGU>

%% Defaults

numerical = true;            % Use quadprog
n = 10;      % Polynomial order
constraints_per_seg = 10;   % Number of inequality constraints to enforce per segment

%% Process varargin
for idx = 1:2:length(options)
    
    switch options{idx}
        case 'polyorder'
            n = options{idx+1};
        case 'minderiv'
            % What derivative are we minimizing
            % minderiv = 4 corresponds to snap
            % minderiv = 2 corresponds to acceleration
            % minderiv = 0 corresponds to position
            minderiv = max(0,options{idx+1});
            if max(minderiv)>4; warning('This program can only support up to 4 derivatives at this time.'); end; %#ok<WNTAG>
        case 'ndim'
            % The number of dimensions we have.
            % For example, for a typical quadrotor, we have 4: x,y,z,psi
            d = options{idx+1};
            if d <= 0;  warning('Do you really want a <= 0 dimensional system?'); end %#ok<WNTAG>
    end
end


%% Keytimes, Segments

% We have one less segment than we have waypoints
N = size(waypoints,2) - 1;

keytimes = [waypoints.time]; % Keytimes

%% Generate our linear differential operator

D = differential_linear_operators(n);

%% Equality constraints

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
    
    % Extract the time
    t = waypoints(pt).time;
    
    % We need to make the last waypoint fall on the end of the last
    % segment, not the beginning of the next segment.
    bgroup = min(pt,N);
    
    % Now, establish the constraints
    if ~isempty(waypoints(pt).pos)
        deriv = 0;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).pos(idx)) && (deriv < minderiv(idx))
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
            if ~isnan(waypoints(pt).vel(idx)) && (deriv < minderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).vel(idx);
                row = row+1;
            end
        end
    end
    
    if ~isempty(waypoints(pt).acc)
        deriv = 2;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).acc(idx)) && (deriv < minderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).acc(idx);
                row = row+1;
            end
        end
    end
    
    if ~isempty(waypoints(pt).jerk)
        deriv = 3;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).jerk(idx)) && (deriv < minderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).jerk(idx);
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
nrows = sum((N-1)*minderiv);
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
    
    % Extract the time at this particular waypoint
    t = waypoints(pt).time;
    
    % Loop through the derivatives
    for deriv = 0:max(minderiv)
        
        % Determine our basis at this timestep and for this derivative
        temp = basis(t,deriv,n,D);
        
        % Now loop through the dimensions
        for idx = 1:d
            
            % We don't want to impose constraints on derivatives higher
            % than or equal to what we are minimizing
            if deriv < minderiv(idx)
                
                % The first basis group starts here
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                C(row,startidx:startidx+n) = temp;
                
                % The second basis group starts (n+1)*d columns later
                startidx = startidx + (n+1)*d;
                C(row,startidx:startidx+n) = -temp; % Note the negative sign
                
                % Advance to the next row
                row = row+1;
            end
        end
    end
end

%% The H Matrix

% Initalize H
H = zeros((N)*d*(n+1));

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
    for idx = 1:d
        
        % Generate a matrix which represents the powers of H
        Hpow = Hpow_base-2*minderiv(idx);

        % Determine the coefficients
        if ~isequal(minderiv(idx),0)
            Hcoeffs = sum(D{minderiv(idx)})';
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
        H(rows,rows) = Hcoeffs.*(keytimes(seg+1).^Hpow) - Hcoeffs.*(keytimes(seg).^Hpow);
        
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

% Initalize Aineq and bineq
Aineq = zeros(nrows,N*(n+1)*d);
bineq = zeros(nrows,1);

% Initalize our row counter
rows = 0;

% Loop through the bounds
for bidx = 1:length(bounds)

    % Now, bounds(bidx).time is a vector of times when we wish to enforce
    % the constraint.  So, we willl generate a basis for each time and use
    % the basis block where we need it.  It will have dimensions of
    % length(t) by (n+1) where t is the time vector for this bound
    basis_block = basis(bounds(bidx).time, bounds(bidx).deriv, n, D);
    
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
            
        case '2norm'
            
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
    out = temp\[zeros(n+1,1); problem.beq];
    if any(isnan(out)); keyboard; end;  %#### Remove this at some point!
%     coeffs{flat_out,seg} = out(1:n+1);
    
else
    % Set up the problem
    problem.options = optimset('MaxIter',1000,'Display','on');
    problem.solver = 'quadprog';

    % Numerical Solution
    ticker2 = tic;
    [x, fval, exitflag] = quadprog(problem);
    toc(ticker2)
    
    switch exitflag
        case 1
            fprintf('Solution found\n');
        case 0
            fprintf('Exceeded options.MaxIter\n');
        case -2
            fprintf('Problem is infeasible\n');
        case -3
            fprintf('Problem is unbounded\n');
        case 4
            fprintf('Local minimizer was found\n');
        case -7
            fprintf('Magnitude of search direction became too small\n');
    end
end

% How much time has elapsed?
% toc(ticker)

% Just return coeffs_vec for now
traj = x;

end