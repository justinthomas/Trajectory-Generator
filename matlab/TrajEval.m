function val = TrajEval(traj, t)
% val = TrajEval(traj, t)
% 
% Evaluates a trajectory at the times specified in the vector t
% at all available derivatives
%
% val(tidx, dimension, deriv + 1);
% tidx is the index of the times in t.
% dimension is the dimension, d (x, y, z, psi, ...)
% deriv+1 is the derivative.  For example, to obtain a column vector of the
% positions of the first dimension, you would use val(:,1,1)
% To obtain the velocities, you would use val(:,1,2)

% If we are passed a cell array of trajectory polynomials, 
if iscell(traj)
    val = cell(length(traj), 1);
    for idx = 1:length(traj)
        val{idx} = TrajEval(traj{idx}, t);
    end
    % Concatenate the evaluated trajectories
    val = cat(2, val{:});
    return;
end

poly_size = size(traj.poly);

% The polynomial order
n = poly_size(1) - 1;

% The number of dimensions
d = poly_size(2);

% The number of segments
N = poly_size(3);

% The number of derivatives to evaluate
deriv = poly_size(4) - 1;

% preallocate
val = zeros(deriv + 1, d, length(t));

for idx = 1:length(t)
    
    % Determine the segment
    seg = find(traj.keytimes <= t(idx), 1, 'last');
    
    % This will only affect seg when t = traj.keytimes(end)
    seg = min(seg,N);
    
    seg_dt = traj.durations(seg);
    nondim_t = (t(idx) - traj.keytimes(seg)) / seg_dt;
    
    % Generate our basis
    basis = nondim_t.^(n:-1:0);
    
    % The old way:
    
    % % Make it the right size
    % basis_block = repmat(basis', [1, d, 1, deriv + 1]);
    %
    % % And evaluate the trajectory
    % eval = sum(traj.poly(:,:,seg,:).*basis_block, 1);
    
    % The faster way
    temp = bsxfun(@times, traj.poly(:,:,seg,:), basis');
    eval = sum(temp(:,:,1,:),1);

    %% Now store the derivative values in the rows, the dimensions in the
    % columns, and the time index as the depth
    val(:,:,idx) = permute(eval(1,:,1,:),[4, 2, 3, 1]);
    
    %% Fix time scaling (Note: This could be done before the sum above)    
    val(:,:,idx) = bsxfun(@times, val(:,:,idx), 1./seg_dt.^(0:deriv)');
    
end

% Now store in a more sensible format so that the first dimension
% indexes the time, the second dimension indexes the dim, and the third
% dimension indexes the derivative
% And for some reason, we have to rotate it.
val = permute(val, [3 2 1]);

end
