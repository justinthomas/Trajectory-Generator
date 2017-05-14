function val = TrajEval(traj, t, derivatives)
% val = TrajEval(traj, t, derivatives)
% 
% Evaluates a trajectory at the times specified in the vector t
% at the derivatives specified in the derivatives vector
%
% val(tidx, dimension, derivatives_idx);
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
if length(poly_size) > 2
  N = poly_size(3);
else
  N = 1;
end

% The number of derivatives
if nargin < 3
  derivatives = 0;
end


% Make sure the traj has a basis field
if ~isfield(traj, 'basis')
  traj.basis = 'power';
end

if N == 1 && isfield(traj, 'basis') && isfield(traj, 'eval')
  
  % preallocate
  val = zeros(length(t), d, length(derivatives));
  
  for d_idx = 1:length(derivatives)
    val(:,:,d_idx) = traj.eval(t, derivatives(d_idx)).';
%     deriv = derivatives(d_idx);
%     basis = double(traj.basis(t(:) / traj.durations(1), deriv));
%     val(:,:,d_idx) = traj.poly.' * basis / traj.durations(1)^deriv;
  end
  
else
  
  % preallocate
  val = zeros(length(derivatives), d, length(t));
  
  for idx = 1:length(t)
    
    % Determine the segment
    seg = find(traj.keytimes <= t(idx), 1, 'last');
    
    % This will only affect seg when t = traj.keytimes(end)
    seg = min(seg,N);
    
    seg_dt = traj.durations(seg);
    nondim_t = (t(idx) - traj.keytimes(seg)) / seg_dt;
    
    for d_idx = 1:length(derivatives)
      
      deriv = derivatives(d_idx);
      
      switch traj.basis_type
        case 'Legendre'
          b = double(lbasis(nondim_t, deriv, n)).';
        otherwise
          % Generate our basis
          b = nondim_t.^(n:-1:0);
      end
      
      % The faster way
      temp = bsxfun(@times, traj.poly(:,:,seg), b');
      eval = sum(temp(:,:,1,:),1);
      
      %% Now store the derivative values in the rows, the dimensions in the
      % columns, and the time index as the depth
      val(d_idx,:,idx) = permute(eval(1,:,1,:),[4, 2, 3, 1]);
      
      %% Fix time scaling (Note: This could be done before the sum above)
      val(d_idx,:,idx) = bsxfun(@times, val(d_idx,:,idx), 1./seg_dt.^(deriv)');
    end
  end
  
  % Now store in a more sensible format so that the first dimension
  % indexes the time, the second dimension indexes the dim, and the third
  % dimension indexes the derivative
  % And for some reason, we have to rotate it.
  val = permute(val, [3 2 1]);
end


end
