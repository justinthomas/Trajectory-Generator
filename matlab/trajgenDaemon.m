function traj = trajgenDaemon(filename)

while 1

    % Check once a second to see if the file has been created
    if ~exist(filename, 'file') 
        pause(1)
    else
        % Load the mat file containing the trajectory arguments
        load(filename);
        
        % Plan the trajectory
        traj = trajgen(waypoints, options, bounds);
        traj.end_time = waypoints(end).time; %#ok<COLND>
        
        % Locate the specific file that we just used
        filename = which(filename);
        pathstr = fileparts(filename);
        
        % Delete the file used
        delete(filename)
        
        % Save the trajectory to the same path
        save([pathstr, '/traj.mat'], 'traj')
    end
    
end

end