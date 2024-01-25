% Parameters
R = 100; % Circle's radius
pd = 5;  % Point spacing

% Generate points within the circle
[x, y] = meshgrid(-R:pd:R, -R:pd:R);
inside_circle = x.^2 + y.^2 <= R^2;
x = x(inside_circle);
y = y(inside_circle);

% Plot the points
figure;
scatter(x, y, 'k.');
axis square;
hold on;
viscircles([0,0], R, 'Color', 'b'); % Draw the circle

% Initialize visited array and path
visited = zeros(size(x));
path = [];

% Find a central point to start
[~, center_idx] = min(sqrt(x.^2 + y.^2)); % Prefer proximity to the center
current_point = [x(center_idx), y(center_idx)];
path = [current_point];
visited(center_idx) = 1;

% Directions for movement
directions = [-pd, 0; pd, 0; 0, -pd; 0, pd; -pd, -pd; pd, pd; pd, -pd; -pd, pd];

% Move to adjacent points in one of the eight possible directions
while any(visited == 0)
    % Find available neighboring points
    neighbors = find_available_neighbors(current_point, x, y, visited, pd);
    
    % Choose a random available direction
    if ~isempty(neighbors)
        direction_idx = randi(size(neighbors, 1));
        current_point = neighbors(direction_idx, :);
        current_idx = find(x == current_point(1) & y == current_point(2));
        visited(current_idx) = 1;
        path = [path; current_point];
    else
        % If dead zone, perform local turnaround
        [path, visited] = local_turnaround(path, x, y, visited, pd);
        if isempty(path)
            break; % If no path is found, end the loop
        end
        current_point = path(end, :); % Update the current point
    end
end

% Draw the path
plot(path(:,1), path(:,2), 'r-');
hold off;

function neighbors = find_available_neighbors(current_point, x, y, visited, pd)
    neighbors = [];
    directions = [-pd, 0; pd, 0; 0, -pd; 0, pd; -pd, -pd; pd, pd; pd, -pd; -pd, pd];
    for i = 1:size(directions, 1)
        next_point = current_point + directions(i, :);
        next_idx = find(x == next_point(1) & y == next_point(2), 1);
        if ~isempty(next_idx) && visited(next_idx) == 0
            neighbors = [neighbors; next_point];
        end
    end
end

function [path, visited] = local_turnaround(path, x, y, visited, pd)
    for i = size(path, 1):-1:1
        current_point = path(i, :);
        neighbors = find_available_neighbors(current_point, x, y, visited, pd);
        if ~isempty(neighbors)
            % Disconnect at current point and reverse the path after the breakpoint
            new_path = path(1:i-1, :);
            reversed_path = flipud(path(i:end, :));
            % Reconnect the paths in order
            path = [new_path; reversed_path];
            % Update the visited points
            for j = 1:size(reversed_path, 1)
                idx = find(x == reversed_path(j, 1) & y == reversed_path(j, 2));
                visited(idx) = 1;
            end
            return
        end
    end
    % If no unvisited neighbors found, return empty path
    path = [];
end

