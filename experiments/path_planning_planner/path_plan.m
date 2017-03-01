function [ path ] = path_plan( distances )
%PATH_PLAN Summary of this function goes here
%   path is a matrix of two column vectors [x y]
    %path = zeros(numel(find(distances > 0)), 2);
    
    path = [];
    visited = zeros(size(distances));
    
    %goal = find(distances == 1, 1);
    
    % Set goal to position with highest value
    %start = find(distances == max(max(distances)), 1);
    start = 14334; % hardcoded position closer to robot start pos in stage.
    
    [start_row, start_col] = ind2sub(size(distances), start);
    [current_row, current_col] = ind2sub(size(distances), start);
    
    % Insert current row in path
    path(1, 1) = current_row;
    path(1, 2) = current_col;
    visited(current_row, current_col) = 1;
    
    while 1
        node = find_max(visited, distances, current_row, current_col);
        
        if node.val == -inf
            % Backtrace until we find a free node
            
            new_node = {};
            new_node.row = current_row;
            new_node.col = current_col;
            found = false;
            for i = size(path, 1):-1:1
                back_node = find_max(visited, distances, path(i, 1), path(i, 2));
                
                if (back_node.val ~= -inf)
                    % FIXME: Insert path in node???!??!
                    node = back_node;
                    found = true;
                    break;
                end
            end
            
            if (~found)
                break;
            end
        end
        
        current_row = node.row;
        current_col = node.col;
        
        path(end + 1, 1) = current_row;
        path(end, 2) = current_col;
        visited(current_row, current_col) = 1;
        if (numel(path) > 20000) 
            break;
        end
    end
end

function [node] = find_max(visited, distances, current_row, current_col)
    node = {};
    node.row = inf;
    node.col = inf;
    node.val = -inf;
    
    % Silly loop to cover all movements, don't hate me. :(
    for r = -1:1
        for c = -1:1
            if ((r == 0 && c == 0) || visited(current_row + r, current_col + c))
                continue;
            end
            val = distances(current_row + r, current_col + c);
            if (val >= node.val)
                node.row = current_row + r;
                node.col = current_col + c;
                node.val = val;
            end
        end
    end
end

