function [ obstacle_matrix ] = distance_to_obstacles( free_matrix )
%DISTANCE_TO_OBSTACLES Calculates the distance to nearest non-free cell.
%   Runs some sort of distributed bfs for a happy life.
    obstacle_matrix = zeros(size(free_matrix));
    queue = {};
%     queue{1}.row = goal_row;
%     queue{1}.col = goal_col;
%     queue{1}.dist = 1;

    % Initial fill of queue
    for i = find(free_matrix == 1)'
        [row, col] = ind2sub(size(free_matrix), i);
        for r = -1:1
            for c = -1:1
                if (r == 0 && c == 0)
                    continue;
                end
                
                if (~free_matrix(row + r, col + c))
                    node = {};
                    node.row = row;
                    node.col = col;
                    node.dist = 1;
                    obstacle_matrix(row, col) = 1;
                    queue{end+1} = node;
                    break;
                end
            end
        end
    end
    
    % Main loop
    while (size(queue, 2)) 
        
        pos = queue{1};
        queue(1) = [];
        
        % Add adjacency coords to queue.
        for r = -1:1
            for c = -1:1
                if (r == 0 && c == 0)
                    continue;
                end
                
                new_queue = {};
                new_queue.row = pos.row + r;
                new_queue.col = pos.col + c;
                new_queue.dist = pos.dist + 1;
                
                if (~free_matrix(new_queue.row, new_queue.col) || obstacle_matrix(new_queue.row, new_queue.col))
                   continue;
                end
                
                queue{end + 1} = new_queue;
                obstacle_matrix(new_queue.row, new_queue.col) = new_queue.dist;
            end
        end
    end

end

