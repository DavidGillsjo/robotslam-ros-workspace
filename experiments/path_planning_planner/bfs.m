function [ distances ] = bfs( goal, free_matrix )
%BFS Performs bfs and returns matrix with distance from goal
    
    distances = zeros(size(free_matrix));
    
    [goal_row, goal_col] = ind2sub(size(free_matrix), goal);
    
    queue = {{}};
    queue{1}.row = goal_row;
    queue{1}.col = goal_col;
    queue{1}.dist = 1;
    
    distances(goal_row, goal_col) = 1;
    
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
                
                if (~free_matrix(new_queue.row, new_queue.col) || distances(new_queue.row, new_queue.col))
                   continue;
                end
                
                queue{end + 1} = new_queue;
                distances(new_queue.row, new_queue.col) = new_queue.dist;
            end
        end
    end
end

