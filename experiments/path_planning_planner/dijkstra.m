function [ distances ] = dijkstra( goal, free_matrix, cost_matrix )
%BFS Performs dijkstra and returns matrix with distance from goal
    
    distances = zeros(size(free_matrix));
    
    [goal_row, goal_col] = ind2sub(size(free_matrix), goal);
    
    queue = {{}};
    queue{1}.row = goal_row;
    queue{1}.col = goal_col;
    queue{1}.cost = 1;
    
    distances(goal_row, goal_col) = 1;
    
    while (size(queue, 2)) 
        min_i = min_cost(queue);
        pos = queue{min_i};
        queue(min_i) = [];
        
        % Add adjacency coords to queue.
        for r = -1:1
            for c = -1:1
                if (r == 0 && c == 0)
                    continue;
                end
                
                new_queue = {};
                new_queue.row = pos.row + r;
                new_queue.col = pos.col + c;
                new_queue.cost = pos.cost + 1 + cost_matrix(new_queue.row, new_queue.col);
                
                if (~free_matrix(new_queue.row, new_queue.col) || distances(new_queue.row, new_queue.col))
                   continue;
                end
                
                queue{end + 1} = new_queue;
                distances(new_queue.row, new_queue.col) = new_queue.cost;
            end
        end
    end
end

function index = min_cost(queue)
    index = 1;
    min_node = queue{1};
    for i = 2:size(queue, 2)
        if queue{i}.cost < min_node.cost
            index = i;
            min_node = queue{i};
        end
    end
end

