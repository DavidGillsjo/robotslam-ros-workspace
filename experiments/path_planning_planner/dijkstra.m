function [ distances ] = dijkstra( goal, free_matrix, cost_matrix )
%BFS Performs dijkstra and returns matrix with distance from goal
    
    distances = zeros(size(free_matrix));
    distances(:) = inf;
    is_in_queue = zeros(size(free_matrix));
    
    [goal_row, goal_col] = ind2sub(size(free_matrix), goal);
    
    queue = {{}};
    queue{1}.row = goal_row;
    queue{1}.col = goal_col;
    %queue{1}.cost = 1;
    
    distances(goal_row, goal_col) = 1;
    
    while (size(queue, 2)) 
        min_i = min_cost(queue, distances);
        pos = queue{min_i};
        cost = distances(pos.row, pos.col);
        queue(min_i) = [];
        is_in_queue(pos.row, pos.col) = 0;
        
        % Add adjacency coords to queue.
        for r = -1:1
            for c = -1:1
                if (r == 0 && c == 0)
                    continue;
                end
                
                new_queue = {};
                new_queue.row = pos.row + r;
                new_queue.col = pos.col + c;
                new_cost = cost + 1 + cost_matrix(new_queue.row, new_queue.col);
                
                if (~free_matrix(new_queue.row, new_queue.col))
                   continue;
                end
                
                if (new_cost < distances(new_queue.row, new_queue.col))
                    index = find_in_queue(new_queue, queue);
                    distances(new_queue.row, new_queue.col) = new_cost;
                end
            end
        end
    end
end

function index = min_cost(queue, distances)
    index = 1;
    min_node = queue{1};
    min_cost = distances(min_node.row, min_node.col);
    for i = 2:size(queue, 2)
        node = queue{i};
        cost = distances(node.row, node.col);
        if cost < min_cost
            index = i;
            min_node = queue{i};
        end
    end
end

function index = find_in_queue(node, queue)
    index = 0;
    for i = 2:size(queue, 2)
        if queue{i}.row == node.row && queue{i}.col == node.col
            index = i;
            return;
        end
    end
end