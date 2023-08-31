function [se, time_array, distance_array, association_array, n] = AssociateAndGetSquareErrors(meas, times, ref, minimal_distance)
    % Obtain matrices
    idx_triangle = 1;
    triangles(1:6, idx_triangle) = [meas(:,1); zeros(4,1)];
    time_array = times(1);
    for i=2:length(meas(1, :))
        if norm(meas(:,i) - triangles(1:2, idx_triangle)) >= minimal_distance
            idx_triangle = idx_triangle + 1;
            triangles(1:6, idx_triangle) = [meas(:,i); zeros(4,1)];
            time_array(idx_triangle) = times(i);
        end
    end
    reference_position = reshape([ref(1:2, :); ref(end, :)], [3, 1, length(ref(1, :))]);
    reference_position = repmat(reference_position, [1 length(triangles(1,:)) 1]);

    % Get square distances
    square_distances = sum((reference_position(1:2,:,:) - triangles(1:2, :)) .^ 2, 1);

    % Get indices of closest points
    closer_idx = (square_distances == min(square_distances,[],3));
    for j=1:length(triangles(1,:))
        triangles(3:4, j) = reference_position(1:2, j, closer_idx(1, j, :));
        square_distances(1, j, closer_idx(1, j, :)) = NaN; % Exclude from search of minimal in next iteration
        association_array(j) = reference_position(3, j, closer_idx(1, j, :));
    end

    % Get indices of second closest points
    closer_idx = (square_distances == min(square_distances,[],3));
    for j=1:length(triangles(1,:))
        triangles(5:6, j) = reference_position(1:2, j, closer_idx(1, j, :));
    end
    
    % Get squared distances to segment defined by closest points and calculate squared errors
    squared_ab = (triangles(1, :) - triangles(3, :)).^2 + (triangles(2, :) - triangles(4, :)).^2;
    squared_ac = (triangles(5, :) - triangles(3, :)).^2 + (triangles(6, :) - triangles(4, :)).^2;
    dot_product = (triangles(1, :) - triangles(3, :)) .* (triangles(5, :) - triangles(3, :)) + (triangles(2, :) - triangles(4, :)) .* (triangles(6, :) - triangles(4, :));
    distance_array = (squared_ab - (dot_product.^2) ./ squared_ac);
    se = sum(distance_array);
    distance_array = sqrt(distance_array);
    n = length(triangles(1,:));
    association_array = [triangles(1:4, :); association_array];
