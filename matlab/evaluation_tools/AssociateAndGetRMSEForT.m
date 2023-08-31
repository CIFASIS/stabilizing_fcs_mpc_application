function [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEForT(logsout, time_table, initial_idx, minimal_distance, end_x)
    % Separate arrays
    idx_x = initial_idx;
    idx_y = idx_x + 1;
    ref_max_y_idx = find(time_table(2, :) == max(time_table(2, :))); % First part is from beginning to uppermost
    ref_max_y_idx = ref_max_y_idx(1); % Avoid possible duplicate
    ref_1 = [time_table(1:2, 1:ref_max_y_idx); time_table(end, 1:ref_max_y_idx)];
    ref_max_x_idx = find(time_table(1, :) == max(time_table(1, :))); % Second part is from uppermost to rightmost. Last part is the rest.
    ref_max_x_idx = ref_max_x_idx(1);
    ref_2 = [time_table(1:2, ref_max_y_idx+1:ref_max_x_idx); time_table(end, ref_max_y_idx+1:ref_max_x_idx)];
    ref_3 = [time_table(1:2, ref_max_x_idx+1:end); time_table(end, ref_max_x_idx+1:end)];
    meas_max_y_idx = find(logsout{idx_y}.Values.Data == max(logsout{idx_y}.Values.Data));
    meas_max_y_idx = meas_max_y_idx(1);
    meas_1 = [logsout{idx_x}.Values.Data(1:meas_max_y_idx)'; logsout{idx_y}.Values.Data(1:meas_max_y_idx)'];
    times_1 = logsout{idx_x}.Values.Time(1:meas_max_y_idx);
    meas_max_x_idx = find(logsout{idx_x}.Values.Data == max(logsout{idx_x}.Values.Data));
    meas_max_x_idx = meas_max_x_idx(1);
    meas_2 = [logsout{idx_x}.Values.Data(meas_max_y_idx+1:meas_max_x_idx)'; logsout{idx_y}.Values.Data(meas_max_y_idx+1:meas_max_x_idx)'];
    times_2 = logsout{idx_x}.Values.Time(meas_max_y_idx+1:meas_max_x_idx);
    meas_3 = [logsout{idx_x}.Values.Data(meas_max_x_idx+1:end)'; logsout{idx_y}.Values.Data(meas_max_x_idx+1:end)'];
    times_3 = logsout{idx_x}.Values.Time(meas_max_x_idx+1:end);
    if end_x > 0
        data_filter = meas_3(1,:) > end_x;
        meas_3 = meas_3(:, data_filter);
        times_3 = times_3(data_filter);
    end

    % Calculate square error of each part
    [se1, time_array1, distance_array1, association_array1, n1] = AssociateAndGetSquareErrors(meas_1, times_1, ref_1, minimal_distance);
    [se2, time_array2, distance_array2, association_array2, n2] = AssociateAndGetSquareErrors(meas_2, times_2, ref_2, minimal_distance);
    [se3, time_array3, distance_array3, association_array3, n3] = AssociateAndGetSquareErrors(meas_3, times_3, ref_3, minimal_distance);

    time_array = [time_array1, time_array2, time_array3];
    distance_array = [distance_array1, distance_array2, distance_array3];
    association_array = [association_array1, association_array2, association_array3];

    % Calculate RMSE
    rmse = ((se1 + se2 + se3) / (n1 + n2 + n3))^0.5;
