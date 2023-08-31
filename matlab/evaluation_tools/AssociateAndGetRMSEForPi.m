function [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEForPi(logsout, time_table, initial_idx, minimal_distance, end_x)
    % Separate arrays
    idx_x = initial_idx;
    idx_y = idx_x + 1;
    ref_max_x_idx = find(time_table(1, :) == max(time_table(1, :))); % First part is from beginning to rightmost
    ref_max_x_idx = ref_max_x_idx(1); % Avoid possible duplicate
    ref_1 = [time_table(1:2, 1:ref_max_x_idx); time_table(end, 1:ref_max_x_idx)];
    ref_2 = [time_table(1:2, ref_max_x_idx+1:end); time_table(end, ref_max_x_idx+1:end)];
    meas_max_x_idx = find(logsout{idx_x}.Values.Data == max(logsout{idx_x}.Values.Data));
    meas_max_x_idx = meas_max_x_idx(1);
    meas_1 = [logsout{idx_x}.Values.Data(1:meas_max_x_idx)'; logsout{idx_y}.Values.Data(1:meas_max_x_idx)'];
    times_1 = logsout{idx_x}.Values.Time(1:meas_max_x_idx);
    meas_2 = [logsout{idx_x}.Values.Data(meas_max_x_idx+1:end)'; logsout{idx_y}.Values.Data(meas_max_x_idx+1:end)'];
    times_2 = logsout{idx_x}.Values.Time(meas_max_x_idx+1:end);
    if end_x > 0
        data_filter = meas_2(1,:) > end_x;
        meas_2 = meas_2(:, data_filter);
        times_2 = times_2(data_filter);
    end

    % Calculate square error of each part
    [se1, time_array1, distance_array1, association_array1, n1] = AssociateAndGetSquareErrors(meas_1, times_1, ref_1, minimal_distance);
    [se2, time_array2, distance_array2, association_array2, n2] = AssociateAndGetSquareErrors(meas_2, times_2, ref_2, minimal_distance);

    time_array = [time_array1, time_array2];
    distance_array = [distance_array1, distance_array2];
    association_array = [association_array1, association_array2];

    % Calculate RMSE
    rmse = ((se1 + se2) / (n1 + n2))^0.5;
