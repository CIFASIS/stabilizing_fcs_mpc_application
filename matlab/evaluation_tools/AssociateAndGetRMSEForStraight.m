function [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEForStraight(logsout, time_table, initial_idx, minimal_distance, end_x)
    idx_x = initial_idx;
    idx_y = idx_x + 1;
    ref = [time_table(1:2,:); time_table(end,:)];
    meas = [logsout{idx_x}.Values.Data'; logsout{idx_y}.Values.Data'];
    times = logsout{idx_x}.Values.Time;
    if end_x > 0
        data_filter = meas(1,:) < end_x;
        meas = meas(:, data_filter);
        times = times(data_filter);
    end

    % Calculate square error
    [se, time_array, distance_array, association_array, n] = AssociateAndGetSquareErrors(meas, times, ref, minimal_distance);

    % Calculate RMSE
    rmse = (se / n)^0.5;
