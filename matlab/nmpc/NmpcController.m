function [state_reference, speed_reference, current_speed_reference] = NmpcController(meas, trajectory, Hp, step_time, L)
    original_state_input_reference = GetReference(meas, trajectory, Hp, step_time, L);
    state_reference = original_state_input_reference(1:4,2:end)';
    speed_reference = original_state_input_reference(5,2:end)';
    current_speed_reference = original_state_input_reference(5,2);
end


function ref = GetReference(meas, trajectory, Hp, step_time, L)
    persistent close_idx;
    if isempty(close_idx)
        close_idx = 0; % Assume first position is close to beginning of reference trajectory
    end
    close_idx = close_idx + 1; % Under normal conditions, close_idx should be incremented by one with respect to previous step
    close_point = trajectory(1:2, close_idx); % close_point is not necessarily closer to current position than next_point
    next_idx = close_idx + 1;
    next_point = trajectory(1:2, next_idx); % Assume indices are not out of bounds
    ab = meas(1:2) - close_point; % These are segments used to project current position on the path
    ac = next_point - close_point;
    dot_product = ab' * ac;
    ad = ac * dot_product / (norm(ac) ^ 2);
    global_ad = ad + close_point;

    % Solve possible problems if there are
    while (close_idx > 1 && dot_product < 0) || (next_idx < length(trajectory(1,:)) && (dot_product / norm(ac)) > norm(ac))
        close_idx_alternative = close_idx + sign(dot_product);
        close_point_alternative = trajectory(1:2, close_idx_alternative);
        next_idx_alternative = next_idx + sign(dot_product);
        next_point_alternative = trajectory(1:2, next_idx_alternative);
        ab_alternative = meas(1:2) - close_point_alternative;
        ac_alternative = next_point_alternative - close_point_alternative;
        dot_product_alternative = ab_alternative' * ac_alternative;
        ad_alternative = ac_alternative * dot_product_alternative / (norm(ac_alternative) ^ 2);
        global_ad_alternative = ad_alternative + close_point_alternative;
        if (dot_product < 0 && (dot_product_alternative / norm(ac_alternative)) > norm(ac_alternative)) || (dot_product_alternative < 0 && (dot_product / norm(ac)) > norm(ac))
            if dot_product_alternative < 0
                close_idx = close_idx_alternative;
                close_point = close_point_alternative;
                next_idx = next_idx_alternative;
                next_point = next_point_alternative;
                ab = ab_alternative;
                ac = ac_alternative;
                dot_product = dot_product_alternative;
                ad = ad_alternative;
                global_ad = global_ad_alternative;
            end
            break;
        end
        close_idx = close_idx_alternative;
        close_point = close_point_alternative;
        next_idx = next_idx_alternative;
        next_point = next_point_alternative;
        ab = ab_alternative;
        ac = ac_alternative;
        dot_product = dot_product_alternative;
        ad = ad_alternative;
        global_ad = global_ad_alternative;
    end

    ratio_ad_ac = dot_product / (norm(ac) ^ 2);
    next_points = trajectory(1:2, next_idx: next_idx + Hp + 3); % +3 because of relative degree between position and omega (steering angle speed)
    close_points = trajectory(1:2, close_idx: close_idx + Hp + 3);
    interpolated_ref = close_points + (next_points - close_points) * ratio_ad_ac;

    if norm(ab) < norm(meas(1:2) - next_point) % Since close_point is not necessarily closer to meas than next_point, it is unknown a priori which speed should be assigned for the reference
        speed_idx = close_idx;
    else
        speed_idx = next_idx;
    end
    
    interpolated_speed_x_y = diff(interpolated_ref, 1, 2) / step_time;
    sign_change = sign(sum((interpolated_speed_x_y(:, 2:end) .* interpolated_speed_x_y(:, 1:end - 1)), 1)); % This is the sign of dot products between consecutive speeds. Negative sign means speed is inverted.
    sign_speed = [1, cumprod(sign_change)] * sign(trajectory(5, speed_idx)); % Multiplied by first speed sign because "reference speed sign" depend on that
    interpolated_speed = sqrt(sum((interpolated_speed_x_y .^ 2), 1)) .* sign_speed;

    interpolated_theta = diff(interpolated_ref, 1, 2);
    interpolated_theta = atan2(interpolated_theta(2,:), interpolated_theta(1,:));
    interpolated_theta = interpolated_theta + 0.5 * pi * (1 - sign(interpolated_speed)); % Vectorized way of adding pi to theta when speed is negative to avoid wrong theta calculation

    interpolated_phi = atan(L/step_time*wrapToPi(diff(interpolated_theta))./interpolated_speed(1:end-1));
    interpolated_omega = diff(interpolated_phi) / step_time;

    ref = [interpolated_ref(1:2, 1:end-3); interpolated_theta(1:end-2); interpolated_phi(1:end-1); interpolated_speed(1:end-2); interpolated_omega];
end
