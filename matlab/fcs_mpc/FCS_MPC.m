function [control, unfeasible] = FCS_MPC(meas, trajectory, Hp, Hc, step_time, R, phi_max, L, omega_f, F_OmegaI, g_OmegaI, F_OmegaO, g_OmegaO, r, c, varying_cost, speed_cost_coefficient)
    ref = GetReference(meas, trajectory, Hp, step_time, L);
    [control, unfeasible] = PredictEvaluate(meas, ref, Hp, Hc, step_time, R, phi_max, L, omega_f, F_OmegaI, g_OmegaI, F_OmegaO, g_OmegaO, r, c, varying_cost, speed_cost_coefficient);
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
    next_points = trajectory(1:2, next_idx: next_idx + Hp + 2); % +2 because of relative degree between position and phi angle
    close_points = trajectory(1:2, close_idx: close_idx + Hp + 2);
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

    ref = [interpolated_ref(1:2, 1:end-2); interpolated_theta(1:end-1); interpolated_phi; interpolated_speed(1:end-1); zeros(1, Hp + 1)];
end


function [control, unfeasible] = PredictEvaluate(meas, ref, Hp, Hc, step_time, R, phi_max, L, omega_f, F_OmegaI, g_OmegaI, F_OmegaO, g_OmegaO, r, c, varying_cost, speed_cost_coefficient)
    persistent u_complete;
    persistent g_OmegaI_extended;
    if isempty(u_complete)
        u_complete = GenerateControl(Hc, Hp);
        g_OmegaI_extended = repmat(g_OmegaI, [1, 3^Hc]);
    end

    x0 = repmat(meas, [1, 3^Hc]);
    speed_reference = reshape(ref(5, 1:end-1), [1, 1, Hp]); % Avoid last reference because, since in this case it is just an input, it does not affect predicted state evolution (inputs up to k affects states up to k+1). It exists, however, because it is part of the reference matrix that includes states and we need states up to k+1 to calculate errors.
    speed_reference = repmat(speed_reference, [1 size(x0, 2) 1]);
    u = [speed_reference; u_complete];
    [t, x] = Euler1(@Ackerman, x0, u, Hp, step_time, phi_max, L, omega_f);

    % Get errors
    reference = reshape(ref(1:4, 2:end), [4, 1, Hp]); % Avoid first state and first reference because they are the same for all
    reference = repmat(reference, [1 size(x, 2) 1]);
    position_error = x(1:2,:,2:end) - reference(1:2,:,:);
    squared_position_error = position_error(1, :, :).^2 + position_error(2, :, :).^2;
    error = sqrt(squared_position_error);
    error(2,:,:) = wrapToPi(x(3,:,2:end) - reference(3,:,:)); % With wrapToPi, problems regarding discontinuities due to range (-pi; pi] (atan2) are avoided (orientation is continuous but reference orientation could have discontinuities)
    error(3,:,:) = x(4,:,2:end) - reference(4,:,:);

    % Remove states that do not satisfy constraint
    constraint_filter = false(1, size(error, 2));
    for jj = 1:Hc
        constraint_filter = constraint_filter | all(F_OmegaI*error(:,:,jj)<g_OmegaI_extended,1);
    end

    % At this point, execution diverges depending on constraint fulfillment
    if (sum(constraint_filter)==0)
        unfeasible = meas;
        % Compute total cost
        if varying_cost
            reference_speed = reshape(ref(5, 2:end), [1, 1, Hp]);
            reference_speed = repmat(reference_speed, [1 size(error, 2) 1]);
            squared_theta_error = (error(2,:,:) .* reference_speed * step_time).^2;
            squared_phi_error = (error(3,:,:) .* (reference_speed.^2) * step_time^2 / L).^2;
        else
            squared_theta_error = (error(2,:,:) * (speed_cost_coefficient) * step_time).^2;
            squared_phi_error = (error(3,:,:) * ((speed_cost_coefficient)^2) * step_time^2 / L).^2;
        end
        total_cost_characteristic = sum((squared_position_error + squared_theta_error + squared_phi_error), 3);
    else
        unfeasible = [NaN; NaN; NaN; NaN];
        
        u = u(:, constraint_filter, :);
        error = error(:, constraint_filter, :);

        % Generate function used in stage cost
        characteristic_mask = ones(1,size(error, 2),Hp);
        g_OmegaO_extended = repmat(g_OmegaO, [1, size(error,2)]);
        for jj = 1:Hc
            filter = all(F_OmegaO*error(:,:,jj)<g_OmegaO_extended,1);
            characteristic_mask(:,filter,jj) = c;
            characteristic_mask(:,~filter,jj) = 1 + r * (jj - 1);
        end
        for jj = (Hc + 1):Hp
            characteristic_mask(:,:,jj) = c;
        end

        % Compute total cost
        if varying_cost
            reference_speed = reshape(ref(5, 2:end), [1, 1, Hp]);
            reference_speed = repmat(reference_speed, [1 size(error, 2) 1]);
            squared_theta_error = (error(2,:,:) .* reference_speed * step_time).^2;
            squared_phi_error = (error(3,:,:) .* (reference_speed.^2) * step_time^2 / L).^2;
        else
            squared_theta_error = (error(2,:,:) * (speed_cost_coefficient) * step_time).^2;
            squared_phi_error = (error(3,:,:) * ((speed_cost_coefficient)^2) * step_time^2 / L).^2;
        end
        total_cost_characteristic = sum((squared_position_error(:,constraint_filter,:) + squared_theta_error + squared_phi_error) .* characteristic_mask, 3);
    end

    % Find control action
    control_idx = find(total_cost_characteristic == min(total_cost_characteristic));
    control_idx = control_idx(1); % control_idx could be an array if costs are the same for different input sequences
    control = u(:, control_idx, 1);
    control(1) = control(1) * 30 / (pi * R);
end










function u = GenerateControl(Hc, Hp)
    amount_control_sequences = 3^Hc;
    omega = zeros(amount_control_sequences, Hp); % After the control horizont, there are no more changes in control actions
    for i = 1:Hc
        block_size = amount_control_sequences / (3^i);
        for j = 1:3^(i - 1)
            offset = 3 * (j - 1) * block_size;
            omega(offset + 1: offset + block_size, i) = ones(block_size, 1);
            omega(offset + block_size + 1: offset + 2 * block_size, i) = zeros(block_size, 1);
            omega(offset + 2 * block_size + 1: offset + 3 * block_size, i) = -ones(block_size, 1);
        end
    end
    u = reshape(omega, [1, amount_control_sequences, Hp]);
end


function dx = Ackerman(t, x, u, L, omega_f)
    dx = [u(1, :) .* cos(x(3, :)); u(1, :) .* sin(x(3, :)); u(1, :) .* tan(x(4, :)) / L; u(2, :) * omega_f];
end


function [t, x] = Euler1(f, x0, u, N, h, phi_max, L, omega_f)
    t = zeros(N + 1, 1);
    x = zeros(length(x0(:, 1)), length(u(1, :, 1)), N + 1);
    x(:, :, 1) = x0;
    for k = 1:N
        t(k + 1) = t(k) + h;
        x(:, :, k + 1) = x(:, :, k) + h * f(t(k), x(:, :, k), u(:, :, k), L, omega_f);

        % Saturation. This should be done in a function passed as argument, AckermanSaturate for example. And Ackerman function should be called AckermanDerivate.
        mask = x(4, :, k + 1) > phi_max;
        x(4, mask, k + 1) = phi_max;
        mask = x(4, :, k + 1) < -phi_max;
        x(4, mask, k + 1) = -phi_max;
    end
end
