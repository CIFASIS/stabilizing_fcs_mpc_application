sigma_position = 0.03;
sigma_orientation = 0.03;
extreme_position = 0.1;
extreme_orientation = 0.1;
time_noise = [0:0.2:120]';

for i=1:10
    prob_dist = makedist('Normal', 0,sigma_position);
    prob_dist_truncated = truncate(prob_dist,-extreme_position,extreme_position);
    noise_x = [time_noise, random(prob_dist_truncated,length(time_noise),1)];
    noise_y = [time_noise, random(prob_dist_truncated,length(time_noise),1)];
    prob_dist = makedist('Normal', 0,sigma_orientation);
    prob_dist_truncated = truncate(prob_dist,-extreme_orientation,extreme_orientation);
    noise_theta = [time_noise, random(prob_dist_truncated,length(time_noise),1)];
    save(sprintf("noise_%02d.mat", i),'noise_x','noise_y', 'noise_theta')
end
