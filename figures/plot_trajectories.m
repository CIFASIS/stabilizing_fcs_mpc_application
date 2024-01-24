addpath('../matlab/evaluation_tools');

%% Plot Trajectories with details
clear; clc;

cd ../matlab/nmpc;
addpath('.');

parameters;

simulations_number = 4;
references = cell(simulations_number,1);
references{1} = trajectory_omega;
references{2} = trajectory_omega;
references{3} = trajectory_pi;
references{4} = trajectory_t;
AssociateAndGetRMSEGeneral = cell(simulations_number,1);
AssociateAndGetRMSEGeneral{1} = @AssociateAndGetRMSEForOmega;
AssociateAndGetRMSEGeneral{2} = @AssociateAndGetRMSEForOmega;
AssociateAndGetRMSEGeneral{3} = @AssociateAndGetRMSEForPi;
AssociateAndGetRMSEGeneral{4} = @AssociateAndGetRMSEForT;
names = cell(simulations_number,1);
names{1} = 'omega_vert';
names{2} = 'omega_cut';
names{3} = 'pi_cut';
names{4} = 't_cut';
figures = cell(simulations_number,1);
end_x_trajectories = [25, 25, 30, 30];
end_x_idx = [334, 334, 227, 410];
xlims = [0, 51; 30, 51; 35, 47; 35, 51];
ylims = [-1.75, 4.50; -1.75, 4.50; -0.15, 6.1; -2.6, 3.65];
aspects = [51, 6.25, 1; 21, 6.25, 1; 12, 6.25, 1; 16, 6.25, 1];
positions = [100 100 2000 275; 100 100 2000 275; 100 100 2000 275; 100 100 2000 275];
scales_plot = [0.6 1.1; 0.37 1; 0.37 1; 0.37 1];

for i = 1:simulations_number
    trajectory = references{i};
    final_time = step_time * (length(trajectory(1,:)) - Hp - 1);
    try
        clear NmpcController
        sim('nmpc', 1.5*final_time);
    end
    idx_x = 13;
    idx_y = idx_x + 1;
    [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{i}(logsout, trajectory, idx_x, minimal_distance, end_x_trajectories(i));
    rmse
    [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{i}(logsout, trajectory, idx_x, 0, end_x_trajectories(i));
    ang = degtorad(-10);
    tras = [0 -0.1];
    rotation_matrix = [cos(-ang) sin(-ang) tras(1); -sin(-ang) cos(-ang) tras(2); 0 0 1];
    rotated_positions = rotation_matrix * [association_array(1:2,:); ones(1, length(association_array(1,:)))];
    rotated_trajectory = rotation_matrix * [trajectory(1:2,:); ones(1, length(trajectory(1,:)))];

    figures{i} = figure('visible','off');
    plot(rotated_trajectory(1,1:end_x_idx(i)),rotated_trajectory(2,1:end_x_idx(i)), 'k--', 'LineWidth', 0.1);
    hold 'on';
    plot(rotated_positions(1,:),rotated_positions(2,:), 'b-.', 'LineWidth', 0.1);
end

clearvars -except figures end_x_trajectories xlims ylims simulations_number references AssociateAndGetRMSEGeneral names aspects positions scales_plot;% clc;

cd ../fcs_mpc;

parameters;

for i = 1:simulations_number
    trajectory = references{i};
    final_time = step_time * (length(trajectory(1,:)) - Hp - 1);
    try
        clear FCS_MPC
        sim('fcs_mpc', 1.5*final_time);
    end
    idx_x = 12;
    idx_y = idx_x + 1;
    [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{i}(logsout, trajectory, idx_x, minimal_distance, end_x_trajectories(i));
    rmse
    [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{i}(logsout, trajectory, idx_x, 0, end_x_trajectories(i));
    ang = degtorad(-10);
    tras = [0 -0.1];
    rotation_matrix = [cos(-ang) sin(-ang) tras(1); -sin(-ang) cos(-ang) tras(2); 0 0 1];
    rotated_positions = rotation_matrix * [association_array(1:2,:); ones(1, length(association_array(1,:)))];

    figure(figures{i});
    plot(rotated_positions(1,:),rotated_positions(2,:), 'r-', 'LineWidth', 0.1);

    xlim(xlims(i,:));
    ylim(ylims(i,:));
    pbaspect(aspects(i,:));
    ax = gca;
    ax.FontSize = 16; % Font size
    set(gca,'TickLabelInterpreter','latex')
    grid 'on'
    xlabel('$x\;[m]$','Interpreter','latex');
    ylabel('$y\;[m]$','Interpreter','latex');
    figures{i}.Position = positions(i, :);
    if i == 1
        ax.FontSize = 22; % Font size
        hLg = legend({'Reference $\quad$', 'NMPC $\quad$', 'FCS-MPC $\quad$'}, 'Orientation','horizontal','Location','north','Interpreter','latex');
        pos = hLg.Position;
        pos(1) = pos(1);
        pos(2) = pos(2) + 0.07;
        hLg.Position = pos;
        legend('boxoff')
        hYLabel = get(gca,'YLabel');
        set(hYLabel,'rotation',-90,'VerticalAlignment','middle')
        pyl = get(hYLabel,'position');
        pyl(1) = 3*pyl(1);
        set(hYLabel,'position',pyl)
    end
    set(figures{i},'Units','Inches');
    pos = get(figures{i},'Position');
    set(figures{i},'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3)*scales_plot(i,1), pos(4)*scales_plot(i,2)])
    print(figures{i},names{i},'-dpdf','-r0')
    movefile(sprintf('%s.pdf', names{i}), '../../figures/');
end

cd ../../figures;
close all
