addpath('matlab/evaluation_tools'); 

%% Explore horizons space with fcs_mpc

clear; clc;

cd matlab/fcs_mpc;
addpath('.');

parameters;

t = datetime('now','Format','yyyyMMdd''_''HHmmss');
directory = ['evaluation_', char(t)];
mkdir('.', directory);
cd(directory);
Hps = 10:1:24;
Hcs = 6:1:10;
simulations_number = 3;
references = cell(simulations_number,1);
references{1} = trajectory_omega;
references{2} = trajectory_pi;
references{3} = trajectory_t;
straight_assoc = [153, 276; 153, 188; 178, 364];
AssociateAndGetRMSEGeneral = cell(simulations_number,1);
AssociateAndGetRMSEGeneral{1} = @AssociateAndGetRMSEForOmega;
AssociateAndGetRMSEGeneral{2} = @AssociateAndGetRMSEForPi;
AssociateAndGetRMSEGeneral{3} = @AssociateAndGetRMSEForT;
titles = cell(simulations_number,1);
titles{1} = 'omega';
titles{2} = 'pi';
titles{3} = 't';
end_x_trajectories = [25, 30, 30];
rmses = cell(simulations_number,1);
crop_row_distances = cell(simulations_number,1);
unfeasible_without_curve = cell(simulations_number,1);
noises_number = 10;
noises = cell(noises_number,1);
noises{1} = 'noise_01.mat';
noises{2} = 'noise_02.mat';
noises{3} = 'noise_03.mat';
noises{4} = 'noise_04.mat';
noises{5} = 'noise_05.mat';
noises{6} = 'noise_06.mat';
noises{7} = 'noise_07.mat';
noises{8} = 'noise_08.mat';
noises{9} = 'noise_09.mat';
noises{10} = 'noise_10.mat';

for k = 1:simulations_number
    mkdir('.', titles{k});
    cd(titles{k});
    rmses_total = zeros(length(Hps), length(Hcs));
    crop_row_distances_total = zeros(length(Hps), length(Hcs));
    unfeasible_without_curve_total = zeros(length(Hps), length(Hcs));
for jjj = 1:noises_number
    i = 1;
    j = 1;
    rmses{k} = zeros(length(Hps), length(Hcs));
    crop_row_distances{k} = zeros(length(Hps), length(Hcs));
    unfeasible_without_curve{k} = zeros(length(Hp), length(Hc));
    trajectory = references{k};
    for Hp = Hps
        for Hc = Hcs
            final_time = step_time * (length(trajectory(1,:)) - Hp - 1);
            load(noises{jjj});
            try
                clear FCS_MPC
                sim('../../fcs_mpc', 1.5*final_time);
            end
            save(sprintf("logsout_%s_hp_%d_hc_%d_%s", titles{k}, Hps(i), Hcs(j), noises{jjj}), 'logsout');
            idx_x = 12;
            idx_y = idx_x + 1;
            try
                [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{k}(logsout, trajectory, idx_x, minimal_distance, end_x_trajectories(k));
                rmses{k}(i, j) = rmse;
            catch
                rmses{k}(i, j) = NaN;
            end
            try
                [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{k}(logsout, [trajectory; [true(1,straight_assoc(k,1)), false(1,straight_assoc(k,2) - straight_assoc(k,1) - 1), true(1,size(trajectory,2) - straight_assoc(k,2) + 1)]], idx_x, 0, end_x_trajectories(k));
                filter = (association_array(5,:)==1);
                indices = find(filter == 1);
                time_array1 = time_array(filter);
                association_array1 = association_array(:, filter);
                total_distance = 0;
                for iii = 2:length(indices)
                    if (indices(iii) == (indices(iii-1) + 1))
                        total_distance = total_distance + norm(association_array1(1:2, iii) - association_array1(1:2, iii-1));
                    end
                end
                y_max = 0.1;
                filter = ((distance_array > y_max) & (association_array(5,:)==1));
                indices = find(filter == 1);
                time_array2 = time_array(filter);
                association_array2 = association_array(:, filter);
                crop_row_distance = 0;
                for iii = 2:length(indices)
                    if (indices(iii) == (indices(iii-1) + 1))
                        crop_row_distance = crop_row_distance + norm(association_array2(1:2, iii) - association_array2(1:2, iii-1));
                    end
                end
                crop_row_distances{k}(i, j) = crop_row_distance / total_distance * 100.;
            catch
                crop_row_distances{k}(i, j) = NaN;
            end
            idx_x_sampled = 9; % It may be needed to change this
            idx_y_sampled = idx_x_sampled + 1;
            logsout2 = logsout; % This is just a "hack" to simplify things
            logsout2{idx_x}.Values = logsout{1}.Values;
            logsout2{idx_x}.Values.Data = logsout{idx_x_sampled}.Values.Data((repmat([1, zeros(1,sampling_scale-1)], [1,length(logsout2{idx_x}.Values.Data(:,1))])==1), :);
            logsout2{idx_y}.Values = logsout{1}.Values;
            logsout2{idx_y}.Values.Data = logsout{idx_y_sampled}.Values.Data((repmat([1, zeros(1,sampling_scale-1)], [1,length(logsout2{idx_x}.Values.Data(:,1))])==1), :);
            try
                [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{k}(logsout2, [trajectory; [true(1,straight_assoc(k,1)), false(1,straight_assoc(k,2) - straight_assoc(k,1) - 1), true(1,size(trajectory,2) - straight_assoc(k,2) + 1)]], idx_x, 0, end_x_trajectories(k));
                unfeasible_without_curve{k}(i, j) = sum((~isnan(logsout{1}.Values.Data(:,1))) & ([association_array(5,:),zeros(1,length(logsout{1}.Values.Data(:,1))-length(association_array(5,:)))]==1)') / sum(association_array(5,:)) * 100.;
            catch
                unfeasible_without_curve{k}(i, j) = NaN;
            end

            disp(sprintf("Done %d simulations of %d", j + (i - 1) * length(Hcs), length(Hps) * length(Hcs)));
            j = j + 1;
            close all;
        end
        i = i + 1;
        j = 1;
    end
    disp(sprintf("Finished %s", titles{k}));
    f = figure('visible','off');
    h = heatmap(string(Hcs), string(Hps), rmses{k});
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, sprintf("heatmap_rmses_%s.png", noises{jjj}),'png');
    close all;
    mean(mink(rmses{k}(:),10))
    rmses_total = rmses_total + rmses{k};

    f = figure('visible','off');
    h = heatmap(string(Hcs), string(Hps), crop_row_distances{k});
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, sprintf("heatmap_crop_row_distances_%s.png", noises{jjj}),'png');
    close all;
    mean(mink(crop_row_distances{k}(:),10))
    crop_row_distances_total = crop_row_distances_total + crop_row_distances{k};

    f = figure('visible','off');
    h = heatmap(string(Hcs), string(Hps), unfeasible_without_curve{k});
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, sprintf("heatmap_unfeasible_without_curve_%s.png", noises{jjj}),'png');
    close all;
    mean(mink(unfeasible_without_curve{k}(:),10))
    unfeasible_without_curve_total = unfeasible_without_curve_total + unfeasible_without_curve{k};
    save(sprintf("workspace_%s_%d.mat", titles{k}, jjj),'rmses', 'crop_row_distances', 'unfeasible_without_curve')
end
    disp(sprintf("Finished %s %s", titles{k}, noises{jjj}));
    f = figure('visible','off');
    rmses_total = rmses_total / noises_number;
    h = heatmap(string(Hcs), string(Hps), rmses_total);
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, 'heatmap_rmses_total.png','png');
    close all;
    mean(mink(rmses_total(:),10))

    f = figure('visible','off');
    crop_row_distances_total = crop_row_distances_total / noises_number;
    h = heatmap(string(Hcs), string(Hps), crop_row_distances_total);
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, 'heatmap_crop_row_distances_total.png','png');
    close all;
    mean(mink(crop_row_distances_total(:),10))

    f = figure('visible','off');
    unfeasible_without_curve_total = unfeasible_without_curve_total / noises_number;
    h = heatmap(string(Hcs), string(Hps), unfeasible_without_curve_total);
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, 'heatmap_unfeasible_without_curve_total.png','png');
    close all;
    mean(mink(unfeasible_without_curve_total(:),10))
    save(sprintf("workspace_%s.mat", titles{k}),'rmses_total','crop_row_distances_total', 'unfeasible_without_curve_total')
    cd ..;
end

cd ..;
cd ../..;




%% Explore horizons space with nmpc

clear; %clc;

cd matlab/nmpc;
addpath('.');

parameters;

t = datetime('now','Format','yyyyMMdd''_''HHmmss');
directory = ['evaluation_', char(t)];
mkdir('.', directory);
cd(directory);
Hps = 10:1:24;
Hcs = 6:1:10;
simulations_number = 3;
references = cell(simulations_number,1);
references{1} = trajectory_omega;
references{2} = trajectory_pi;
references{3} = trajectory_t;
straight_assoc = [153, 276; 153, 188; 178, 364];
AssociateAndGetRMSEGeneral = cell(simulations_number,1);
AssociateAndGetRMSEGeneral{1} = @AssociateAndGetRMSEForOmega;
AssociateAndGetRMSEGeneral{2} = @AssociateAndGetRMSEForPi;
AssociateAndGetRMSEGeneral{3} = @AssociateAndGetRMSEForT;
titles = cell(simulations_number,1);
titles{1} = 'omega';
titles{2} = 'pi';
titles{3} = 't';
end_x_trajectories = [25, 30, 30];
rmses = cell(simulations_number,1);
crop_row_distances = cell(simulations_number,1);
unfeasible_without_curve = cell(simulations_number,1);
noises_number = 10;
noises = cell(noises_number,1);
noises{1} = 'noise_01.mat';
noises{2} = 'noise_02.mat';
noises{3} = 'noise_03.mat';
noises{4} = 'noise_04.mat';
noises{5} = 'noise_05.mat';
noises{6} = 'noise_06.mat';
noises{7} = 'noise_07.mat';
noises{8} = 'noise_08.mat';
noises{9} = 'noise_09.mat';
noises{10} = 'noise_10.mat';

for k = 1:simulations_number
    mkdir('.', titles{k});
    cd(titles{k});
    rmses_total = zeros(length(Hps), length(Hcs));
    crop_row_distances_total = zeros(length(Hps), length(Hcs));
for jjj = 1:noises_number
    i = 1;
    j = 1;
    rmses{k} = zeros(length(Hps), length(Hcs));
    crop_row_distances{k} = zeros(length(Hps), length(Hcs));
    trajectory = references{k};
    for Hp = Hps
        for Hc = Hcs
            nlobj.PredictionHorizon = Hp;
            nlobj.ControlHorizon = Hc;
            final_time = step_time * (length(trajectory(1,:)) - Hp - 1);
            load(noises{jjj});
            try
                clear NmpcController
                sim('../../nmpc', 1.5*final_time);
            end
            save(sprintf("logsout_%s_hp_%d_hc_%d_%s", titles{k}, Hps(i), Hcs(j), noises{jjj}), 'logsout');
            idx_x = 13;
            idx_y = idx_x + 1;
            try
                [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{k}(logsout, trajectory, idx_x, minimal_distance, end_x_trajectories(k));
                rmses{k}(i, j) = rmse;
            catch
                rmses{k}(i, j) = NaN;
            end
            try
                [rmse, time_array, distance_array, association_array] = AssociateAndGetRMSEGeneral{k}(logsout, [trajectory; [true(1,straight_assoc(k,1)), false(1,straight_assoc(k,2) - straight_assoc(k,1) - 1), true(1,size(trajectory,2) - straight_assoc(k,2) + 1)]], idx_x, 0, end_x_trajectories(k));
                filter = (association_array(5,:)==1);
                indices = find(filter == 1);
                time_array1 = time_array(filter);
                association_array1 = association_array(:, filter);
                total_distance = 0;
                for iii = 2:length(indices)
                    if (indices(iii) == (indices(iii-1) + 1))
                        total_distance = total_distance + norm(association_array1(1:2, iii) - association_array1(1:2, iii-1));
                    end
                end
                y_max = 0.1;
                filter = ((distance_array > y_max) & (association_array(5,:)==1));
                indices = find(filter == 1);
                time_array2 = time_array(filter);
                association_array2 = association_array(:, filter);
                crop_row_distance = 0;
                for iii = 2:length(indices)
                    if (indices(iii) == (indices(iii-1) + 1))
                        crop_row_distance = crop_row_distance + norm(association_array2(1:2, iii) - association_array2(1:2, iii-1));
                    end
                end
                crop_row_distances{k}(i, j) = crop_row_distance / total_distance * 100.;
            catch
                crop_row_distances{k}(i, j) = NaN;
            end
            disp(sprintf("Done %d simulations of %d", j + (i - 1) * length(Hcs), length(Hps) * length(Hcs)));
            j = j + 1;
            close all;
        end
        i = i + 1;
        j = 1;
    end
    disp(sprintf("Finished %s", titles{k}));
    f = figure('visible','off');
    h = heatmap(string(Hcs), string(Hps), rmses{k});
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, sprintf("heatmap_rmses_%s.png", noises{jjj}),'png');
    close all;
    mean(mink(rmses{k}(:),10))
    rmses_total = rmses_total + rmses{k};

    f = figure('visible','off');
    h = heatmap(string(Hcs), string(Hps), crop_row_distances{k});
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, sprintf("heatmap_crop_row_distances_%s.png", noises{jjj}),'png');
    close all;
    mean(mink(crop_row_distances{k}(:),10))
    crop_row_distances_total = crop_row_distances_total + crop_row_distances{k};
    save(sprintf("workspace_%s_%d.mat", titles{k}, jjj),'rmses', 'crop_row_distances')
end
    disp(sprintf("Finished %s %s", titles{k}, noises{jjj}));
    f = figure('visible','off');
    rmses_total = rmses_total / noises_number;
    h = heatmap(string(Hcs), string(Hps), rmses_total);
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, 'heatmap_rmses_total.png','png');
    close all;
    mean(mink(rmses_total(:),10))

    f = figure('visible','off');
    crop_row_distances_total = crop_row_distances_total / noises_number;
    h = heatmap(string(Hcs), string(Hps), crop_row_distances_total);
    f.Position = [200 200 500 400];
    h.XLabel = '$H_c$';
    h.YLabel = '$H_p$';
    h.NodeChildren(3).XAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).YAxis.Label.Interpreter = 'latex';
    h.NodeChildren(3).XAxis.TickLabelInterpreter = 'latex';
    h.NodeChildren(3).YAxis.TickLabelInterpreter = 'latex';
    set(f,'Units','Inches');
    pos = get(f,'Position');
    set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
    saveas(f, 'heatmap_crop_row_distances_total.png','png');
    close all;
    mean(mink(crop_row_distances_total(:),10))
    save(sprintf("workspace_%s.mat", titles{k}),'rmses_total','crop_row_distances_total')
    cd ..;
end

cd ..;
cd ../..;
