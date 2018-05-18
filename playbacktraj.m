clear; clc; close all; 
addpath('../')
%% playback trajectories
fname = 'TrajOptSwingUp7';
save_dir = '~/dev/drake-var-hamr/drake/examples/HAMR-URDF/dev/SimpleHAMR/6832_project/trajectories/'; 

traj = load([save_dir, fname]);

%% set up hamr model
% file
urdf = fullfile(getDrakePath, 'examples', 'HAMR-URDF', 'urdf', 'HAMRSimple_scaled.urdf');

% options
options.terrain = RigidBodyFlatTerrain();
options.ignore_self_collisions = true;
options.collision_meshes = false;
options.use_bullet = false;
options.floating = true;
options.collision = true;

global hamr 

% options
options.ignore_self_collisions = true;
options.collision_meshes = false;
options.z_inactive_guess_tol = 0.1;
options.use_bullet = false;

% options to change
options.dt = .1;
gait = 'TROT';
SAVE_FLAG = 0;
ISFLOAT = true; % floating (gnd contact) or in air (not floating)

if ISFLOAT
    options.floating = ISFLOAT;
    options.collision = ISFLOAT;
    x = zeros(76, 1); x(3) = 12.69;
    options.terrain = RigidBodyFlatTerrain();
    
else
    options.floating = ISFLOAT;
    options.collision = ISFLOAT;
    x0 = zeros(64, 1);
    options.terrain = [];
end

hamr = HamrSimpleTSRBM(urdf,options);

%%
nq = hamr.getNumPositions();
nqd = hamr.getNumVelocities();
nu = hamr.getNumInputs();

%% playback traj opt
v = hamr.constructVisualizer();
tt = traj.xtraj.getBreaks();
uu = traj.utraj.eval(tt);
qq1 = traj.xtraj.eval(tt); 
qq = qq1(1:nq, :); 
qtraj_scaled = PPTrajectory(foh(tt*1e-3, qq));
qtraj_scaled = qtraj_scaled.setOutputFrame(v.getInputFrame());
% v.playback(qtraj_scaled, struct('slider', true));


%% simulate open loop
time = tt(end);
t = tt;
tsize = size(t);

u = PPTrajectory(foh(t, uu));
u = setOutputFrame(u, hamr.getInputFrame());
hamr_OL = cascade(u, hamr);

xtraj_sim_OL = simulate(hamr_OL, [0, time], hamr.getInitialState());

xtraj_scaled = DTTrajectory(xtraj_sim_OL.getBreaks()*1e-3, xtraj_sim_OL.eval(xtraj_sim_OL.getBreaks()));
xtraj_scaled = xtraj_scaled.setOutputFrame(xtraj_sim_OL.getOutputFrame());
options.slider = true;
v.playback(xtraj_scaled, options);


%% Plotting

x_guess = [hamr.getInitialState(); zeros(12,1)];
x_guess(5) = -.927;
x_upright = solve_x_fixed_point(x_guess);

% tt_sol = tt; % 
% tt_solOL = xtraj_sim_OL.getBreaks();

tt_sol = xtraj_sim_OL.getBreaks();
xx_sol_OL = xtraj_sim_OL.eval(tt_sol);
xx_sol_trajopt = traj.xtraj.eval(tt_sol);
u_OL = uu;


% lp_b = [0 0 -14.988382167532292; 
%     0 0 -14.988382167532292;
%     0 0 -14.988382167532292;
%     0 0 -14.988382167532292];
% 
% 
% 
% act_dof = hamr.getActuatedJoints();
% ndof = hamr.getNumDiscStates();
% title_str = {'Front Left Swing', 'Front Left Lift', ...
%     'Rear Left Swing', 'Rear Left Lift', ...
%     'Front Right Swing', 'Front Right Lift', ...
%     'Rear Rear Swing', 'Rear Rear Lift'};
% 
% figure(1); clf; hold on;
% title('Traj Opt Chosen Inputs')
% for i = 1:numel(act_dof)
%     subplot(4,2,i); hold on; title(title_str{i})
%     plot(tt, u_OL(i,:));
%     legend('Inputs');
% end
% 
% 
% 
% figure(2); clf; hold on;
% for i = 1:numel(act_dof)
%     subplot(4,2,i); hold on; title(title_str{i})
%     plot(tt_sol, xx_sol_OL(act_dof(i), :));
%     plot(tt_sol, xx_sol_trajopt(act_dof(i), :));
% %     plot(ttdN, xxdN(act_dof(i), :));
%     legend('OL Act Defl','Traj Opt Act Defl');
% end
ttsize=size(tt_sol);
if options.floating
    figure(3); clf;
    title_str = {'Com-X(mm)', 'Com-Y(mm)', 'Com-Z(mm)', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)'};
    for i = 1:6
        subplot(2,3,i); hold on; ylabel(title_str(i), 'FontSize', 18)
        xlabel('Time(s)', 'FontSize', 18)
        these = x_upright(1:6);
        if i > 3
            plot(tt_sol*1e-3, rad2deg(xx_sol_OL(i,:)), 'LineWidth', 1.5);
            plot(tt_sol*1e-3, rad2deg(xx_sol_trajopt(i,:)),'LineWidth',1.5);
            plot(tt_sol*1e-3, (ones(ttsize)*rad2deg(x_upright(i))),'--b');
%             reflin = refline(0,0);%rad2deg(x_upright(i)));
%             reflin.Color = 'r';
%             reflin.LineStyle = '--';
            lh = legend('OL','Traj Opt','Desired');
            set(lh, 'box', 'off')
        end
        if i < 4
            plot(tt_sol*1e-3, xx_sol_OL(i,:), 'LineWidth', 1.5);
            plot(tt_sol*1e-3, xx_sol_trajopt(i,:),'LineWidth',1.5);
%             reflin1 = refline(0,x_upright(i));
%             reflin1.Color = 'r';
%             reflin1.LineStyle = '--';
            lh = legend('OL','Traj Opt');
            set(lh, 'box', 'off')
        end
    end    
end

figure(4); clf;
subplot(2,3,1); hold on; ylabel('COM-X', 'Fontsize',18);
plot(tt_sol*1e-3, rad2deg(xx_sol_OL(1,:)), 'LineWidth', 1.5);
plot(tt_sol*1e-3, rad2deg(xx_sol_trajopt(1,:)),'LineWidth',1.5);
plot(tt_sol*1e-3, (ones(ttsize)*rad2deg(x_upright(1))),'--b');
lh = legend('OL','Traj Opt','Desired');
set(lh, 'box', 'off')

subplot(2,3,2); hold on; ylabel('COM-Y', 'Fontsize',18);
plot(tt_sol*1e-3, rad2deg(xx_sol_OL(2,:)), 'LineWidth', 1.5);
plot(tt_sol*1e-3, rad2deg(xx_sol_trajopt(2,:)),'LineWidth',1.5);
plot(tt_sol*1e-3, (ones(ttsize)*rad2deg(x_upright(2))),'--b');
lh = legend('OL','Traj Opt','Desired');
set(lh, 'box', 'off')


% pfCL = zeros([numel(tt_sol), size(lp_b')]);
% pfDesW = zeros([numel(tt_sol), size(lp_b')]);
% 
% legs = {'FL2','RL2','FR2','RR2'};
% fkopt.base_or_frame_id = hamr.findLinkId('Chassis');
% 
% for j = 1:numel(tt_sol)
% 
%     qCL = xx_sol_OL(1:ndof/2, j);
%     qdCL = xx_sol_OL(ndof/2+1: ndof, j);
%     kinsolCL = hamr.doKinematics(qCL, qdCL);
%     for i = 1:size(lp_b,1)
%         pfCL(j,:,i) = hamr.forwardKin(kinsolCL, hamr.findLinkId(legs{i}), lp_b(i,:)'); %,fkopt);
%     end
% end
