%% contact
clear; clc; close all; 
addpath('../')
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
global nq
global nqd
global nu
global nc
global nx
global q_inds
global qd_inds
global u_inds
global c_inds

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

nq = hamr.getNumPositions();
nqd = hamr.getNumVelocities();
nu = hamr.getNumInputs();
nc = hamr.getNumContactPairs();
nx = nq + nqd;

q_inds = 1:nq;
qd_inds = nq+(1:nqd);
u_inds = nq + nqd + (1:nu);
c_inds = nq + nqd + nu + (1:nc); 

v = hamr.constructVisualizer();

%% solve for upright fixed point

x_guess = [hamr.getInitialState(); zeros(12,1)];
x_guess(5) = -.927;
%[x_upright,objval,exitflag,infeasible_constraint_name] = solve_x_fixed_point(x_guess)
x_upright = solve_x_fixed_point(x_guess);


%%
%v.inspector(x_upright(1:28));

%% find linearized A and B
[A, B] = new_solve_A_B(x_upright);

%% set up costs and find u

% these gains work
Q = eye(nx,nx); 
Q(5,5) = 30.0;
Q(3,3) = 100.0;
Q(4,4) = 100.0;
Q(1,1) = 0;
Q(2,2) = 0;
R = eye(nu);

% test gains
% Q = zeros(nx);
% Q(1,1) = 0;
% Q(2,2) = 0;
% Q(3,3) = 100.0;
% Q(4,4) = 100.0;
% Q(5,5) = 100.0;
% Q(6,6) = 0;
% R = eye(nu);

[X,L,G] = dare(A,B,Q,R);

x_desired = x_upright(1:nx);
x_initial = x_desired;
% x_initial(5) = x_initial(5)+1e-6; %10e-5 too big, 1e-6 good
u0 = x_upright(u_inds);
% u0([1,5]) = .01; %doesnt work u0([1,5]) = -.01; %doesnt stabilize
% u0([2,6])=-.01; %worked but no longer works -- because i changed mu
% u0([2,6])=-.05; % doesnt stabilize


%% add front leg perturbation
time = 800;
% t = 0:options.dt:time;
% tsize = size(t);
% uu = zeros(8,tsize(2));
% uu([1:2, 5:6],1) = 0.1; %(rand(4, tsize(2))-0.5);
% 
% u = PPTrajectory(foh(t, uu));
% u = setOutputFrame(u, hamr.getInputFrame());
% hamr_OL = cascade(u, hamr);


%% simulate
LegTracking = LQR(hamr, u0, x_desired, G); 

% mimo outputs
output_select(1).system = 1;
output_select(1).output = hamr.getOutputFrame.getFrameByName('HamrPosition');
output_select(2).system = 1;
output_select(2).output = hamr.getOutputFrame.getFrameByName('HamrVelocity');
output_select(3).system = 2;
output_select(3).output = LegTracking.getOutputFrame();

% hamrWact_CL = mimoFeedback(hamrWact, PDTracking); 
hamr_CL = mimoFeedback(hamr, LegTracking, [], [], [], output_select); 
xtraj_sim_CL = simulate(hamr_CL, [0, time], x_initial);

xtraj_scaled = DTTrajectory(xtraj_sim_CL.getBreaks()*1e-3, xtraj_sim_CL.eval(xtraj_sim_CL.getBreaks()));
xtraj_scaled = xtraj_scaled.setOutputFrame(xtraj_sim_CL.getOutputFrame());
options.slider = true;
v.playback(xtraj_scaled, options);



%% Plotting

lp_b = [0 0 -14.988382167532292; 
    0 0 -14.988382167532292;
    0 0 -14.988382167532292;
    0 0 -14.988382167532292];

tt_sol = xtraj_sim_CL.getBreaks();
yy_sol_CL = xtraj_sim_CL.eval(tt_sol);

xx_sol_CL = yy_sol_CL(1:nq+nqd, :); 

vv_sol_CL = yy_sol_CL(nq+nqd+(1:nu),:); 

act_dof = hamr.getActuatedJoints();
ndof = hamr.getNumDiscStates();
title_str = {'Front Left Swing', 'Front Left Lift', ...
    'Rear Left Swing', 'Rear Left Lift', ...
    'Front Right Swing', 'Front Right Lift', ...
    'Rear Rear Swing', 'Rear Rear Lift'};

figure(1); clf; hold on;
set(gca,'fontsize',20)
for i = 1:numel(act_dof)
    subplot(4,2,i); hold on; title(title_str{i})
    plot(tt_sol, vv_sol_CL(i,:));
    legend('Inputs');
    xlabel('Time(s)', 'FontSize', 15)

end



figure(2); clf; hold on;
set(gca,'fontsize',15)
for i = 1:numel(act_dof)
    subplot(4,2,i); hold on; title(title_str{i})
    plot(tt_sol, xx_sol_CL(act_dof(i), :));
%     plot(ttdN, xxdN(act_dof(i), :));
    legend('CL Act Defl');
end

if options.floating
    figure(3); clf;
    set(gca,'fontsize',15)
    title_str = {'Com-X(mm)', 'Com-Y(mm)', 'Com-Z(mm)', 'Roll(deg)', 'Pitch(deg)', 'Yaw(deg)'};
    for i = 1:6
        subplot(2,3,i); hold on; ylabel(title_str(i), 'FontSize', 15)
        xlabel('Time(s)', 'FontSize', 15)
        if i > 3
            plot(tt_sol*1e-3, rad2deg(xx_sol_CL(i,:)), 'LineWidth', 1.5); 
            reflin = refline(0,rad2deg(x_upright(i)));
            reflin.Color = 'r';
            lh = legend('Actual','Desired');
            set(lh, 'box', 'off')
        else
            plot(tt_sol*1e-3, xx_sol_CL(i,:), 'LineWidth', 1.5); 
            reflin = refline(0,x_upright(i));
            reflin.Color = 'r';
            lh = legend('Actual','Desired');
            set(lh, 'box', 'off')
        end
    end    
end

pfCL = zeros([numel(tt_sol), size(lp_b')]);
pfDesW = zeros([numel(tt_sol), size(lp_b')]);

legs = {'FL2','RL2','FR2','RR2'};
fkopt.base_or_frame_id = hamr.findLinkId('Chassis');

for j = 1:numel(tt_sol)

    qCL = xx_sol_CL(1:ndof/2, j);
    qdCL = xx_sol_CL(ndof/2+1: ndof, j);
    kinsolCL = hamr.doKinematics(qCL, qdCL);
    for i = 1:size(lp_b,1)
        pfCL(j,:,i) = hamr.forwardKin(kinsolCL, hamr.findLinkId(legs{i}), lp_b(i,:)'); %,fkopt);
    end
    

    xi = x_desired;
    qi = xi(1:ndof/2);
    qdi =  xi(ndof/2+1:ndof);
    kinsol0 = hamr.doKinematics(qi, qdi);
    for i = 1:size(lp_b,1)
        pfDesW(j,:,i) = hamr.forwardKin(kinsol0, hamr.findLinkId(legs{i}), lp_b(i,:)'); %,fkopt);
    end
end

figure(4); clf; hold on;
set(gca,'fontsize',20)
nc = size(lp_b,1);

for i = 1:nc

    subplot(nc,3,3*(i-1)+1); hold on; ylabel('Foot X','FontSize', 15); title(legs{i})
    plot(tt_sol, pfCL(:,1,i))
    plot(tt_sol, pfDesW(:,1,i));
    xlabel('Time(s)', 'FontSize', 15)

%     legend('Leg Position', 'Desired Leg Position');

    subplot(nc,3,3*(i-1)+2); hold on; ylabel('Foot Y','FontSize', 15); title(legs{i})
    plot(tt_sol, pfCL(:,2,i));
    plot(tt_sol, pfDesW(:,2,i));
    xlabel('Time(s)', 'FontSize', 15)
%     legend('Leg Position', 'Desired Leg Position');

    subplot(nc,3,3*(i-1)+3); hold on;  ylabel('Foot Z','FontSize', 15); title(legs{i})
    plot(tt_sol, pfCL(:,3,i)); %ylim([0, 1.2*max(pfFullW(:,3,i))]);
    plot(tt_sol, pfDesW(:,3,i));% ylim([0, 1.2*max(pfSimpleW(:,3,i))]);
    legend('Actual', 'Desired');
    xlabel('Time(s)', 'FontSize', 15)

end

