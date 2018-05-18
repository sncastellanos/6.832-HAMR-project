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
x_upright = solve_x_fixed_point(x_guess);


%% call swing up traj opt

trial_name = 'TrajOptSwingUp8';
save_dir = '~/dev/drake-var-hamr/drake/examples/HAMR-URDF/dev/SimpleHAMR/6832_project/trajectories/'; 

[hamr,xtraj,utraj,ctraj,btraj,...
    psitraj,etatraj,jltraj, kltraj, straj, ...
    z,F,info,infeasible_constraint_name] = SimpleHAMRSwingUpTrajOpt(x_upright(1:nx));

save([save_dir, trial_name], 'xtraj', 'utraj', 'ctraj', 'btraj', 'psitraj', 'etatraj', ...
    'jltraj', 'kltraj', 'straj')

%% list of saved trajectories

% TrajOptSwingUp4: best one yet! all slack is .000001, all dynamics = 0, N=21
    % based off of first traj found -- looks a little weird. use this as an
    % initial traj and double the number of knot points
% TrajOptSwingUp5: used #4 as initial condition, but with double N, N=41,
    % stopped with j, not all slack
    % was at 1e-6, dynamics were between all zeros and 0.000008
% #6 stopped with 43 constrains it cant satisfy
% #7 solved, but rear right leg looks weird, too close to the body, N=41
    % state_cost.base_y = 1;
%     state_cost.base_z = 1;                       
%     state_cost.base_pitch = 1;        
%     state_cost.base_roll = 1;           
%     state_cost.base_yaw = 10; rest are zeros           


