function [hamr,xtraj,utraj,ctraj,btraj,...
    psitraj,etatraj,jltraj, kltraj, straj, ...
    z,F,info,infeasible_constraint_name] = SimpleHAMRSwingUpTrajOpt(x_upright,traj_init)

% file
urdf = fullfile(getDrakePath, 'examples', 'HAMR-URDF', 'urdf', 'HAMRSimple_scaled.urdf');

% options
options.terrain = RigidBodyFlatTerrain();
options.ignore_self_collisions = true;
options.collision_meshes = false;
options.use_bullet = false;
options.floating = true;
options.collision = true;

hamr = HAMRSimpleRBM(urdf,options);
v = hamr.constructVisualizer();

% state/input dimenisons
nq = hamr.getNumPositions();
nv = hamr.getNumVelocities();
nx = nq+nv;
nu = hamr.getNumInputs();

xf=x_upright(1:nx);

% % --- Set Input limits ---
Vlim = inf; %100;                     % set max torque
V2Tau = 0.0013; 
umin = -Vlim*V2Tau*ones(nu,1);
umax = Vlim*V2Tau*ones(nu, 1);

% --- Initialize TrajOpt---
optimoptions.s_weight = 20; % slack variable weight
optimoptions.joint_limit_collisions = false; 
optimoptions.add_ccost = true; 

% ---- Initial Guess ----
fname = 'TrajOptSwingUp7';
save_dir = '~/dev/drake-var-hamr/drake/examples/HAMR-URDF/dev/SimpleHAMR/6832_project/trajectories/'; 
traj0 = load([save_dir, fname]);
t_init = traj0.xtraj.getBreaks(); 

N = 1*(numel(t_init)-1)+1; 
if N < numel(t_init)
    T = t_init(N); 
else 
    T = t_init(end);
end

N=41;

t_init = linspace(0, T, N); 
x0 = traj0.xtraj.eval(t_init(1)); 
q0 = x0(1:nq);
traj_init.x = traj0.xtraj;
traj_init.u = traj0.utraj;
traj_init.c = traj0.ctraj;
traj_init.b = traj0.btraj;
traj_init.eta = traj0.etatraj;
traj_init.psi = traj0.psitraj;
traj_init.s = traj0.straj;

% -- Initialize Traj Opt ---% 

% T = 20;
T2 = T + 50;
% N=41;

T_span = [T T2]; 
traj_opt = VariationalTrajectoryOptimization(hamr,N,T_span,optimoptions);
 
% -- State Costs ---%
state_cost = Point(getStateFrame(hamr),ones(nx,1));

state_cost.base_x = 0;
state_cost.base_y = 1;
state_cost.base_z = 1;                       
state_cost.base_pitch = 1;           
state_cost.base_roll = 1;            
state_cost.base_yaw = 1;          

state_cost.FL_lift = 0; 
state_cost.FL_swing = 0;
state_cost.RL_lift = 1; 
state_cost.RL_swing = 0;
state_cost.FR_lift = 0; 
state_cost.FR_swing = 0; 
state_cost.RR_lift = 1; 
state_cost.RR_swing = 0;

state_cost = double(state_cost);
Q = diag(state_cost); 

% --- Cost Functions ---%
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
traj_opt = traj_opt.addFinalCost(@final_cost_fun); 

% -- State Constraints ---%
[qmin, qmax] = hamr.getJointLimits(); 
traj_opt = traj_opt.addPositionConstraint(BoundingBoxConstraint(qmin, qmax),1:N);
traj_opt = traj_opt.addPositionConstraint(ConstantConstraint(q0),1);  
traj_opt = traj_opt.addVelocityConstraint(ConstantConstraint(zeros(nv,1)),1);


% Input Constraint 
traj_opt = traj_opt.addInputConstraint(BoundingBoxConstraint(umin, umax),1:N-1);
traj_opt = traj_opt.addTrajectoryDisplayFunction(@displayTraj);


% Solver options
traj_opt = traj_opt.setSolver('snopt');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',10000);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',10000000);
traj_opt = traj_opt.setSolverOptions('snopt','SuperbasicsLimit',1000);
traj_opt = traj_opt.setSolverOptions('snopt','MajorOptimalityTolerance',1e-6);
traj_opt = traj_opt.setSolverOptions('snopt','MinorOptimalityTolerance',1e-6);
traj_opt = traj_opt.setSolverOptions('snopt','MajorFeasibilityTolerance',1e-6);
traj_opt = traj_opt.setSolverOptions('snopt','MinorFeasibilityTolerance',1e-6);
traj_opt = traj_opt.setSolverOptions('snopt','constraint_err_tol',1e-6);

disp('Solving...')
tic
[xtraj,utraj,ctraj,btraj,psitraj,etatraj,jltraj,kltraj,straj ...
    ,z,F,info,infeasible_constraint_name] = solveTraj(traj_opt,t_init,traj_init);
toc

    function [f,df] = running_cost_fun(h,x,u)
        ugain = 10*Vlim*V2Tau; 
        R = eye(nu)*0.01; % (1/ugain)^2*eye(nu);
        f = (1/2)*(x-xf)'*Q*(x-xf) + (1/2)*u'*R*u;
        df = [0,(x-xf)'*Q,u'*R];
    end

    function [f,df] = final_cost_fun(tf,x)
        a = 10;
        f = (a/2)*(x-xf)'*Q*(x-xf);
        df = [0,a*(x-xf)'*Q];
    end

    function displayTraj(h,x,u)
        disp('Displaying Trajectory...')
        h = h/1e3;
        ts = [0;cumsum(h)];
        for i=1:length(ts)
            v.drawWrapper(0,x(:,i));
            pause(5*h(1));
        end
        
    end
end
