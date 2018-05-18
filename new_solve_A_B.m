function [A,B] = new_solve_A_B(z0)

%% setting stuff up
urdf = fullfile(getDrakePath, 'examples', 'HAMR-URDF', 'urdf', 'HAMRSimple_scaled.urdf');

%setting up space and the hamr time steppting RBM
global nu
global q_inds
global qd_inds
global u_inds
global nx

% options
options.ignore_self_collisions = true;
options.collision_meshes = false;
options.z_inactive_guess_tol = 0.1;
options.use_bullet = false;

% options to change
options.dt = 1;
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

hamrTS = HamrSimpleTSRBM(urdf, options);

%% solving for A and B

u = z0(u_inds);
x = [z0(q_inds);z0(qd_inds)];

t = 0;
dx = 1e-6; % set up dx and u pertubations
du = 1e-6;

df_dx = zeros(nx,nx);

for i = drange(1:nx) 
    x_plus = x;
    x_minus = x;
    x_plus(i) = x(i) + dx;
    x_minus(i) = x(i) - dx;
    x_ts_plus = hamrTS.update(t,x_plus,u); % get next state with +dx on the ith x
    x_ts_minus = hamrTS.update(t,x_minus,u); % get next state with -dx on the ith x
    df_dx(:,i) = (x_ts_plus - x_ts_minus)/(2*dx); % assemble df_dx
end

df_du = zeros(nx,nu);

for i = drange(1,nu)
    u_plus = u;
    u_minus = u;
    u_plus(i) = u(i) + du;
    u_minus(i) = u(i) - du;
    x_ts_plus_u = hamrTS.update(t,x,u_plus); % get next state with +du on the ith u
    x_ts_minus_u = hamrTS.update(t,x,u_minus); % get next state with -du on the ith u
    df_du(:,i) = (x_ts_plus_u - x_ts_minus_u)/(2*du); % assemble df_du
end
    
    
A = df_dx;
B = df_du;

%% Check controllability
% controllability_matrix = ctrb(A,B);
% sizeA = size(A);
% if rank(controllability_matrix) == sizeA(1)
%     disp('Controllable')
% end

end
