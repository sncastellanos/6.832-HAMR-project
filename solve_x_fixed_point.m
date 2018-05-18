function x_fixed_point = solve_x_fixed_point(x_guess)

global hamr 
global nq
global nqd
global nu
global nc
global q_inds
global qd_inds
global u_inds
global c_inds 

num_vars = nq+ nqd + nu + nc;

x_name = cell(num_vars, 1);

for i = 1:num_vars
    x_name{i} = sprintf('a[%d]',i);
end

nlp = NonlinearProgram(num_vars, x_name);

nlp = nlp.setSolver('snopt');
nlp = nlp.setSolverOptions('snopt','MajorIterationsLimit',10000);
nlp = nlp.setSolverOptions('snopt','MinorIterationsLimit',200000);
nlp = nlp.setSolverOptions('snopt','IterationsLimit',5000000);
nlp = nlp.setSolverOptions('snopt','SuperbasicsLimit',1000);

tol = 1e-6;
nlp = nlp.setSolverOptions('snopt','MajorOptimalityTolerance',tol);
nlp = nlp.setSolverOptions('snopt','MinorOptimalityTolerance',tol);
nlp = nlp.setSolverOptions('snopt','MajorFeasibilityTolerance',tol);
nlp = nlp.setSolverOptions('snopt','MinorFeasibilityTolerance',tol);
nlp = nlp.setSolverOptions('snopt','constraint_err_tol',tol);


% Manipulator Constraint
manip_const = FunctionHandleConstraint(zeros(nq, 1), zeros(nq, 1), num_vars-nq, ...
    @manipulator_constraint);
manip_const = manip_const.setName('manipulator_constraint');
nlp = nlp.addConstraint(manip_const, {q_inds'; u_inds'; c_inds'});

% body z must be greater than zero constraint
desired_z_min = 0.0;
desired_z_max = inf;
nlp = nlp.addConstraint(BoundingBoxConstraint(desired_z_min, desired_z_max), q_inds(3));

% Set qd = zero
desired_qd = zeros(nqd,1);
nlp = nlp.addConstraint(ConstantConstraint(desired_qd), qd_inds);

% set u = zero
desired_u = zeros(nu,1);
nlp = nlp.addConstraint(ConstantConstraint(desired_u), u_inds);


% set front two contact forces to zero c(1) c(3)
desired_front_contact_force = 0.0;
nlp = nlp.addConstraint(ConstantConstraint(desired_front_contact_force),c_inds(1));
nlp = nlp.addConstraint(ConstantConstraint(desired_front_contact_force),c_inds(3));

% add bounding constraint on pitch
desired_pitch = pi;
nlp = nlp.addConstraint(BoundingBoxConstraint(-desired_pitch, desired_pitch), q_inds(5));

% add constraint saying the back contact points need to be on the ground
% Foot z position Constraint
foot_z_desired = zeros(2,1);
foot_const = FunctionHandleConstraint(foot_z_desired, foot_z_desired, nq, ...
    @foot_constraint);
foot_const = foot_const.setName('foot_constraint');
nlp = nlp.addConstraint(foot_const, {q_inds'});

% Bounding box Constraints: 
% normal force >= zero 
normal_force_min = zeros(nc,1);
normal_force_max = inf*ones(nc,1);
nlp = nlp.addConstraint(BoundingBoxConstraint(normal_force_min, normal_force_max), c_inds);

% solve
%[x_fixed_point,objval,exitflag,infeasible_constraint_name] = nlp.solve(x_guess);
x_fixed_point = nlp.solve(x_guess);

%%
    function [f, df] = manipulator_constraint(q, u, c)
        
        % Contact basis, using kinematics, solve for n and dn for
        % the contact points
        kinopts = struct();
        kinopts.compute_gradients = true;
        kin = hamr.doKinematics(q, 0*q, kinopts);
        [~,~,~,~,~,~,~,~,n,~,dn,~] = hamr.contactConstraints(kin);
        
        % Manipulator Dynamics: solve for C, B, dC, dB
        [~,C,B,~,dC,dB] = hamr.manipulatorDynamics(q, 0*q);

        % form f(z) and derivs
        f = B*u + n'*c - C;
        
        % Solve for df(z)/dq and then df
        df_dq = kron(u', eye(nqd))*dB(:,1:nq) + kron(c',eye(nq))*comm(nc, nq)*dn ...
            - dC(:,1:nq);
        
        % df/dz = [df_dq, df/du, df/dc]  (df/dqd = 0)
        df = [df_dq, B, n'];

    end

    function [footz, dfootz] = foot_constraint(q)

        legs = {'FL2','RL2','FR2','RR2'};

        pf = [0 0 -14.988382167532292]'; % position of foot in local frame    

        kinsol = hamr.doKinematics(q, 0*q);
        [footRL,Jrl] = hamr.forwardKin(kinsol, hamr.findLinkId(legs{2}), pf); 
        [footRR,Jrr] = hamr.forwardKin(kinsol, hamr.findLinkId(legs{4}), pf); 
        
        footz = [footRL(3); footRR(3)];
        dfootz = [Jrl(3,:); Jrr(3,:)];
        
    end 

end
