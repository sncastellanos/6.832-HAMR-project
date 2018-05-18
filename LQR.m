classdef LQR < DrakeSystem
    
    
    properties (SetAccess=private)
        robot;          % to be controlled
        nq;             % number of positions
        nv;             % number of velocities
        qa;             % active dof
        x_des;          % desired position/velocity
        u0;             % feed forward voltage
        K;              % full state feedback matrix
    end
    
    
    methods
        function obj = LQR(r, u0, x_des,K)
            % @param r rigid body manipulator instance
            
            input_frame = r.getOutputFrame();
            output_frame = r.getInputFrame();
            
            obj = obj@DrakeSystem(0,0,r.getNumOutputs,r.getNumInputs,true,true);
            
            obj = setInputFrame(obj,input_frame);
            obj = setOutputFrame(obj,output_frame);
            
            obj.robot = r;
            obj.K = K;
            obj.nq = r.getNumPositions();
            obj.nv = r.getNumVelocities();
            obj.qa = r.getActuatedJoints();
            obj.u0 = u0;
            obj.x_des = x_des;
        end
        
     
        function u = output(obj, t, ~, x)
            
            dim = 3; % 3D
            uff_t = obj.u0;      % feedforward control
            nq = obj.nq;
            nv = obj.nv;            

            % desired actuator state
            x_des_t = obj.x_des;
            q_des_t = x_des_t(1:nq);
            qd_des_t = x_des_t(nq+(1:nv));       
            
            % current actuator state
            q_t = x(1:nq);
            qd_t = x(nq+(1:nv));
                    
            q_err_t = q_t - q_des_t;
            qd_err_t = qd_t - qd_des_t;  
            x_err_t = [q_err_t; qd_err_t];
            K_t = obj.K;                     

            u = uff_t -  K_t * x_err_t(:);

            
            
% %             Input Limits
%             u(u < 0.15) = 0.15;
%             u(u > -0.15) = -0.15; 
            
        end
        
        
    end
    
end