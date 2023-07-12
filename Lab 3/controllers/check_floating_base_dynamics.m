function floating_dynamics = check_floating_base_dynamics(model, x)
    %% System State

    % Extract generalized positions and velocities
    q = x(1:10);    % [10x1] Generalized coordinates [q_b, q_F, q_H, q_A]'
    dq = x(11:20);  % [10X1] Generalized velocities
    
    %% Model Parameters 

    % Extract dynamics at current state
    params = model.parameters.values;
    floating_dynamics.M = model.dynamics.compute.M(q,dq,[],[],params); % [10X10] Inertia matrix
    floating_dynamics.b = model.dynamics.compute.b(q,dq,[],[],params); % [10X1] Nonlinear-dynamics vector
    floating_dynamics.g = model.dynamics.compute.g(q,dq,[],[],params); % [10X1] Gravity vector
    floating_dynamics.S = [zeros(7, 3), eye(7)];                       % [7x10] Selection matrix

    % Get Jacobians and Derivatives at current state
    floating_dynamics.J_B  = eval_jac(model.body( 1).kinematics.compute.I_J_IBi, q, [], params);    % [3X10] body position and orientation Jacobian
    floating_dynamics.J_FF = eval_jac(model.body( 7).kinematics.compute.I_J_IBi, q, [], params);	% [3X10] Front foot position and orientation Jacobian
    floating_dynamics.J_HF = eval_jac(model.body( 4).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Hind foot position and orientation Jacobian
    floating_dynamics.J_EE = eval_jac(model.body(11).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Arm End-effector position and orientation Jacobian
    floating_dynamics.dJ_B  = eval_jac(model.body( 1).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3X10] body position and orientation Jacobian derivative
    floating_dynamics.dJ_FF = eval_jac(model.body( 7).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3X10] Front foot position and orientation Jacobian derivative
    floating_dynamics.dJ_HF = eval_jac(model.body( 4).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3x10] Hind foot position and orientation Jacobian derivative
    floating_dynamics.dJ_EE = eval_jac(model.body(11).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3x10] Arm End-effector position and orientation Jacobian derivative

    % Extract forward kinematics of the base
    floating_dynamics.p_B = q(1:3);
    floating_dynamics.w_B = dq(1:3);

    % Extract forward kinematics of the end-effector
    floating_dynamics.T_I_EE = model.body(11).kinematics.compute.T_IBi(q, [], [], params); % [4x4] Homogeneous transfrom from inertial to EE frame
    floating_dynamics.p_EE = [floating_dynamics.T_I_EE(1,4); floating_dynamics.T_I_EE(3,4); acos(floating_dynamics.T_I_EE(1,1))]; % [3x1] pose of the EE
    floating_dynamics.w_EE = floating_dynamics.J_EE(1:3,:)*dq; % [3x1] xz twist of the EE

    % Assemble constraint Jacobian -> Only constrain linear velocity at feet
    floating_dynamics.J_c = [floating_dynamics.J_FF(1:2,:) ; floating_dynamics.J_HF(1:2,:)];
    floating_dynamics.dJ_c = [floating_dynamics.dJ_FF(1:2,:); floating_dynamics.dJ_HF(1:2,:)];

    % Reduce end-effector Jacobian to only contain linear velocity terms
    floating_dynamics.J_EE = floating_dynamics.J_EE(1:3,:);
    floating_dynamics.dJ_EE = floating_dynamics.dJ_EE(1:3,:);
end
function jac = eval_jac(f, q, dq, params)
jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end