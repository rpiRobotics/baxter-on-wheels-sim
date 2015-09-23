function BOWDualArmKeyboardControl()

    % define robot and total structure
    gripper_aperture = 0.11;
    left_gripper_options.param = struct('aperture',gripper_aperture,  ...
                                                    'height',0.06);
    left_gripper_options.props = {};
    right_gripper_options.param = struct('aperture',gripper_aperture,  ...
                                                    'height',0.06);
    right_gripper_options.props = {};
    [bow_const, bow_structure] = defineBaxterOnWheels(...
                                'LeftGripper', left_gripper_options,  ...
                                'RightGripper', right_gripper_options);
    % Create Scene
    fig = figure; clf;
    bow = createCombinedRobot(bow_const, bow_structure);

    % Get angle structure
    q = get_angle_structure(bow);
    % indices to particular robots
    LEFT_ARM = strcmpi({q.name},'baxter_left_arm');
    RIGHT_ARM = strcmpi({q.name},'baxter_right_arm');
    LEFT_GRIPPER = ~cellfun('isempty',strfind({q.name},'left_gripper'));
    RIGHT_GRIPPER = ~cellfun('isempty',strfind({q.name},'right_gripper'));
    WHEELCHAIR = strcmpi({q.name},'wheelchair');
    % gripper kinematic constants
    left_gripper = bow.robots(LEFT_GRIPPER);
    right_gripper = bow.robots(RIGHT_GRIPPER);
    % relationship to centers of grippers
    pLGl_L = 1/2*(left_gripper(1).frames(end).t + ...
                    left_gripper(2).frames(end).t);
    pRGr_R = 1/2*(right_gripper(1).frames(end).t + ...
                    right_gripper(2).frames(end).t);

    
    % set initial pose
    q(LEFT_ARM).state = [-pi/12 -pi/6 -pi/3 pi/2 0 -pi/4 -pi/4];
    q(RIGHT_ARM).state = [pi/12 -pi/6 pi/3 pi/2 0 -pi/4 pi/4];
    bow = updateRobot(q,bow);
    drawnow;
    
    
    % full body kinematic structures
    left_arm_kin = get_kinematic_chain(bow, 'wheelchair', ...
                                            'baxter_left_arm');
    right_arm_kin = get_kinematic_chain(bow, 'wheelchair', ...
                                            'baxter_right_arm');
    % Initial forward kinematics
    [ROL, pOL_O] = fwdkin(left_arm_kin,[q(WHEELCHAIR).state ...
                                        q(LEFT_ARM).state]);
    [ROR, pOR_O] = fwdkin(right_arm_kin,[q(WHEELCHAIR).state ...
                                        q(RIGHT_ARM).state]);
    
    % Define grasped object
    plug_radius = 0.03;
    plug_handle = createCylinder(rot([1;0;0],pi/2),[0;0;0], ...
                                struct('radius',plug_radius, ...
                                        'height',0.3), ...
                                'FaceColor',[0;0;1]);
    plug_insert = createCylinder(rot([0;1;0],pi/2),[0.2;0;0], ...
                                struct('radius',plug_radius, ...
                                        'height',0.4), ...
                                'FaceColor',[0;0;1]);
    plug = combineRigidBodies(plug_handle,plug_insert);
    pOT_O = 1/2*(pOL_O + ROL*pLGl_L + pOR_O + ROR*pRGr_R);
    plug = updateRigidBody(eye(3),pOT_O, plug);
    
    port_stand = createCylinder(rot([0;1;0],pi/2),[0.405;0;0], ...
                                struct('radius',10*plug_radius, ...
                                        'height',0.01), ...
                                'FaceColor',[0.2;0.2;0.2]);
    port = createCylinder(rot([0;1;0],pi/2),[0.2;0;0], ...
                                struct('radius',2*plug_radius, ...
                                        'height',0.4), ...
                                'FaceColor',[0;1;0], ...
                                'FaceAlpha',0.5, ...
                                'EdgeAlpha', 0);
    port = combineRigidBodies(port_stand,port);
    port = updateRigidBody(eye(3),[2.5;-1;1],port);
    
    % close grippers on object
    [q(LEFT_GRIPPER).state] = deal(gripper_aperture/2 - plug_radius);
    [q(RIGHT_GRIPPER).state] = deal(gripper_aperture/2 - plug_radius);
    bow = updateRobot(q,bow);
    drawnow;
    % Attach object to an arm as a load
    bow = graspLoad(plug, bow, bow.robots(LEFT_ARM).name);
    
    axis equal; axis([-1 3 -2 2 0 2]); grid on;
        
    dt = 0.05;

    % constants for inequality bound sigma(h_I)
    c_b = 0.9; eta_b = 0.15; epsilon_b = 0.15; e_b = 0.01;
    c_sc = 0.9; eta_sc = 0.1; epsilon_sc = 0.15; e_sc = 0.01;
    % relative weights
    lambda1 = 1;
    lambda2 = 0.1;
    % feedback gain for equality constraint
    kE = 1.9;
    % Selection matrices
    % Selection matrix for wheelchair command
    C = [eye(2) zeros(2,7) zeros(2,7)]; 
    % Selection matrix for constrained dof
    Sfull = eye(6);
    % Selection matrix for human-controlled dof
    SH = Sfull(4:6,:); 

    % Want to maintain rigid coupling on grasped object and initial
    % orientation
    RBLd = bow.robots(LEFT_ARM).frames(end).R;
    RBRd = bow.robots(RIGHT_ARM).frames(end).R;
    pLR_Ld = ROL'*(pOR_O - pOL_O);

    % joint boundaries as suggested by Lu's TASE paper
    lower_joint_limit = [-pi/4 -pi/3 -2*pi/3 pi/2 -pi/2 -pi/2 -pi/2];
    upper_joint_limit = [pi/2 pi/3 -pi/4 2*pi/3 pi/2 pi/2 pi/2];
    
    h = zeros(30,1);        % 14 joints have upper and lower limits + 
                            % 2 simple self collision preventions
                            % Lu's code has additional obstacle avoidance. 
                            %(# objects additional inequality constraints)
    sigma = zeros(size(h)); % boundary values
    % Partial derivative of inequality constraints with respect to q.  
    % Joint limits are simply a positive and negative identity
    dhdq = [zeros(14,3) eye(14); ...
            zeros(14,3) -eye(14); ...
            zeros(2,17)];
    dqdu = zeros(17,16); % Jacobian relating input to state
    dqdu(4:end,3:end) = eye(14);

    % Options set for MATLAB quadratic programming solver
    quadprog_options = optimoptions('quadprog','Display','none');
    
    % Set figure to listen for key press events
    set(fig,'WindowKeyPressFcn',{@wkp});
    set(fig,'WindowKeyReleaseFcn',{@wkr});
    
    % Structure for user input
    user_input = struct('velocity_command', [0;0;0], ...
                        'exit', false(1), ...
                        'grasp_command', false(1), ...
                        'release_command', false(1));

    while ~user_input.exit

        %%% Forward Kinematics
        [ROB, pOB_O] = fwdkin(bow.robots(WHEELCHAIR).kin, q(WHEELCHAIR).state);
        [ROL, pOL_O] = fwdkin(left_arm_kin,[q(WHEELCHAIR).state ...
                                            q(LEFT_ARM).state]);
        [ROR, pOR_O] = fwdkin(right_arm_kin,[q(WHEELCHAIR).state ...
                                            q(RIGHT_ARM).state]);
        pOT_O = pOL_O + ROL*bow.robots(LEFT_ARM).load.tb;
        % Grasp matrix
        GL = screw_matrix(ROB'*(pOT_O - pOL_O));
        GR = screw_matrix(ROB'*(pOT_O - pOR_O));
        G = [SH*GL*SH' SH*GR*SH'];

        % Jacobian for base + left arm with respect to origin
        JOL_B = blkdiag(ROB',ROB')*robotjacobian(left_arm_kin, ...
                        [q(WHEELCHAIR).state q(LEFT_ARM).state]);
        % Jacobian for base + right arm with respect to origin
        JOR_B = blkdiag(ROB',ROB')*robotjacobian(right_arm_kin, ...
                        [q(WHEELCHAIR).state q(RIGHT_ARM).state]);
        % Jacobian for left arm with respect to base
        JBL_B = robotjacobian(bow.robots(LEFT_ARM).kin, q(LEFT_ARM).state);
        % Jacobian for right arm with respect to base
        JBR_B = robotjacobian(bow.robots(RIGHT_ARM).kin, q(RIGHT_ARM).state);
        
        % Select only user-controllable degrees of freedom
        % rotated into base frame for ego-centric control
        JH = [ SH*[JOL_B(:,1:2) JOL_B(:,3:9) zeros(6,7)]; ...
               SH*[JOR_B(:,1:2) zeros(6,7) JOR_B(:,3:9)] ];
        JB = unicyclejacobian(bow.robots(WHEELCHAIR).kin, ...
                                            q(WHEELCHAIR).state);
        dqdu(1:3,1:2) = JB;

        %%% Read input from operator
        vH = user_input.velocity_command;

        %%% equality constraints 
        % preserve original hand orientations as well as hand-to-hand
        % translation relative to the current hand orientation
        % form error vector (local feedback control) vE = -kE*hE(q)
        eqL = R2q(RBLd*(ROB'*ROL)');
        eqR = R2q(RBRd*(ROB'*ROR)');
        epLR = ROB'*((pOR_O - pOL_O) - ROL*pLR_Ld);
        vE = -kE*[eqL(2:4);eqR(2:4);epLR];
        % convert orientation jacobian into quaternion velocity instead of
        %   angular velocity
        JqL = quatjacobian(eqL);
        JqR = quatjacobian(eqR);
        JE = [-JqL(2:4,:)*[[0;0;0] [0;0;0] JBL_B(1:3,:) zeros(3,7)]; ...
              -JqR(2:4,:)*[[0;0;0] [0;0;0] zeros(3,7) JBR_B(1:3,:)]; ...
                        [[0;0;0] [0;0;0] -JBL_B(4:6,:) JBR_B(4:6,:)]];

        %%% Solve for command as quadratic program

        % If planar command is sufficiently large, impose constraint 
        % that the motion generated by the base should follow the motion
        % command, i.e. the wheelchair should try to point in the direction of
        % the commanded velocity
        if norm(vH(1:2)) >= 1e-3
            JEB = [[0;0;1]'*hat(ROB*vH(1:3))*JB(:,1) ...
                    [0;0;1]'*hat(ROB*vH(1:3))*hat((pOT_O-pOB_O))*[0;0;1] ... 
                     zeros(1,14)];
            E = [JE;JEB]; 
            beta = [vE;0];
        else % add planar motion constraint
            E = JE; 
            beta = vE;
        end
        
        %%% inequality constraints
        % joint limit boundary constraints
        if norm(pOT_O(1:2)-port.t(1:2)) >= 1
            lower_joint_limit(4) = pi/2;
        else
            lower_joint_limit(4) = 0;
        end
        h(1:14) = [q(LEFT_ARM).state(:) - lower_joint_limit(:); ...
                   q(RIGHT_ARM).state(:) - lower_joint_limit(:)];
        h(15:28) = -[q(LEFT_ARM).state(:) - upper_joint_limit(:); ...
                     q(RIGHT_ARM).state(:) - upper_joint_limit(:)];
        % simple self-collision avoidance algorithm
        h(29) = norm(pOL_O - pOB_O) - 0.3;
        dhdq(29,:) = [0 0 0 ROB(1,:)*JBL_B(4:6,:) zeros(1,7)];
        h(30) = norm(pOR_O - pOB_O) - 0.3;
        dhdq(30,:) = [0 0 0 zeros(1,7) ROB(1,:)*JBR_B(4:6,:)];
        sigma(1:28) = inequality_bound(h(1:28),c_b,eta_b,epsilon_b,e_b);
        sigma(29:30) = inequality_bound(h(29:30),c_sc,eta_sc,epsilon_sc,e_sc);
        % Lu's algorithm has object detection and collision avoidance here
        % my adjustment would be to, before starting, get convhull of 
        % static obstacles and detect distances to that set of 
        % vertices / line segments
        
        
        %%% 
        % Pseudo-inverse map from constrained task space to joint space
        Epinv = pinv(E); 
        % Null space of JE for optimization variables
        Enull = null(E); 
        % nullspace of human-direction motion (JH), constrained 
        % motion (JE) and base motion (C)
        JHEC_n = null([JH;E;C]);
        % Quadratic cost matrices (min x'*H*x + f'*x s.t. Ax <= b)
        H1 = (JH'*JH) + lambda1*(JHEC_n*JHEC_n') + lambda2*(C'*C);
        H = Enull'*H1*Enull;
        if norm(H - H') > 0, H = (H + H')/2;    end
        f = Enull'*(H1*Epinv*beta - JH'*G'*vH);
        % Inequality constraints
        A = -dhdq*dqdu*Enull;
        b = -(sigma - dhdq*dqdu*Epinv*beta);
        xi = quadprog(H,f,A,b,[],[],[],[],[],quadprog_options);
        u = Epinv*beta + Enull*xi;

        % Update display
        q(WHEELCHAIR).state(:) = q(WHEELCHAIR).state(:) + JB*u(1:2)*dt;
        q(LEFT_ARM).state(:) = q(LEFT_ARM).state(:) + u(3:9)*dt;
        q(RIGHT_ARM).state(:) = q(RIGHT_ARM).state(:) + u(10:16)*dt;
        bow = updateRobot(q,bow);
        drawnow;
    end
    
    close(fig)
    
    function wkp(~, evd)
        if strcmpi(evd.Modifier,'control') % camera control
            [az, el] = view();
            if strcmpi(evd.Key,'leftarrow')
                view([az - 10, el]);
            elseif strcmpi(evd.Key,'rightarrow')
                view([az + 10, el]);
            elseif strcmpi(evd.Key,'uparrow')
                view([az, el+10]);
            elseif strcmpi(evd.Key,'downarrow')
                view([az, el-10]);
            end
        else % motion control
            if strcmpi(evd.Key,'escape')
                user_input.exit = true(1);
            elseif strcmpi(evd.Key,'leftarrow')
                user_input.velocity_command(2) = 0.5;
            elseif strcmpi(evd.Key,'rightarrow')
                user_input.velocity_command(2) = -0.5;
            elseif strcmpi(evd.Key,'uparrow')
                if any(strcmpi(evd.Modifier,'shift'))
                    user_input.velocity_command(3) = 0.5;
                else
                    user_input.velocity_command(1) = 0.5;
                end
            elseif strcmpi(evd.Key,'downarrow')
                if any(strcmpi(evd.Modifier,'shift'))
                    user_input.velocity_command(3) = -0.5;
                else
                    user_input.velocity_command(1) = -0.5;
                end
            elseif strcmpi(evd.Key,'g')
                user_input.grasp_command = true(1);
            elseif strcmpi(evd.Key,'r')
                user_input.release_command = true(1);
            end
        end
    end
    
    function wkr(~, evd)
        if strcmpi(evd.Key,'leftarrow') || strcmpi(evd.Key,'rightarrow')
            user_input.velocity_command(2) = 0;
        elseif strcmpi(evd.Key,'uparrow') || strcmpi(evd.Key,'downarrow')
            user_input.velocity_command([1 3]) = 0;
            elseif strcmpi(evd.Key,'g')
                user_input.grasp_command = false(1);
            elseif strcmpi(evd.Key,'r')
                user_input.release_command = false(1);
        end
    end
end