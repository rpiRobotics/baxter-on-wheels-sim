function BOWKeyboardControl()


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
    shelves = createShelves(rot([0;0;1],-pi/2),[2.5;0;0], ...
                                            struct('shelf_param', ...
                                                struct('width', 1, ...
                                                    'length', 0.5, ...
                                                    'height', 0.3), ...
                                            'board_width', 0.05, ...
                                            'n_shelves', 4));
    obj_radius = 0.045;
    obj_height = 0.2;
    obj = createCylinder(eye(3),[0;0;0],struct('radius',obj_radius, ...
                                            'height',obj_height), ...
                                            'FaceColor', [0;0;1]);
    table = createTable(eye(3), [1;-1.5;0], struct('surface_param', ...
                                                struct('width', 1, ...
                                                       'length', 0.5, ...
                                                       'height', 0.02), ...
                                                    'leg_param', ...
                                                struct('width', 0.05, ...
                                                       'length', 0.05, ...
                                                       'height', 0.7)));
    target_radius = 3*obj_radius;
    target_location = createCylinder(eye(3),[0;0;0], ...
                                        struct('radius', target_radius, ...
                                               'height', 0.02), ...
                                        'FaceColor', [0;0.9;0], ...
                                        'EdgeAlpha', 0);
    target_location = updateRigidBody(eye(3),[1;-1.5;0.73],target_location);
    
    obj = updateRigidBody(eye(3),[2.4;0;0.85],obj);
    axis equal; axis([-1 3 -2 2 0 2]); grid on;
    
    % Set figure to listen for key press events
    set(fig,'WindowKeyPressFcn',{@wkp});
    set(fig,'WindowKeyReleaseFcn',{@wkr});

    % Get angle structure for use in controller
    q = get_angle_structure(bow);
    % indices for future updates
    LEFT_ARM = strcmpi({q.name},'baxter_left_arm');
    RIGHT_ARM = strcmpi({q.name},'baxter_right_arm');
    LEFT_GRIPPER = ~cellfun('isempty',strfind({q.name},'left_gripper'));
    WHEELCHAIR = strcmpi({q.name},'wheelchair');
    % full body kinematic structures
    left_arm_kin = get_kinematic_chain(bow, 'wheelchair', ...
                                            'baxter_left_arm');
    left_gripper = bow.robots(LEFT_GRIPPER);
    pLT_L = 1/2*(left_gripper(1).frames(end).t + ...
                    left_gripper(2).frames(end).t);
    
    % joint boundaries as suggested by Lu's TASE paper
    lower_joint_limit = [-pi/4 -pi/3 -2*pi/3 pi/2 -pi/2 -pi/2 -pi/2];
    upper_joint_limit = [pi/2 pi/3 -pi/4 2*pi/3 pi/2 pi/2 pi/2];

    % set initial pose
    q(LEFT_ARM).state = [pi/4 0 -pi/2 pi/2 0 pi/4 pi/2];
    q(RIGHT_ARM).state = [-pi/4 pi/4 pi/2 pi/2 0 pi/2 0];

    bow = updateRobot(q,bow);
    drawnow;
    
    dt = 0.05;

    % constants for inequality bound sigma(h_I)
    c_b = 0.9; eta_b = 0.15; epsilon_b = 0.15; e_b = 0.01;
    c_sc = 0.9; eta_sc = 0.1; epsilon_sc = 0.15; e_sc = 0.01;
    % relative weights
    lambda1 = 1;
    lambda2 = 0.1;
    % feedback gain for equality constraint
    kE = 0.9;
    % Selection matrices
    C = [eye(2) zeros(2,7)]; % Selection matrix for wheelchair command
    SE = [eye(3) zeros(3)]; % Selection matrix for constrained dof
    SH = [zeros(3) eye(3)]; % Selection matrix for human-controlled dof

    % Want initial orientation between end effector and base to be maintained
    RBLd = bow.robots(LEFT_ARM).frames(end).R;


    h = zeros(15,1);        % 7 joints have upper and lower limits
                            % Lu's code has additional obstacle avoidance. 
                            %(# objects additional inequality constraints)
    sigma = zeros(size(h)); % boundary values
    % Partial derivative of inequality constraints with respect to q.  Joint
    % limits are simply a positive and negative identity
    dhdq = [zeros(7,3) eye(7); ...
            zeros(7,3) -eye(7);
            zeros(1,10)];
    dqdu = zeros(10,9); % Jacobian relating input to state
    dqdu(4:10,3:9) = eye(7);

    % Options set for MATLAB quadratic programming solver
    quadprog_options = optimoptions('quadprog','Display','none');
    
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
        pOT_O = pOL_O + ROL*pLT_L;

        % Jacobian of Forward Kinematics for base + arm with respect to origin
        JOL_O = robotjacobian(left_arm_kin,[q(WHEELCHAIR).state ...
                                            q(LEFT_ARM).state]);
        % Jacobian of Forward Kinematics for arm with respect to base
        JBL_B = robotjacobian(bow.robots(LEFT_ARM).kin, q(LEFT_ARM).state);
        % Select only user-controllable degrees of freedom
        JH = ROB'*SH*JOL_O; % rotated into base frame for ego-centric control
        JB = unicyclejacobian(bow.robots(WHEELCHAIR).kin, ...
                                            q(WHEELCHAIR).state);
        dqdu(1:3,1:2) = JB;

        %%% Read input from operator
        vH = user_input.velocity_command;
        if user_input.grasp_command && isempty(bow.robots(LEFT_ARM).load)
            % grasp test
            disp('attempting grasp');
            
            if norm(pOT_O(1:2) - obj.t(1:2)) < obj_radius && ...
                    norm(pOL_O(3) - obj.t(3)) < obj_height
                bow = graspLoad(obj, bow, bow.robots(LEFT_ARM).name);
            else
                disp('grasp failed');
            end
        end
        if user_input.release_command && ~isempty(bow.robots(LEFT_ARM).load)
            % release
            disp('attempting release');
            disp(num2str(pOT_O - target_location.t - [0;0;obj_height/2]))
            if norm(pOT_O(1:2) - target_location.t(1:2)) < target_radius && ...
                abs(pOT_O(3) - obj_height/2 - target_location.t(3)) < 0.04 
                [bow, obj] = releaseLoad(bow, bow.robots(LEFT_ARM).name);
            else
                disp('release failed');
            end
        end

        %%% equality constraints 
        % 
        % form error vector (local feedback control) vE = -kE*hE(q)
        eq = R2q(RBLd*(ROB'*ROL)');
        vE = -kE*eq(2:4);
        % convert orientation jacobian into quaternion velocity instead of
        %   angular velocity
        Jq = quatjacobian(eq);
        JE = -Jq(2:4,:)*[[0;0;0] [0;0;0] SE*JBL_B];

        %%% Solve for command as quadratic program

        % If planar command is sufficiently large, impose constraint 
        % that the motion generated by the base should follow the motion
        % command, i.e. the wheelchair should try to point in the direction of
        % the commanded velocity
        if norm(vH(1:2)) >= 1e-3
            JEB = [[0;0;1]'*hat(ROB*vH)*JB(:,1) ...
                    [0;0;1]'*hat(ROB*vH)*hat((pOL_O-pOB_O))*[0;0;1] ... 
                     zeros(1,7)];
            E = [JE;JEB]; 
            beta = [vE;0];
        else % add planar motion constraint
            E = JE; 
            beta = vE;
        end
        
        %%% inequality constraints
        % joint limit boundary constraints
        if norm(pOT_O(1:2)-obj.t(1:2)) >= 1
            lower_joint_limit(4) = pi/2;
        else
            lower_joint_limit(4) = 0;
        end
        h(1:7) = q(LEFT_ARM).state(:) - lower_joint_limit(:);
        h(8:14) = -(q(LEFT_ARM).state(:) - upper_joint_limit(:));
        % simple self-collision avoidance algorithm
        h(15) = norm(pOL_O - pOB_O) - 0.3;
        dhdq(15,:) = [0 0 0 ROB(1,:)*JBL_B(4:6,:)];
        sigma(1:14) = inequality_bound(h(1:14),c_b,eta_b,epsilon_b,e_b);
        sigma(15) = inequality_bound(h(15),c_sc,eta_sc,epsilon_sc,e_sc);
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
        f = Enull'*(H1*Epinv*beta - JH'*vH);
        % Inequality constraints
        A = -dhdq*dqdu*Enull;
        b = -(sigma - dhdq*dqdu*Epinv*beta);
        xi = quadprog(H,f,A,b,[],[],[],[],[],quadprog_options);
        u = Epinv*beta + Enull*xi;

        % Update display
        q(WHEELCHAIR).state(:) = q(WHEELCHAIR).state(:) + JB*u(1:2)*dt;
        q(LEFT_ARM).state(:) = q(LEFT_ARM).state(:) + u(3:9)*dt;
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