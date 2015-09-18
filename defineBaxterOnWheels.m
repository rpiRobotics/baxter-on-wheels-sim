function [bow_const, bow_structure] = defineBaxterOnWheels(varargin)
    %
    % [bow_const, bow_structure] = defineBaxterOnWheels()
    % [bow_const, bow_structure] = defineBaxterOnWheels(...) 
    %                   allows additional optional parameters
    %       'Origin'        :   default [eye(3) [0;0;0]; [0 0 0] 1]
    %       'Color'         :   default [0.8;0;0.8]
    %       'BaxterNumber'  :   default: '2'
    %       'LeftGripper' / :   structure with creation options accepted by
    %       'RightGripper'      baxterGripper as fields
    %                               -> param 
    %                               -> props
    %                           default: no gripper attached
    %
    % define-file for the Baxter on Wheels and its wheelchair base.  
    %   Returns struct with the following form:
    %
    % root
    %   -> name         : string denoting the name of the robot
    %   -> kin          : container for kinematics
    %       -> H                 : [3 x 3] joint axes
    %       -> P                 : [3 x 4] inter-joint displacements
    %       -> joint_type        : [1 x 3] joint types
    %   -> limit        : container for limits
    %   -> vis          : container for visualization
    %       -> joints            :  3 index struct array containing fields
    %                               param
    %                               props
    %       -> links             :  4 index cell array containing fields
    %                               handle
    %                               R
    %                               t
    %                               param
    %                               props
    %       -> frame             : struct containing appropriate sized
    %                               dimensions for visualizing the 
    %                               3D frames
    %       -> peripherals       : struct containing extra visual
    %                               attachments to the robot
    %
    % see also CREATEROBOT CREATECOMBINEDROBOT DEFINEBAXTER
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    flags = {'Origin', 'Color', 'BaxterNumber', ...
                'LeftGripper', 'RightGripper'};
    defaults = {eye(4), [0.8;0;0.8], '2', [], []};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    origin = opt_values{1};
    c = opt_values{2};
    bn = opt_values{3};
    left_gripper_options = opt_values{4};
    right_gripper_options = opt_values{5};
    
    R0 = origin(1:3,1:3);
    t0 = origin(1:3,4);
    
    switch(bn)
        case '1'
            p3T = [0;0;0]; % TBD
        case '2'
            p3T = [0.055;0;0.3];
        case '3'
            p3T = [0;0;0]; % TBD
        otherwise
            error('Unrecognized Baxter Number');
    end
    
    % Get Baxter constants
    [baxter_const, baxter_structure] = defineBaxter('Pedestal','off', ...
                                    'LeftGripper',left_gripper_options, ...
                                    'RightGripper',right_gripper_options);
    
    % Grab standard robot structure, defining 4 robots for the 
    %  wheelchair + 3 robot chains in the baxter
    bow_const = defineEmptyRobot(numel(baxter_const) + 1);
    
    bow_const(1:numel(baxter_const)) = baxter_const;
    
    %%% Define wheelchair
    % Name
    bow_const(end).name = 'wheelchair';
    
    % Kinematic constants
    bow_const(end).kin.H = R0*[x0 y0 z0];
    bow_const(end).kin.P = R0*[.145*z0 zed zed p3T];
    bow_const(end).kin.P(:,1) = t0 + bow_const(end).kin.P(:,1);
    bow_const(end).kin.joint_type = [3 3 2];
    
    % Visualization constants
    % no joint visualizations
    bow_const(end).vis.joints = struct('param',cell(1,3),'props',cell(1,3));
    
    % only 'link' is at tool frame
    bow_const(end).vis.links = struct('handle',cell(1,4), ...
                            'R',cell(1,4),'t',cell(1,4), ...
                            'param',cell(1,4),'props',cell(1,4));
    
    bow_const(end).vis.links(4).handle = @createCuboid;
    bow_const(end).vis.links(4).R = eye(3);
    bow_const(end).vis.links(4).t = [0;0;0.125];
    bow_const(end).vis.links(4).param = struct('width',.45, ...
                                                'length',.5, ...
                                                'height',.35);
    bow_const(end).vis.links(4).props = {'FaceColor', c, 'EdgeAlpha', 1};
    
    % appropriate scale for coordinate frame
    bow_const(end).vis.frame = struct('scale', 0.3, 'width', 0.015);
    
    % Wheels and supports
    bow_const(end).vis.peripherals(1).id = 'left_wheel';
    bow_const(end).vis.peripherals(1).frame = 3;
    bow_const(end).vis.peripherals(1).handle = @createCylinder;
    bow_const(end).vis.peripherals(1).R = R0*rot(x0,pi/2);
    bow_const(end).vis.peripherals(1).t = R0*[0;.275;0];
    bow_const(end).vis.peripherals(1).param = struct('radius',0.145, ...
                                                    'height',0.05);
    bow_const(end).vis.peripherals(1).props = {'FaceColor',[0.2;0.2;0.2]};
    
    bow_const(end).vis.peripherals(2).id = 'right_wheel';
    bow_const(end).vis.peripherals(2).frame = 3;
    bow_const(end).vis.peripherals(2).handle = @createCylinder;
    bow_const(end).vis.peripherals(2).R = R0*rot(x0,pi/2);
    bow_const(end).vis.peripherals(2).t = R0*[0;-.275;0];
    bow_const(end).vis.peripherals(2).param = struct('radius',0.145, ...
                                                    'height',0.05);
    bow_const(end).vis.peripherals(2).props = {'FaceColor',[0.2;0.2;0.2]};
    
    
    bow_const(end).vis.peripherals(3).id = 'front_left_wheel';
    bow_const(end).vis.peripherals(3).frame = 3;
    bow_const(end).vis.peripherals(3).handle = @createCylinder;
    bow_const(end).vis.peripherals(3).R = R0*rot(x0,pi/2);
    bow_const(end).vis.peripherals(3).t = R0*[.25;.275;-0.075];
    bow_const(end).vis.peripherals(3).param = struct('radius',0.07, ...
                                                    'height',0.04);
    bow_const(end).vis.peripherals(3).props = {'FaceColor',[0.4;0.4;0.4]};
    
    bow_const(end).vis.peripherals(4).id = 'front_right_wheel';
    bow_const(end).vis.peripherals(4).frame = 3;
    bow_const(end).vis.peripherals(4).handle = @createCylinder;
    bow_const(end).vis.peripherals(4).R = R0*rot(x0,pi/2);
    bow_const(end).vis.peripherals(4).t = R0*[.25;-.275;-0.075];
    bow_const(end).vis.peripherals(4).param = struct('radius',0.07, ...
                                                    'height',0.04);
    bow_const(end).vis.peripherals(4).props = {'FaceColor',[0.4;0.4;0.4]};
    
    bow_const(end).vis.peripherals(5).id = 'back_left_wheel';
    bow_const(end).vis.peripherals(5).frame = 3;
    bow_const(end).vis.peripherals(5).handle = @createCylinder;
    bow_const(end).vis.peripherals(5).R = R0*rot(x0,pi/2);
    bow_const(end).vis.peripherals(5).t = R0*[-.32;.1;-0.075];
    bow_const(end).vis.peripherals(5).param = struct('radius',0.07, ...
                                                    'height',0.04);
    bow_const(end).vis.peripherals(5).props = {'FaceColor',[0.4;0.4;0.4]};
    
    bow_const(end).vis.peripherals(6).id = 'back_right_wheel';
    bow_const(end).vis.peripherals(6).frame = 3;
    bow_const(end).vis.peripherals(6).handle = @createCylinder;
    bow_const(end).vis.peripherals(6).R = R0*rot(x0,pi/2);
    bow_const(end).vis.peripherals(6).t = R0*[-.32;-.1;-0.075];
    bow_const(end).vis.peripherals(6).param = struct('radius',0.07, ...
                                                    'height',0.04);
    bow_const(end).vis.peripherals(6).props = {'FaceColor',[0.4;0.4;0.4]};
    
    bow_const(end).vis.peripherals(7).id = 'back_left_wheel_support';
    bow_const(end).vis.peripherals(7).frame = 3;
    bow_const(end).vis.peripherals(7).handle = @createCylinder;
    bow_const(end).vis.peripherals(7).R = R0*rot(y0,pi/2);
    bow_const(end).vis.peripherals(7).t = R0*[-.2725;.1;0.035];
    bow_const(end).vis.peripherals(7).param = struct('radius',0.04, ...
                                                    'height',0.095);
    bow_const(end).vis.peripherals(7).props = {'FaceColor',c};
    
    bow_const(end).vis.peripherals(8).id = 'back_right_wheel_support';
    bow_const(end).vis.peripherals(8).frame = 3;
    bow_const(end).vis.peripherals(8).handle = @createCylinder;
    bow_const(end).vis.peripherals(8).R = R0*rot(y0,pi/2);
    bow_const(end).vis.peripherals(8).t = R0*[-.2725;-.1;0.035];
    bow_const(end).vis.peripherals(8).param = struct('radius',0.04, ...
                                                    'height',0.095);
    bow_const(end).vis.peripherals(8).props = {'FaceColor',c};
    
    
    %%% Define structure of combined baxter-on-wheels
    
    bow_structure = defineEmptyRobotStructure(numel(baxter_const) + 1);
    bow_structure(1:numel(baxter_structure)) = baxter_structure;
    bow_structure(end).name = bow_const(end).name;
    [bow_structure(1:3).left] = deal(bow_const(end).name);
    bow_structure(end).right = {baxter_const(1:3).name};
end