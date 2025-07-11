% 
% Junnan Li
% 
% Comments:
%   Jacobian function:  
%       the column number does not match to the coordinateSet




classdef osim_model < handle

    % Initialize simulation  
%     properties (Constant)      
%         stepsize = 0.01; 
%     end
    
    
    properties
        model_path
        
%         integrator_accuracy = 5e5;
        model
        init_state % init state
        state % state for update
        viz
        MuscleSet
        MuscleSet_list
        ForceSet
        ForceSet_list
        BodySet
        BodySet_list
        CoordinateSet
        CoordinateSet_list          % colume 1: Coordinate names
                                    % colume 2: constrained parent coordinates  
                                    % colume 3: 
        Coordinate_body                                         
        Coord_minimal               % index of minimal set of coordinates of body
        Coord_minimal_range         % range of minimal coordinates
        CoordinateInOrder           % getCoordinateNamesInMultibodyTreeOrder the order in smss system
        ConstraintSet
        ConstraintSet_list  
        Constraint_on               % [1] consider constraints while set_coordinate, IK  
        FrameList
        JointSet
        JointSet_list
        MarkerSet
        MarkerSet_list
        ContactGeometrySet
        ContactGeometrySet_list
        smss % MatterSubsystem
        marker_point_list
%         noutput
        scale_tool
%         last_action
%         state
        manager
%         istep
%         state_desc
%         maxforces = [];
%         curforces = [];
%         state_desc_istep = [];
%         prev_state_desc
%         vtgt
    end

    methods
        function om = osim_model(model_path)
            import org.opensim.modeling.*;
            if nargin == 0
                % default model in the folder
                om.model_path = strcat(Path_Model, '\WristModel_with_allMuscles.osim');            
            else
                % Provide values for superclass constructor
                % and initialize other inputs
                om.model_path = model_path;
            end

            om.model = org.opensim.modeling.Model(om.model_path);
            
            om.viz = [];
            om.model.setUseVisualizer(false);
            om.model.initSystem();
            om.update_system;
            om.update_set(); % update all set and lists of the model
            om.marker_point_list = {};
            om.Constraint_on = 1;

%             om.noutput = om.muscleSet.getSize();    
        end
    


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% update system
        function update_set(om)
            import org.opensim.modeling.*;
            om.MuscleSet = om.model.getMuscles();
            om.MuscleSet_list = Setlist_read(om.MuscleSet);
            om.ForceSet = om.model.getForceSet();
            om.ForceSet_list = Setlist_read(om.ForceSet);
            om.BodySet = om.model.getBodySet();
            om.BodySet_list = Setlist_read(om.BodySet);
            om.JointSet = om.model.getJointSet();
            om.JointSet_list = Setlist_read(om.JointSet);
            om.CoordinateSet = om.model.getCoordinateSet();
            om.CoordinateSet_list = Setlist_read(om.CoordinateSet);
            om.FrameList
            om.MarkerSet = om.model.getMarkerSet();
            om.MarkerSet_list = Setlist_read(om.MarkerSet);
            om.ContactGeometrySet = om.model.getContactGeometrySet();
            om.ContactGeometrySet_list = Setlist_read(om.ContactGeometrySet);
            om.ConstraintSet = om.model.getConstraintSet();
            om.ConstraintSet_list = Setlist_read(om.ConstraintSet);
            
            CNIMTO = om.model.getCoordinateNamesInMultibodyTreeOrder;
            om.CoordinateInOrder = [];
            for i = 1:CNIMTO.capacity
                om.CoordinateInOrder = [om.CoordinateInOrder;{char(CNIMTO.getElt(i-1))}];
            end
            % update constraints information
            om.update_ConstraintsInfo_in_CoordinateList;
            
            % update the minimal set of coordinates
            om.Coord_minimal = {};
%             om.coord_min_index
            for i = 1:size(om.CoordinateSet_list,1)
                if isempty(om.CoordinateSet_list{i,2})
                    om.Coord_minimal = [om.Coord_minimal; ...
                        [om.CoordinateSet_list{i,1},num2cell(i)]];
                end
            end
            om.Coord_minimal_range = zeros(size(om.Coord_minimal,1),2);
            for i = 1:size(om.Coord_minimal,1)
                coor_i = om.CoordinateSet.get(om.Coord_minimal{i,1});
                om.Coord_minimal_range(i,:)  = [coor_i.getRangeMin,coor_i.getRangeMax];
            end
            
        end

        function update_ConstraintsInfo_in_CoordinateList(om)
            % in the coordinateSet_list
            % column 1: constraint names
            %        2: the parent coordinate if exists
            %        3: cubicspline points coor [x,y] with nx2 dimension
            %        4: the matlab cubic function according to the points
            %       
            % if only two points exist, linear
            import org.opensim.modeling.*;
            if ~isempty(om.ConstraintSet_list)
                for i = 1:length(om.ConstraintSet_list)
                    con_i = om.ConstraintSet.get(i-1);
                    coor_name = con_i.getPropertyByIndex(3).toString; % parent coord. name
                    coord_parent = char(coor_name.substring(1,coor_name.length-1));
                    coord_child = char(con_i.getPropertyByIndex(4)); % child coord. name
                    coor_child_index = find(ismember(om.CoordinateSet_list(:,1),coord_child));
                    om.CoordinateSet_list(coor_child_index,2) = {coord_parent};

                    func = con_i.getPropertyByIndex(2).getValueAsObject(0);
                    text = {};
                    text(1) = func.getPropertyByIndex(0).toString; % SimmSpline viapoints X values
                    text(2) = func.getPropertyByIndex(1).toString; % SimmSpline viapoints Y values
                    % convert String to num
                    x_split = split(erase(text{1},{'(',')'}),' ');
                    y_split = split(erase(text{2},{'(',')'}),' ');
                    % update the third column of the CoordinateSet_list
                    % with constrainted point 
                    om.CoordinateSet_list{coor_child_index,3} = [str2double(x_split),str2double(y_split)];
                    % generate cubic function to the fourth column
                    x_i = [str2double(x_split),str2double(y_split)];
                    f_i = spline(x_i(:,1),x_i(:,2));
                    om.CoordinateSet_list{coor_child_index,4} = f_i;
                end
            end
        end

        function update_system(om)
            import org.opensim.modeling.*;
            om.init_state = om.model.initSystem();
            om.state = om.init_state;
            om.smss = om.model.getMatterSubsystem();
        end

        function status = save_model(om,path)
            import org.opensim.modeling.*;
            om.model.print(path);
            fprintf('OpenSim model saved \n');
            status = 1;
        end

        function stateNames = get_systemStateNames(om)
            % system state includes Y = [Q,U,Z];
            % Q: joint angle
            % U: coordinate velocity
            % Z: auxiliary variables (muscle activation and fiber length)
            import org.opensim.modeling.*;
            stateNames = Setlist_read(om.model.getStateVariableNames);
        end

        function stateValues = get_systemStateValues(om)
            import org.opensim.modeling.*;
            stateValues = osimMatrix2matrix(om.model.getStateVariableValues(om.state));
        end

        function states = get_systemState(om)
            % system state includes Y = [Q,U,Z];
            % Q: joint angle
            % U: coordinate velocity
            % Z: auxiliary variables (muscle activation and fiber length)
            import org.opensim.modeling.*;
            states = {};
            states(:,1) = om.get_systemStateNames();
            states(:,2) = num2cell(om.get_systemStateValues());
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% visualization

        function set_visualize(om)
            import org.opensim.modeling.*;
            om.model.setUseVisualizer(true);
            om.init_state = om.model.initSystem();
            om.state = om.init_state;
            om.update_set(); % update all set and lists of the model
            om.smss = om.model.getMatterSubsystem();
            om.viz = om.model.updVisualizer().updSimbodyVisualizer();
            om.viz.setShowSimTime(true);
            om.viz.setBackgroundColor(Vec3(0)); % white
            om.viz.setGroundHeight(-5)
        end

        function model_visualize(om)
            import org.opensim.modeling.*;
%             om.model.setUseVisualizer(true);
            om.model.getVisualizer().show(om.state);
        end

        function plot_all_body(om)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;
            num_body = om.BodySet.getSize;
            
            for i = 1:num_body % eul2rotm([0,0,pi/2])*
            w_p_bodyi_pos = osimMatrix2matrix(om.BodySet.get(i-1).getTransformInGround(om.state).p);
            plot3(w_p_bodyi_pos(1),w_p_bodyi_pos(2),w_p_bodyi_pos(3),'.-',MarkerSize=15,Color='r')
            hold on
            axis equal
            grid on
            end
        end

        function plot_all_body_frame(om)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;
            num_body = om.BodySet.getSize;
            for i = 1:num_body % eul2rotm([0,0,pi/2])*
                w_p_bodyi_pos = osimMatrix2matrix(om.BodySet.get(i-1).getTransformInGround(om.state).p);
                w_R_bodyi_pos = osimMatrix2matrix(om.BodySet.get(i-1).getTransformInGround(om.state).R);
                plot3(w_p_bodyi_pos(1),w_p_bodyi_pos(2),w_p_bodyi_pos(3),'.-',MarkerSize=15,Color='r')
                hold on
                axis equal
                grid on
                om.plot_frame(w_p_bodyi_pos,w_R_bodyi_pos,0.01);
            end
        end
        
        function plot_body(om, body_list)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;
            num_body = length(body_list);
            
            for i = 1:num_body % eul2rotm([0,0,pi/2])*

                w_p_bodyi_pos = osimMatrix2matrix(om.BodySet.get(body_list{i}).getTransformInGround(om.state).p);
                plot3(w_p_bodyi_pos(1),w_p_bodyi_pos(2),w_p_bodyi_pos(3),'.-',MarkerSize=15,Color='r')
                hold on
                axis equal
                grid on
            end
        end

        function plot_all_Coordinate(om, axis_length)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;
            
            w_aor_all = om.get_coordinate_axis_all();
            w_x_all = om.get_coordinate_pos_all();
            for i = 1:size(w_x_all,2)
                om.plot_axis( w_x_all(:,i), w_aor_all(:,i), axis_length)
            end

        end

        function plot_world_frame(om)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;
            ground = om.model.getGround;
            w_p = osimMatrix2matrix(ground.getTransformInGround(om.state).p);
            w_R = osimMatrix2matrix(ground.getTransformInGround(om.state).R);
            om.plot_frame(w_p,w_R,0.1);
            axis equal
            grid on
        end

        function plot_mp_frame(om)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;
            num_marker_point = length(om.marker_point_list);
            for i = 1:num_marker_point
                [~, w_p, w_R] = om.get_mp_frame(i);
                om.plot_frame(w_p,w_R,0.1);
            end
            axis equal
            grid on
        end
        
        

        function plot_muscle_path(om,mus_list)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;

            num_muscle = length(mus_list);
            for i = 1:num_muscle
                PathPoints = om.get_musclePathPoint(mus_list(i));
                num_pp = size(PathPoints,2)-1;
                for j = 1:num_pp
                    body_name = PathPoints{1,j+1};
                    w_p_b = osimMatrix2matrix(om.BodySet.get(body_name).getTransformInGround(om.state).p);
                    w_R_b = osimMatrix2matrix(om.BodySet.get(body_name).getTransformInGround(om.state).R);
                    p = PathPoints{2,j+1}';
                    w_p = w_R_b * p + w_p_b;
                    plot3(w_p(1),w_p(2),w_p(3),'.-',MarkerSize=8,Color='b')
                    hold on
                    axis equal
                    grid on
                end
            end
            axis equal
            grid on
        end

        function plot_frame(om, w_p, w_R, axis_len)
            % plot the x y z axis with given transformation matrix
            % x: red
            % y: blue
            % z: black
            assert(length(w_p)==3,'plot_frame: incorrect input dimension!')
            assert(isequal(size(w_R),[3 3]),'plot_frame: incorrect input dimension!')
            color_axis = {'r','b','k'};
            linewidth = 2;
            for i = 1:3
                axis_pos_i = zeros(3,1);
                axis_pos_i(i) = axis_len;
                w_p_axis = w_R*axis_pos_i;% + w_p;
                h = quiver3(w_p(1),w_p(2),w_p(3),...
                    w_p_axis(1),w_p_axis(2),w_p_axis(3),...
                    'Color',color_axis{i},'LineWidth',linewidth);
                set(h,'AutoScale','on', 'AutoScaleFactor',1)
                hold on
                drawnow
            end
            xlabel('x')
            ylabel('y')
            zlabel('z')
        end
        function plot_axis(om, w_p, axis, axis_len)
            % plot the x y z axis with given transformation matrix
            % x: red
            % y: blue
            % z: black
            assert(length(w_p)==3,'plot_axis: incorrect input dimension!')
%             assert(isequal(size(w_R),[3 3]),'plot_frame: incorrect input dimension!')
            color_axis = {'r'};
            linewidth = 2;


            axis_pos_i = axis_len*axis;
            w_p_axis = axis_pos_i;% + w_p;
            h = quiver3(w_p(1),w_p(2),w_p(3),...
                w_p_axis(1),w_p_axis(2),w_p_axis(3),...
                'Color',color_axis{1},'LineWidth',linewidth);
            set(h,'AutoScale','on', 'AutoScaleFactor',1)
            hold on
            drawnow

            xlabel('x')
            ylabel('y')
            zlabel('z')
        end
        
        %% Body information

        function w_T_body = get_body_T(om,Body_name)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;
            w_p = osimMatrix2matrix(om.BodySet.get(Body_name).getTransformInGround(om.state).p);
            w_R = osimMatrix2matrix(om.BodySet.get(Body_name).getTransformInGround(om.state).R);
            w_T_body = [w_R,w_p;0,0,0,1];
        end


        %%

        
        function set_scalefile(om,setting)
            import org.opensim.modeling.*;
            om.scale_tool = ScaleTool(setting);

        end


        function show_scale_info(om)
            
            modelname = om.scale_tool.getGenericModelMaker.getModelFileName();
            output_modelname = om.scale_tool.getName();
            settingfile_name = om.scale_tool.getDocumentFileName();
            markersetfile_name = om.scale_tool.getGenericModelMaker.getMarkerSetFileName();
            markerfile_name = om.scale_tool.getModelScaler.getMarkerFileName();
            subject_mass = om.scale_tool.getSubjectMass();
            timerange = om.scale_tool.getModelScaler.getTimeRange();
            fprintf("\n-------ScaleTool-------\n  ")
            fprintf('Osim model name: %s \n  ',modelname);
            fprintf('scaled model name: %s \n  ',output_modelname);
            fprintf('Setting file name: %s \n  ',settingfile_name);
            fprintf('MarkerSet name: %s \n  ',markersetfile_name);
            fprintf('Marker file name: %s \n  ',markerfile_name);
            fprintf('Motion start time: %.4f \n  ',timerange.get(0));
            fprintf('Motion end time: %.4f \n  ',timerange.get(1));
            fprintf('Subject total mass: %.4f \n',subject_mass);

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Coordinate operations

        function set_coordinate_value(om, coordinate_name_list, coordinate_value)
            % set the value to the given coordinates
            assert(length(coordinate_value) == length(coordinate_name_list),'input dimension wrong')
            
            for i = 1: length(coordinate_name_list)
                value_i = coordinate_value(i);
                range_i = [om.CoordinateSet.get(coordinate_name_list{i}).getRangeMin,...
                    om.CoordinateSet.get(coordinate_name_list{i}).getRangeMax];
                if value_i < range_i(1) || value_i > range_i(2)
                    fprintf('the value of coordinate %s is out of range! \n', coordinate_name_list{i})
                end
                if om.Constraint_on
                    om.CoordinateSet.get(coordinate_name_list{i}).setValue(om.state,value_i);
                else
                    om.CoordinateSet.get(coordinate_name_list{i}).setValue(om.state,value_i,0);
                end
                
            end
            om.model.equilibrateMuscles(om.state);
        end
        
        function set_coordinate_value_minimal(om, coordinate_value)
            % set the minimal coordinate values
           assert(length(coordinate_value) == size(om.Coord_minimal_range,1),'input dimension wrong')
            
            for i = 1: size(om.Coord_minimal_range,1)
                value_i = coordinate_value(i);
                range_i = om.Coord_minimal_range(i,:);
                if value_i < range_i(1) || value_i > range_i(2)
                    fprintf('the value of coordinate %s is out of range! \n', om.Coord_minimal{i,1})
                end
                if om.Constraint_on
                    om.CoordinateSet.get(om.Coord_minimal{i,1}).setValue(om.state,value_i);
                else
                    om.CoordinateSet.get(om.Coord_minimal{i,1}).setValue(om.state,value_i,0);
                end 
            end
            om.model.equilibrateMuscles(om.state);
        end

        function coord_value = get_coordinate_value(om, coordinate_name_list)
           coord_value = zeros(length(coordinate_name_list),1);
            for i = 1: length(coordinate_name_list)
                coord_value(i) = om.CoordinateSet.get(coordinate_name_list{i}).getValue(om.state);
            end
        end

        function q_minimal = get_coordinate_value_minimal(om)
           q_all = osimMatrix2matrix(om.state.getQ);
           q_minimal = q_all([om.Coord_minimal{:,2}]);
        end

        function set_constrain_disable(om, constrains_diasble)
           % disable constrains
            for i = 1: length(constrains_diasble)
                con_i = om.ConstraintSet.get(constrains_diasble{i});
                con_i.setIsEnforced(om.state,0);
            end
        end
        

        function w_x_i = get_coordinate_pos(om, Coord_Name)
           % get coordinate position in world frame 
           % Omit local position of child frame in parent frame
           % Coord_Name: string
           import org.opensim.modeling.*;

           coord_i = om.CoordinateSet.get(Coord_Name);
           joint_i = coord_i.getJoint();
%            parentFrame = joint_i.getParentFrame();
           childFrame = joint_i.getChildFrame();
           childOrigin = Vec3(0);
           worldLocationVec3 = childFrame.findStationLocationInGround(om.state, childOrigin);
           w_x_i = [worldLocationVec3.get(0), worldLocationVec3.get(1), worldLocationVec3.get(2)]';
        end

        function w_x_all = get_coordinate_pos_all(om)
            % get coordinate position in world frame
            % Omit local position of child frame in parent frame
            % Coord_Name: string
            import org.opensim.modeling.*;
            w_x_all = [];
            for i = 1: om.CoordinateSet.getSize
                w_x_all(:,i) = om.get_coordinate_pos(om.CoordinateSet_list{i,1});
            end
        end

        function w_aor = get_coordinate_axis(om, Coord_Name)
            % get coordinate position in world frame
            % Coord_Name: string
            import org.opensim.modeling.*;

            coord_i = om.CoordinateSet.get(Coord_Name);
            joint_i = coord_i.getJoint();
            joint_Name = joint_i.getName();
            jointType = char(joint_i.getConcreteClassName());
            childFrame = joint_i.getChildFrame();
            switch jointType
                case 'CustomJoint'
                    % Cast to CustomJoint
                    customJoint = CustomJoint.safeDownCast(joint_i);
                    spatialTransform = customJoint.getSpatialTransform();
                    % Check each rotation to find which one contains this coordinate
                    rotations = {spatialTransform.get_rotation1(), ...
                        spatialTransform.get_rotation2(), ...
                        spatialTransform.get_rotation3()};
                    for i = 1:3
                        rotation = rotations{i};
                        coordName = rotation.getCoordinateNames();
                        coordName_char = char(coordName.toString);
                        if strcmp(Coord_Name,coordName_char(2:end-1)) == 1
                            axis = rotation.getAxis();
                            localAxis = Vec3(axis.get(0), axis.get(1), axis.get(2));

                            % Transform to world frame using the appropriate frame
                            worldAxisVec3 = childFrame.expressVectorInGround(om.state, localAxis);
                            w_aor = [worldAxisVec3.get(0), worldAxisVec3.get(1), worldAxisVec3.get(2)]';
                        end
                    end
                otherwise
                    fprintf('[get_coordinate_axis]: joint type not defined \n')
            end
        end
        
        function w_aor_all = get_coordinate_axis_all(om)
            % get coordinate position in world frame
            % Omit local position of child frame in parent frame
            % Coord_Name: string
            import org.opensim.modeling.*;
            w_aor_all = [];
            for i = 1: om.CoordinateSet.getSize
                w_aor_all(:,i) = om.get_coordinate_axis(om.CoordinateSet_list{i,1});
            end
        end

        function Jacobian_matrix = getJacobian_mp_all(om, marker_point_index)
            % Jacobian matrix according to the selected marker_point
            % Jacobian [6xn]: translational;rotational
            import org.opensim.modeling.*;
            Jacobian_m = Matrix();
            mp_i = om.marker_point_list{marker_point_index};
            body_index = om.BodySet.get(mp_i.body_name).getMobilizedBodyIndex();% -1;
            om.smss.calcFrameJacobian(om.state, body_index, Vec3(0), Jacobian_m);
            Jacobian_matrix_revert = osimMatrix2matrix(Jacobian_m);
            Jacobian_matrix(1:3,:) = Jacobian_matrix_revert(4:6,:);
            Jacobian_matrix(4:6,:) = Jacobian_matrix_revert(1:3,:);
        end

        function Jacobian_matrix = getJacobian_mp_all_trans(om, marker_point_index)
            % Jacobian matrix according to the selected marker_point
            % Jacobian [6xn]: translational;rotational
            import org.opensim.modeling.*;
            Jacobian_m = Matrix();
            mp_i = om.marker_point_list{marker_point_index};
            body_index = om.BodySet.get(mp_i.body_name).getMobilizedBodyIndex(); % -1
            om.smss.calcStationJacobian(om.state, body_index, Vec3(0), Jacobian_m);
            Jacobian_matrix = osimMatrix2matrix(Jacobian_m);
        end


        function Jacobian_matrix = getJacobian_mp_sub(om, marker_point_index,coordinate_name_list )
            import org.opensim.modeling.*
            Jacobian_m = Matrix();
            coord_index = zeros(length(coordinate_name_list),1);
            for i = 1:length(coordinate_name_list)
                coord_index(i) = find(matches(om.CoordinateInOrder,coordinate_name_list{i}));
            end
            mp_i = om.marker_point_list{marker_point_index};
            body_index = om.BodySet.get(mp_i.body_name).getMobilizedBodyIndex();% -1;
            om.smss.calcFrameJacobian(om.state, body_index, Vec3(0), Jacobian_m);
            Jacobian_matrix_revert = osimMatrix2matrix(Jacobian_m);
            Jacobian_matrix(1:3,:) = Jacobian_matrix_revert(4:6,coord_index);
            Jacobian_matrix(4:6,:) = Jacobian_matrix_revert(1:3,coord_index);
        end

        function J_syn = getJacobian_mp_minimal(om, marker_point_index )
            % J_syn = J_min + * J_con * df/dq  
            % dx = J_syn * dq_minimal
            import org.opensim.modeling.*
            coord_minimal_list = om.Coord_minimal(:,1);
            coord_con_index = [1:length(om.CoordinateInOrder)];
            coord_con_index([om.Coord_minimal{:,2}]) = [];
            n_qmin = length([om.Coord_minimal{:,2}]); % number of indep. coord.
            n_qcon = length(om.CoordinateInOrder)-n_qmin;
            J_all = om.getJacobian_mp_all(marker_point_index);
%             J_min = om.getJacobian_mp_sub(marker_point_index,coord_minimal_list);
            J_min = J_all(:,[om.Coord_minimal{:,2}]);
            J_con = J_all;
            J_con(:,[om.Coord_minimal{:,2}]) = [];
            dfdq = zeros(n_qcon,n_qmin);

            for i = 1:n_qcon
                coordinate_name_i = om.CoordinateSet_list(coord_con_index(i),1);
                q_con_i = om.get_coordinate_value(coordinate_name_i);
                % the cubic function is imported from
                % model.CoordinateSet_list{:,4}
                f_i = om.CoordinateSet_list{coord_con_index(i),4};
                p_der=fnder(f_i,1);
                y_prime= ppval(p_der,q_con_i);
                paraent_frame_name = om.CoordinateSet_list{coord_con_index(i),2};
                coord_index_j = find(matches([om.Coord_minimal(:,1)],paraent_frame_name));

                dfdq(i,coord_index_j) = y_prime;
            end
            J_syn = J_min + J_con * dfdq;
        end
        
        function Jacobian_matrix = getJacobian_point(om, body_name, pos_vec3 )
            import org.opensim.modeling.*
            Jacobian_m = Matrix();
            body_index = om.BodySet.get(body_name).getMobilizedBodyIndex();% -1;
            om.smss.calcFrameJacobian(om.state, body_index, pos_vec3, Jacobian_m);
            Jacobian_matrix_revert = osimMatrix2matrix(Jacobian_m);
            Jacobian_matrix(1:3,:) = Jacobian_matrix_revert(4:6,:);
            Jacobian_matrix(4:6,:) = Jacobian_matrix_revert(1:3,:);
        end

        function Jacobian_matrix = getJacobian_point_sub(om, body_name, pos_vec3, coordinate_name_list )
            import org.opensim.modeling.*
            Jacobian_m = Matrix();
            coord_index = zeros(length(coordinate_name_list),1);
            for i = 1:length(coordinate_name_list)
                coord_index(i) = find(matches(om.CoordinateInOrder,coordinate_name_list{i}));
            end
            body_index = om.BodySet.get(body_name).getMobilizedBodyIndex();% -1;
            om.smss.calcFrameJacobian(om.state, body_index, pos_vec3, Jacobian_m);
            Jacobian_matrix_revert = osimMatrix2matrix(Jacobian_m);
            Jacobian_matrix(1:3,:) = Jacobian_matrix_revert(4:6,coord_index);
            Jacobian_matrix(4:6,:) = Jacobian_matrix_revert(1:3,coord_index);
        end

        


        function Jacobian_ana = getJacobian_mp_sub_ana(om, marker_point_index,coordinate_name_list )
            % analytical Jacobian with euler XYZ
            import org.opensim.modeling.*
            Jacobian_matrix = getJacobian_mp_sub(om, marker_point_index,coordinate_name_list );
            [~,~, w_R] = get_mp_frame(om, marker_point_index);

            Jacobian_ana = [Jacobian_matrix(1:3,:);w_R'*Jacobian_matrix(4:6,:)];
        end

        function MassMatrix = get_MassMatrix_all(om)
            import org.opensim.modeling.*;
            M = Matrix();
            om.smss.calcM(om.state,M);
            MassMatrix = osimMatrix2matrix(M);
        end
        
        function MassMatrix_sub = get_MassMatrix_sub(om, coordinate_name_list)
            import org.opensim.modeling.*;
            M = Matrix();
            om.smss.calcM(om.state,M);
            MassMatrix = osimMatrix2matrix(M);
            for i = 1:length(coordinate_name_list)
                coord_index(i) = find(matches(om.CoordinateInOrder,coordinate_name_list{i}));
            end
            MassMatrix_sub = MassMatrix(coord_index,coord_index);
        end
        function MassMatrix = get_MassMatrix_minimal(om)
            import org.opensim.modeling.*;
            M = om.get_MassMatrix_all;
            MassMatrix = M([om.Coord_minimal{:,2}],[om.Coord_minimal{:,2}]);
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% muscle information
        % length: MuscleLength = TendonLength + FiberLengthAlongTendon 
        %           f(MuscleLength/OptimalMuscleLength) = ForceLengthMultiplier 
        % Force: TendonForce = FiberForceAlongTendon
        %       FiberForce = ActiveFiberForce + PassiveFiberForce
        %       

        function MA_matrix = get_MomentArmMatrix(om, Coord_Name_list, mus_name_list)
            % moment arm matrix 
            %   row: coordinates
            %   colomn: muscles
            import org.opensim.modeling.*
            MA_matrix = zeros(length(Coord_Name_list),length(mus_name_list));
            for i = 1:length(mus_name_list)
                for j = 1:length(Coord_Name_list)
                    muscle_i = om.MuscleSet.get(mus_name_list{i});
                    MA_matrix(j,i) = muscle_i.computeMomentArm(om.state,om.CoordinateSet.get(Coord_Name_list{j}));
                end
            end
        end

        function set_Activation_all(om, mus_act)
            % set all muscle activation
            import org.opensim.modeling.*
            assert(length(mus_act) == length(om.MuscleSet_list), '[set_Activation_all]: input dimension is incorrect!');
            for i = 1:length(om.MuscleSet_list)
                muscle_i = om.MuscleSet.get(om.MuscleSet_list{i});
                muscle_i.setActivation(om.state,mus_act(i));
            end
            om.model.equilibrateMuscles(om.state);
        end

        function mus_act = get_Activation_all(om)
            % set all muscle activation
            import org.opensim.modeling.*
            mus_act = zeros(length(om.MuscleSet_list),1);
            for i = 1:length(om.MuscleSet_list)
                muscle_i = om.MuscleSet.get(om.MuscleSet_list{i});
                mus_act(i) = muscle_i.getActivation(om.state);
            end
        end

        function MA_matrix = get_MomentArmMatrix_minimal_AllMus(om)
            % moment arm matrix of all muscle and minimcal coordinates
            %   row: coordinates
            %   colomn: muscles
            import org.opensim.modeling.*
            MA_matrix = zeros(size(om.Coord_minimal_range,1),length(om.MuscleSet_list));
            for i = 1:length(om.MuscleSet_list)
                for j = 1:size(om.Coord_minimal_range,1)
                    muscle_i = om.MuscleSet.get(om.MuscleSet_list{i});
                    MA_matrix(j,i) = muscle_i.computeMomentArm(om.state,om.CoordinateSet.get(om.Coord_minimal{j,1}));
                end
            end
        end
        
        % muscle forces
        function mus_MIF_vec = get_MaxIsometricForce(om, varargin)
            % maximal isometric force
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_MIF_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_MIF_vec(i) = muscle_i.getMaxIsometricForce;
            end
        end

        function mus_MIF_vec = get_MaxIsometricForce_all_diag(om)
            % maximal isometric force
            import org.opensim.modeling.*
            nmus = length(om.MuscleSet_list);
            mus_MIF_vec = zeros(nmus,nmus);
            for i = 1:nmus
                muscle_i = om.MuscleSet.get(om.MuscleSet_list{i});
                mus_MIF_vec(i,i) = muscle_i.getMaxIsometricForce;
            end
        end

        function mus_TF_vec = get_TendonForce(om, varargin)
            % passive tendon force
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_TF_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_TF_vec(i) = muscle_i.getTendonForce(om.state);
            end
        end
        
        function mus_AFF_vec = get_ActiveFiberForce(om, varargin)
            % passive fiber force
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_AFF_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_AFF_vec(i) = muscle_i.getActiveFiberForce(om.state);
            end
        end

        function mus_FLM = get_FiberForceLengthMultiplier(om, varargin)
            % Fiber Force Length Multiplier
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_FLM = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_FLM(i) = muscle_i.getActiveForceLengthMultiplier(om.state);
            end
        end

        function mus_FVM = get_FiberForceVelocityMultiplier(om, varargin)
            % Fiber Force Length Multiplier
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_FVM = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_FVM(i) = muscle_i.getActiveForceLengthMultiplier(om.state);
            end
        end

        function mus_PFF_vec = get_PassiveFiberForce(om, varargin)
            % passive fiber force
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_PFF_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_PFF_vec(i) = muscle_i.getPassiveFiberForce(om.state);
            end
        end

        function mus_PLM = get_PassiveForceLengthMultiplier(om, varargin)
            % Fiber Force Length Multiplier
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_PLM = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_PLM(i) = muscle_i.getPassiveForceMultiplier(om.state);
            end
        end

        function mus_FF_vec = get_FiberForce(om, varargin)
            % Fiber force = PFF + AFF
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_FF_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_FF_vec(i) = muscle_i.getFiberForce(om.state);
            end
        end

        function mus_FiberForceAlongTendon = get_FiberForceAlongTendon(om, mus_name_list)
            % muscle length
            import org.opensim.modeling.*
            mus_FiberForceAlongTendon = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_FiberForceAlongTendon(i) = muscle_i.getFiberForceAlongTendon(om.state);
            end
        end

        % length
        function mus_PA_vec = get_PennationAngle(om, varargin)
            % Pennation angle
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_PA_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_PA_vec(i) = muscle_i.getPennationAngle(om.state);
            end
        end

        function mus_ML_vec = get_muscleLength(om, mus_name_list)
            % muscle length = TendonLength + FiberLengthAlongTendon
            import org.opensim.modeling.*
            mus_ML_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_ML_vec(i) = muscle_i.getLength(om.state);
            end
        end

        function mus_TL_vec = get_TendonLength(om, mus_name_list)
            % tendon length
            import org.opensim.modeling.*
            mus_TL_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_TL_vec(i) = muscle_i.getTendonLength(om.state);
            end
        end

        function mus_FL_vec = get_FiberLength(om, varargin)
            % fiber length
            import org.opensim.modeling.*
            if nargin == 1
                mus_name_list = om.MuscleSet_list;
            else 
                mus_name_list = varargin{1};
            end
            mus_FL_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_FL_vec(i) = muscle_i.getFiberLength(om.state);
            end
        end

        function mus_OFL_vec = get_OptimalFiberLength(om, mus_name_list)
            % muscle length
            import org.opensim.modeling.*
            mus_OFL_vec = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_OFL_vec(i) = muscle_i.get_optimal_fiber_length();
            end
        end



        function mus_FVfactor = get_ForceVelocityfactor(om, mus_name_list)
            % 
            import org.opensim.modeling.*
            mus_FVfactor = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_FVfactor(i) = muscle_i.getForceVelocityMultiplier(om.state);
            end
        end

        function mus_ActiveFLfactor = get_activeforcelengthfactor(om, mus_name_list)
            % AFLf = f(FL/OFL)
            import org.opensim.modeling.*
            mus_ActiveFLfactor = zeros(length(mus_name_list),1);
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                mus_ActiveFLfactor(i) = muscle_i.getActiveForceLengthMultiplier(om.state);
            end
        end

        function [PathPoints] = get_musclePathPoint(om, mus_name_list)
            % muscle Path Points
            % PathPoint cell array
            % 
            import org.opensim.modeling.*
            PathPoints = {};
            for i = 1:length(mus_name_list)
                muscle_i = om.MuscleSet.get(mus_name_list{i});
                Geometrypath_i = muscle_i.getGeometryPath();
                PathPointSet_i = Geometrypath_i.getPathPointSet;
                num_pp = PathPointSet_i.getSize;
                for j = 1:num_pp
                    PP_i = PathPointSet_i.get(j-1);
                    PathPoints{2*i-1,1} = char(muscle_i.getName);
                    PathPoints{2*i-1,1+j} = char(PP_i.getBodyName);
                    PathPoints{2*i,1+j} = osimVec3ToArray(PP_i.getLocation(om.state));
                end
            end
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% marker information
        function delete_all_markers(om)
            % muscle length
            import org.opensim.modeling.*
            num_marker = om.MarkerSet.getSize;
            for i = 1:num_marker
                om.MarkerSet.remove(om.MarkerSet.get(0));
            end
            om.MarkerSet = om.model.getMarkerSet();
            om.MarkerSet_list = Setlist_read(om.MarkerSet);
        end

        function add_marker_points(om,mp_name, body_name_list, pos_vec_list )
            % add a marker_point at the attached body frame with position
            % and orientation input 
            import org.opensim.modeling.*
            num_marker = length(body_name_list);
            sphere_geometry = Sphere();
            sphere_geometry.set_radius(0.02);
            sphere_geometry.setColor(Vec3(1, 1,0));
            for i = 1:num_marker
                markeri = Body();
                markeri.setName(mp_name);
                markeri.setMass(0);
                markeri.setMassCenter(Vec3(0));
                markeri.setInertia( Inertia(0,0,0,0,0,0) );
                jointi = WeldJoint("Weldjoint", ...
                             om.BodySet.get(body_name_list{i}), ...
                             pos_vec_list{i}, ...
                             Vec3(0), ...
                             markeri, ...
                             Vec3(0, 0, 0), ...
                             Vec3(0, 0, 0));
                om.model.addBody(markeri);
                markeri.attachGeometry(sphere_geometry);
                om.model.addJoint(jointi);
%                 w_R_i = osimMatrix2matrix(om.BodySet.get(body_name_list{i}).getTransformInGround(om.state).R);
                i_R_i = eye(3);
                marker_point_i = marker_point(markeri.getName, body_name_list{i},osimMatrix2matrix(pos_vec_list{i}),i_R_i);
                om.marker_point_list{end+1} = marker_point_i;
            end
            om.update_system;
            om.update_set;
        end

        function [x_p, w_p, w_R] = get_mp_frame(om, mp_index)
            % plot the body frame origin in matlab plot
            import org.opensim.modeling.*;
            num_marker_point = length(om.marker_point_list);
            assert(mp_index<=num_marker_point, 'get_mp_frame: no such marker_point!')
            mp_i = om.marker_point_list{mp_index};
            body_i = om.BodySet.get(mp_i.body_name);
            w_body_p = osimMatrix2matrix(body_i.getTransformInGround(om.state).p);
            w_R = osimMatrix2matrix(body_i.getTransformInGround(om.state).R);
            w_p = w_body_p; % + w_R*mp_i.p;
%             x_p = [w_body_p;rotm2eul(w_R,'XYZ')'];
            x_p = [w_body_p;R2euler_XYZ(w_R)];
        end



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        

        %% metrics

        function [metric_JL,metric_JL_joints] = metric_joint_limits(om)
            % calculate the minimal coordinates

            import org.opensim.modeling.*;
            joint_limits = om.Coord_minimal_range;
            q_all = osimMatrix2matrix(om.state.getQ);
            q_minimal = q_all([om.Coord_minimal{:,2}]);
            metric_JL_joints = metric_joint_limits(q_minimal, joint_limits);
            metric_JL = prod(metric_JL_joints);
        end

    
%         function om = actuate(om, action)
%             action(action>1) = 1;
%             action(action<0) = 0;
%             om.last_action = action;
% 
%             om.brain = org.opensim.modeling.PrescribedController.safeDownCast(om.model.getControllerSet().get(0));
%             functionSet = om.brain.get_ControlFunctions();
% 
%             for j =0:(functionSet.getSize()-1)
%                 func = org.opensim.modeling.Constant.safeDownCast(functionSet.get(j));
%                 func.setValue( double(action(j+1)) );
%             end
%         end
        
%         function om = set_activations(om, activations)
%             for j = 0:(om.muscleSet.getSize()-1)
%                 om.muscleSet.get(j).setActivation(om.state, activations(j+1))
%                 om.reset_manager()
%             end
% 
%         end
        
%         function cur_activations  = get_activations(om)
%             for j = 0:(om.muscleSet.getSize()-1)
%                 cur_activations = om.muscleSet.get(j).getActivation(om.state);
%             end
%         end
            

%         function [res, om] = compute_state_desc(om)
%             om.model.realizeAcceleration(om.state)
% 
%             % Joints
%             joint_pos = containers.Map();
%             joint_vel = containers.Map();
%             joint_acc = containers.Map();
%             joint_pos_value = zeros(om.jointSet.getSize(),3);
%             joint_vel_value = zeros(om.jointSet.getSize(),3);
%             joint_acc_value = zeros(om.jointSet.getSize(),3);
%             for i = 0:(om.jointSet.getSize()-1)
%                 joint = om.jointSet.get(i);
%                 name = joint.getName();
%                 for j = 0:(joint.numCoordinates()-1)
%                     joint_pos_value(i+1,j+1) = joint.get_coordinates(j).getValue(om.state);
%                     joint_vel_value(i+1,j+1) = joint.get_coordinates(j).getSpeedValue(om.state);
%                     joint_acc_value(i+1,j+1) = joint.get_coordinates(j).getAccelerationValue(om.state);
%                 end
%                 joint_pos(char(name)) = joint_pos_value(i+1,:);
%                 joint_vel(char(name)) = joint_vel_value(i+1,:);
%                 joint_acc(char(name)) = joint_acc_value(i+1,:); 
%             end
%             
% 
%             %% Bodies
%             body_pos = containers.Map();
%             body_vel = containers.Map();
%             body_acc = containers.Map();
%             body_pos_rot = containers.Map();
%             body_vel_rot = containers.Map();
%             body_acc_rot = containers.Map();
%             body_pos_value = zeros(om.bodySet.getSize(),3);
%             body_vel_value = zeros(om.bodySet.getSize(),3);
%             body_acc_value = zeros(om.bodySet.getSize(),3);
%             body_pos_rot_value = zeros(om.bodySet.getSize(),3);
%             body_vel_rot_value = zeros(om.bodySet.getSize(),3);
%             body_acc_rot_value = zeros(om.bodySet.getSize(),3);           
%             for i = 0:(om.bodySet.getSize()-1)
%                 body = om.bodySet.get(i);
%                 name = body.getName();
%                 
%                 for j = 0:2
%                     body_pos_value(i+1,j+1) = body.getTransformInGround(om.state).p().get(j);
%                     body_vel_value(i+1,j+1) = body.getVelocityInGround(om.state).get(1).get(j);
%                     body_acc_value(i+1,j+1) = body.getAccelerationInGround(om.state).get(1).get(j);
%                     body_pos_rot_value(i+1,j+1) = body.getTransformInGround(om.state).R().convertRotationToBodyFixedXYZ().get(j);
%                     body_vel_rot_value(i+1,j+1) = body.getVelocityInGround(om.state).get(0).get(j);
%                     body_acc_rot_value(i+1,j+1) = body.getAccelerationInGround(om.state).get(0).get(j);
%                 end
%                                 
%                 body_pos(char(name)) = body_pos_value(i+1,:);
%                 body_vel(char(name)) = body_vel_value(i+1,:);
%                 body_acc(char(name)) = body_acc_value(i+1,:);
%                 body_pos_rot(char(name)) = body_pos_rot_value(i+1,:);
%                 body_vel_rot(char(name)) = body_vel_rot_value(i+1,:);
%                 body_acc_rot(char(name)) = body_acc_rot_value(i+1,:);
%             end
% 
%             %% Forces
% 
%             forces = containers.Map();
%             for i = 22:(om.forceSet.getSize()-1)
%                 force = om.forceSet.get(i);
%                 name = force.getName();
%                 values = force.getRecordValues(om.state);
%                 value_array = zeros(1,values.size);
%                 for j = 0:(values.size-1)
%                     value_array(j+1) = values.get(j);
%                 end
%                 forces(char(name)) = value_array;
%             end
% 
%             %% Muscles
%             muscles = containers.Map();
%             muscles_activation = containers.Map();
%             muscles_fiber_length = containers.Map();
%             muscles_fiber_velocity = containers.Map();
%             muscles_fiber_force = containers.Map();
%             for i = 0:(om.muscleSet.getSize()-1)
%                 muscle = om.muscleSet.get(i);
%                 name = muscle.getName();
%                 muscles(char(name)) = {};
%                 muscles_activation(char(name)) = muscle.getActivation(om.state);
%                 muscles_fiber_length(char(name)) = muscle.getFiberLength(om.state);
%                 muscles_fiber_velocity(char(name)) = muscle.getFiberVelocity(om.state);
%                 muscles_fiber_force(char(name)) = muscle.getFiberForce(om.state);
%                 % We can get more properties from here http://myosin.sourceforge.net/2125/classOpenSim_1_1Muscle.html 
%             end
% 
% %             %% Markers
% %             markers = containers.Map();
% 
% 
%             %% Mass center
%             mass_center = containers.Map();
%             for i = 0:2
%                 mass_center_pos_vec = om.model.calcMassCenterPosition(om.state);
%                 mass_center_vel_vec = om.model.calcMassCenterVelocity(om.state);
%                 mass_center_acc_vec = om.model.calcMassCenterAcceleration(om.state);
%             end
%             mass_center_pos_value = eval(mass_center_pos_vec.toString().substring(1));
%             mass_center_vel_value = eval(mass_center_vel_vec.toString().substring(1));
%             mass_center_acc_value = eval(mass_center_acc_vec.toString().substring(1));
%             mass_center('mass_center_pos') = mass_center_pos_value;
%             mass_center('mass_center_vel') = mass_center_vel_value;
%             mass_center('mass_center_acc') = mass_center_acc_value;
%             
% 
%             
%             res = {joint_pos, joint_vel, joint_acc,...
%                 body_pos, body_vel, body_acc, body_pos_rot, body_vel_rot, body_acc_rot,...
%                 forces,...
%                 muscles_activation, muscles_fiber_length, muscles_fiber_velocity, muscles_fiber_force,...
%                 mass_center};
%                 
%                 
%         end

        
   
%         function [om, state_desc_] = get_state_desc(om)
%             if ~isequal(om.state_desc_istep, om.istep)
%                 om.prev_state_desc = om.state_desc;
%                 om.state_desc = om.compute_state_desc();
%                 om.state_desc_istep = om.istep;
%             end                
%             state_desc_ = om.state_desc;
% 
%         end
        

%         function set_strength(om, strength)
%             om.curforces = strength;
%             for i = 0:(length(om.curforces)-1)
%                 om.muscleSet.get(i).setMaxIsometricForce(om.curforces(i+1) * om.maxforces(i+1))    
%             end
%         end
        
        
        %         function list_elements(om)
% 
%             fprintf("-------JOINTS-------\n")
%             for i = 0:(om.jointSet.getSize()-1)
%                 fprintf('%3d ',i+1)
%                 disp(om.jointSet.get(i).getName())
%                 fprintf('%c%c', 8, 8);
%                 fprintf('\n');
%             end
%             fprintf("\n-------BODIES-------\n")
%             for i = 0:(om.bodySet.getSize()-1)
%                 fprintf('%3d ',i+1)
%                 disp(om.bodySet.get(i).getName())
%                 fprintf('%c%c', 8, 8);
%                 fprintf('\n');
%             end
%             fprintf("\n-------MUSCLES-------\n")
%             for i = 0:(om.muscleSet.getSize()-1)
%                 fprintf('%3d ',i+1)
%                 disp(om.muscleSet.get(i).getName())
%                 fprintf('%c%c', 8, 8);
%                 fprintf('\n');
%             end
%             fprintf("\n-------FORCES-------\n")
%             for i = 0:(om.forceSet.getSize()-1)
%                 fprintf('%3d ',i+1)
%                 disp(om.forceSet.get(i).getName())
%                 fprintf('%c%c', 8, 8);
%                 fprintf('\n');
%             end
%             fprintf("\n-------MARKERS-------\n")
%             for i = 0:(om.markerSet.getSize()-1)
%                 fprintf('%3d ',i+1)
%                 disp(om.markerSet.get(i).getName())
%                 fprintf('%c%c', 8, 8);
%                 fprintf('\n');
%             end
%         end      
        
        
        function body_ = get_body(om, name)
            body_ = om.BodySet.get(name);
        end

        function jointSet_ = get_joint(om, name)
            jointSet_ = om.JointSet.get(name);
        end

        function muscleSet_ = get_muscle(om, name)
            muscleSet_  = om.MuscleSet.get(name);
        end
        
        function markerSet_ = get_marker(om, name)
            markerSet_ = om.MarkerSet.get(name);
        end
        
        function contactGeometrySet_ = get_contact_geometry(om, name)
            contactGeometrySet_ = om.ContactGeometrySet.get(name);
        end
        
        function forceSet_ = get_force(om, name)
            forceSet_ = om.ForceSet.get(name);
        end
        
        function noutput_ = get_action_space_size(om)
            noutput_ = om.noutput;
        end
        
%         function om = set_integrator_accuracy(om, integrator_accuracy)
%             om.integrator_accuracy = integrator_accuracy;       
%         end
        
        
%         function om = reset_manager(om)
%             om.manager = org.opensim.modeling.Manager(om.model);
%             om.manager.setIntegratorAccuracy(om.integrator_accuracy);
%             om.manager.initialize(om.state);
%         end
        
%         function om = reset(om)
%             om.state = om.model.initializeState();
%             om.model.equilibrateMuscles(om.state);
%             om.state.setTime(0);
%             om.istep = 0;
% 
%             om.reset_manager()
%         end

%         function cur_state = get_state(om)
%             cur_state = org.opensim.modeling.State(om.state);
%         end

%         function om = set_state(om, state)
%             om.state = state;
%             om.istep = fix(om.state.getTime() / om.stepsize) ;
%             om.reset_manager();
%         end

%         function om = integrate(om)
%             % Define the new endtime of the simulation
%             om.istep = om.istep + 1;
% 
%             % Integrate till the new endtime
% %             om.state = om.manager.integrate(om.stepsize * om.istep);
% %             om.manager = org.opensim.modeling.Manager(om.model);
% %             om.manager.initialize(om.state);
% %             
%             endTime = om.stepsize * om.istep;
%             om.state = om.manager.integrate(endTime);
%         end
        
           
        
            
    end
end
