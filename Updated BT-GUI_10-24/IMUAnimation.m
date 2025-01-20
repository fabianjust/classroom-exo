classdef IMUAnimation < handle
    properties
        Axes
        UpperArmPatch
        LowerArmPatch
        PosePlotShoulder
        PosePlotElbow
        UpperArmSTL
        LowerArmSTL
        TorsoSTL
        HeadSTL
        IMUFilter
        LastUpdateTime
        BufferSize = 1000  % Store last 1000 samples
        DataBuffer
        BufferIndex = 1
    end


    methods (Access = public)
        function obj = IMUAnimation(ax)
            obj.Axes = ax;
            obj.initializePlot();
        end

         function initializeBuffer(obj)
            obj.DataBuffer = zeros(obj.BufferSize, 7); % 6 IMU values + 1 angle
            obj.BufferIndex = 1;
            obj.LastUpdateTime = tic;
        end

        
        function initializePlot(obj)
            cla(obj.Axes, "reset");
            radius = 2.5;
            % radius = 150;
            lims = radius * 2;
            
            obj.UpperArmSTL = stlread('stl/arm.stl');
            obj.LowerArmSTL = stlread('stl/arm.stl');
            % obj.UpperArmSTL = stlread('stl/EduExo_UpperArm v2.stl');
            % obj.LowerArmSTL = stlread('stl/EduExo_LowerArm v1.stl');
            obj.TorsoSTL = stlread('stl/torso.stl');
            obj.HeadSTL = stlread('stl/head.stl');
            default_upper_arm_pos = load("upper_arm_default.mat");
            default_lower_arm_pos = load("lower_arm_default.mat");
            
            % The default order for Euler angle rotations is "ZYX".
            obj.UpperArmSTL = obj.transformStl(obj.UpperArmSTL,[-0.25 0 0],eul2quat([0 0 pi]));
            obj.LowerArmSTL = obj.transformStl(obj.LowerArmSTL,[0.25 0 0],eul2quat([pi 0 0]));
            % obj.UpperArmSTL = obj.transformStl(obj.UpperArmSTL,[0 150 0],eul2quat([pi pi pi]));
            % obj.LowerArmSTL = obj.transformStl(obj.LowerArmSTL,[0 0 0],eul2quat([pi/2 -pi pi])); % 0 -pi pi
            
            
            obj.PosePlotShoulder = poseplot("Parent", obj.Axes);
            hold(obj.Axes, 'on');
            obj.PosePlotElbow = poseplot("Parent", obj.Axes);

            obj.Axes.YLim = [-lims lims];
            obj.Axes.XLim = [-lims lims];
            obj.Axes.ZLim = [-lims lims];
            obj.Axes.XGrid = 'off';
            obj.Axes.YGrid = 'off';
            obj.Axes.ZGrid = 'off';
            obj.Axes.XMinorGrid = 'off';
            obj.Axes.YMinorGrid = 'off';
            obj.Axes.ZMinorGrid = 'off';
            obj.Axes.Color = 'none';
            
            obj.UpperArmPatch = trisurf(obj.UpperArmSTL, "Parent", obj.Axes);
            obj.LowerArmPatch = trisurf(obj.LowerArmSTL, "Parent", obj.Axes);

            set(obj.UpperArmPatch, 'Vertices', default_upper_arm_pos.upper_arm_.Points, 'Faces', default_upper_arm_pos.upper_arm_.ConnectivityList, 'FaceColor', 'r');
            set(obj.LowerArmPatch, 'Vertices', default_lower_arm_pos.lower_arm_.Points, 'Faces', default_lower_arm_pos.lower_arm_.ConnectivityList, 'FaceColor', 'y');

            %
            
            Fs = 50;
            obj.IMUFilter = imufilter('SampleRate', Fs, 'DecimationFactor', 1);
            obj.initializeBuffer();
        end

        function resetFilter(obj)
            % Reset the IMU filter and buffer
            obj.IMUFilter.reset();
            obj.initializeBuffer();
        end
       
        function update(obj, newData, angle_deg)
            % Store data in circular buffer
            obj.DataBuffer(obj.BufferIndex, :) = [newData, angle_deg];
            obj.BufferIndex = mod(obj.BufferIndex, obj.BufferSize) + 1;

            % Check update rate (limit to 60 FPS)
            if toc(obj.LastUpdateTime) < 1/60
                return;
            end
            obj.LastUpdateTime = tic;

            
            radius = 2.5;
            gyro = cast(newData(:,1:3), 'double') * pi/180;
            accel = cast(newData(:,4:6), 'double') * 9.81;
            angle_rad = (cast(angle_deg, 'double') / 1800) * pi;
            
            upper_arm_quat = obj.IMUFilter(accel, gyro);
            upper_arm_pos = obj.calcImuPos(upper_arm_quat, radius);
            upper_arm_rotm = quat2rotm(upper_arm_quat);
            [lower_arm_pos, lower_arm_rotm] = obj.calcServoPos(angle_rad);
            global_rotm = upper_arm_rotm * lower_arm_rotm;
            
            obj.plotPose(obj.PosePlotElbow, upper_arm_pos', quaternion(rotm2quat(global_rotm)));
            upper_arm_ = obj.transformStl(obj.UpperArmSTL, upper_arm_pos', upper_arm_quat);
            lower_arm_ = obj.transformStl(obj.LowerArmSTL, upper_arm_pos', quaternion(rotm2quat(global_rotm)));
            
            set(obj.UpperArmPatch, 'Vertices', upper_arm_.Points, 'Faces', upper_arm_.ConnectivityList, 'FaceColor', 'r');
            set(obj.LowerArmPatch, 'Vertices', lower_arm_.Points, 'Faces', lower_arm_.ConnectivityList, 'FaceColor', 'y');

            % Clear unused variables
            clear upper_arm_ lower_arm_;
        end

        function cleanup(obj)
            % Reset filter and buffer without clearing STL objects
            obj.DataBuffer = [];
            obj.resetFilter();
            drawnow();  % Ensure graphics are updated
        end
    end


    methods (Access = public)

        %% IMU Plotting functions
        %------------------------------------------------------------------
        function new_stl=transformStl(obj,stl,p,quat)
            T=se3(quat2rotm(quat),p);
            points = stl.Points;
            t_points = transform(T,points);
            clist = stl.ConnectivityList;
            new_stl=triangulation(clist,t_points);
        end
    
        function tP=calcImuPos(obj,quat,radius)
            norm_vect=[0 -1 0]'; %y column vector
            rotm = quat2rotm(quat); %get rot matrix from quaternion
            nVec = rotm*norm_vect; %get normal vector of xy-plane (Z axis)
            tP = radius*nVec;   
        end
    
        function [tP, rotm]=calcServoPos(obj,theta)
            x = cos(theta);
            y = sin(theta);
            z = 0;
            %rotm = quat2rotm(quat); %get rot matrix from quaternion
            %z, y, x
            rotm = eul2rotm([theta, 0,0]);
            tP = [x; y; z];   
        end


        function plotPose(obj,property, p,q)
            set(property,Position = p,Orientation = q);
        end

    end
end
