classdef SensorPlotManager < handle
    properties
        Plots = struct()  % Container for different sensor plots
        DataAcquisitionTimer
        IsPlotting
        counter
        CommunicationFunctions
        ActiveSensors = struct('imu', false, 'force', false, 'emg', false)
    end
    
    methods
        function obj = SensorPlotManager(commFunctions)
            obj.CommunicationFunctions = commFunctions;
            obj.DataAcquisitionTimer = timer('ExecutionMode', 'fixedRate', ...
                                            'Period', 1/50, ... % 20 Hz
                                            'TimerFcn', @(~,~)obj.acquireAndUpdateData2());
        end
        
        function initializePlot(obj, sensorType, sensorAx, imuAx, varargin)
            switch lower(sensorType)
                case 'imu'
                    obj.Plots.imu = IMUAnimation(imuAx);
                case {'force', 'emg'}
                    obj.Plots.imu = IMUAnimation(imuAx);
                    % obj.Plots.(sensorType) = SensorVisualization(ax, sensorType, varargin{:});
                    newPlot = SensorVisualization(sensorAx, sensorType, varargin{:});
                    newPlot.initializePlot();  % Call initializePlot after creation
                    obj.Plots.(sensorType) = newPlot;
            end
        end

        function clearPlots(obj)
            % First check if timer is running and stop it
            if ~isempty(obj.DataAcquisitionTimer) && isvalid(obj.DataAcquisitionTimer) && ...
                    strcmp(obj.DataAcquisitionTimer.Running, 'on')
                stop(obj.DataAcquisitionTimer);
            end
            
            % Clear Force plot
            if isfield(obj.Plots, 'force') && ~isempty(obj.Plots.force) && isvalid(obj.Plots.force)
                if isvalid(obj.Plots.force.RawLine)
                    clearpoints(obj.Plots.force.RawLine);
                end
                if isvalid(obj.Plots.force.FilteredLine)
                    clearpoints(obj.Plots.force.FilteredLine);
                end
            end
            
            % Clear EMG plot
            if isfield(obj.Plots, 'emg') && ~isempty(obj.Plots.emg) && isvalid(obj.Plots.emg)
                if isvalid(obj.Plots.emg.RawLine)
                    clearpoints(obj.Plots.emg.RawLine);
                end
                if isvalid(obj.Plots.emg.FilteredLine)
                    clearpoints(obj.Plots.emg.FilteredLine);
                end
                % if(obj.)
                if ~isempty(obj.Plots.emg.RawLine2)
                    if isvalid(obj.Plots.emg.RawLine2)
                        clearpoints(obj.Plots.emg.RawLine2);
                    end
                    if isvalid(obj.Plots.emg.FilteredLine2)
                        clearpoints(obj.Plots.emg.FilteredLine2);
                    end
                end
            end

            
            % Clear IMU plot
            if isfield(obj.Plots, 'imu') && ~isempty(obj.Plots.imu) && isvalid(obj.Plots.imu)
                if isvalid(obj.Plots.imu.Axes)
                    % cla(obj.Plots.imu.Axes);
                end
            end
        end
        
        function start(obj, sensorTypes)
            if ~iscell(sensorTypes)
                sensorTypes = {sensorTypes};
            end

            % Stop timer if it's running
            if ~isempty(obj.DataAcquisitionTimer) && isvalid(obj.DataAcquisitionTimer) && ...
                    strcmp(obj.DataAcquisitionTimer.Running, 'on')
                stop(obj.DataAcquisitionTimer);
            end
            
            % Reset counter and clear existing plots
            obj.counter = 1;
            obj.IsPlotting = false;  % Temporarily set to false while clearing
            obj.clearPlots();
            
            % Reset active sensors
            obj.ActiveSensors.imu = false;
            obj.ActiveSensors.force = false;
            obj.ActiveSensors.emg = false;
            
            % Start appropriate sensors
            obj.IsPlotting = true;
            
            % Start appropriate sensors
            for i = 1:length(sensorTypes)
                switch lower(sensorTypes{i})
                    case 'imu'
                        obj.CommunicationFunctions.startAcquisition(0x44);
                        obj.ActiveSensors.imu = true;
                    case 'force'
                        obj.CommunicationFunctions.setServoState(0x52)
                        obj.CommunicationFunctions.startAcquisition(0x41);
                        obj.ActiveSensors.force = true;
                        obj.ActiveSensors.imu = true;
                    case 'emg'
                        if obj.Plots.emg.ShowSecondSensor
                            obj.CommunicationFunctions.setServoState(0x52)
                            obj.CommunicationFunctions.startAcquisition(0x42);
                        else
                            obj.CommunicationFunctions.setServoState(0x52)
                            obj.CommunicationFunctions.startAcquisition(0x40);
                        end
                            obj.ActiveSensors.emg = true;
                            obj.ActiveSensors.imu = true;                        
                end
            end
            
            % Start timer only if it's valid
            if ~isempty(obj.DataAcquisitionTimer) && isvalid(obj.DataAcquisitionTimer)
                set(obj.DataAcquisitionTimer, 'TimerFcn', @(~,~)obj.acquireAndUpdateData2());
                start(obj.DataAcquisitionTimer);
            end
        end
        
        function stop(obj, sensorTypes)
            if nargin < 2  % Stop all if no specific sensor specified
                obj.IsPlotting = false;
                obj.ActiveSensors.imu = false;
                obj.ActiveSensors.force = false;
                obj.ActiveSensors.emg = false;
                stop(obj.DataAcquisitionTimer);
                % delete(obj.DataAcquisitionTimer);
                % obj.DataAcquisitionTimer.UserData = [];  % Clear any stored data
                % set(obj.DataAcquisitionTimer, 'TimerFcn', '');  % Clear the callback
                obj.CommunicationFunctions.setServoState(0x53);
                obj.CommunicationFunctions.stopAcquisition();
                pause(0.5)
                return
            end
            
            if ~iscell(sensorTypes)
                sensorTypes = {sensorTypes};
            end
            
            % Stop specified sensors
            for i = 1:length(sensorTypes)
                obj.ActiveSensors.(lower(sensorTypes{i})) = false;
            end
            
            % If no sensors active, stop timer
            if ~any(structfun(@(x) x, obj.ActiveSensors))
                obj.IsPlotting = false;
                stop(obj.DataAcquisitionTimer);
                obj.DataAcquisitionTimer.UserData = [];  % Clear any stored data
                set(obj.DataAcquisitionTimer, 'TimerFcn', '');  % Clear the callback
                obj.CommunicationFunctions.stopAcquisition();
                % pause(0.5)
            end
        end
        
        function acquireAndUpdateData2(obj)
            try
                if obj.IsPlotting
                
                    obj.counter=obj.counter+1;
                    
                    % Update IMU
                    if obj.ActiveSensors.imu && isfield(obj.Plots, 'imu')
                        [newdata, angle_deg] = obj.CommunicationFunctions.getIMU_raw(1);
                        obj.Plots.imu.update(newdata, angle_deg);
                    end
                    
                    % Update Force
                    if obj.ActiveSensors.force && isfield(obj.Plots, 'force')
                        [temp_force, imu_data, angle_deg] = obj.CommunicationFunctions.getForce_IMU_Angle(1);
                        force_data = double(temp_force);
                        filtered_force = obj.Plots.force.filterData(force_data);
                        obj.Plots.force.update(obj.counter, force_data, filtered_force);
                        % Update IMU visualization using IMUAnimation
                        obj.Plots.imu.update(imu_data, angle_deg);
                    end
                    
                    % Update EMG
                    if obj.ActiveSensors.emg && isfield(obj.Plots, 'emg')
                        % Get second sensor data only if needed
                        if obj.Plots.emg.ShowSecondSensor
                            emg_data = obj.CommunicationFunctions.getTwoEMG_IMU_Angle(1) * 0.3323;  
                            filtered_emg = obj.Plots.emg.filterData(emg_data);
                            obj.Plots.emg.update(obj.counter, emg_data(1), filtered_emg(1), emg_data(2), filtered_emg(2));
                        else 
                            emg_data = obj.CommunicationFunctions.getOneEMG_IMU_Angle(1) * 0.3323;
                            filtered_emg = obj.Plots.emg.filterData(emg_data);
                            obj.Plots.emg.update(obj.counter, emg_data, filtered_emg);
                        end
                    end
                    
                else
                    obj.counter= 1;
                    return
                end

            catch ME
                warning(ME.identifier, 'Error in timer callback: %s', ME.message);
                obj.stop();  % Stop acquisition if there's an error
            end
        end

        
        function setThresholdsActive(obj, sensorType, upperThresh, lowerThresh)
            if isfield(obj.Plots, sensorType)
                obj.Plots.(sensorType).setThresholds([upperThresh, lowerThresh]);
            else
                error('No plot exists for sensor type: %s', sensorType);
            end
        end
        
        function setThresholdsVisible(obj, sensorType, visible)
            if isfield(obj.Plots, sensorType)
                obj.Plots.(sensorType).setShowThresholds(visible);
            else
                error('No plot exists for sensor type: %s', sensorType);
            end
        end

        function setFilterType(obj, sensorType, filterType, filterMode)
            if isfield(obj.Plots, sensorType)
                obj.Plots.(sensorType).setFilterMode(filterType, filterMode);
            else
                error('No plot exists for sensor type: %s', sensorType);
            end
        end
    end
end