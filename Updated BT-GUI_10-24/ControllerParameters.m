classdef ControllerParameters < handle
    properties
        % Basic controller flags
        P_controller_flag = false
        PI_controller_flag = false
        PID_controller_flag = false
        
        % Advanced controller flags
        PI_design_flag = false;
        PI_control_flag = true;
        PID_design_flag = false;
        PID_control_flag = false;
        
        % Controller parameters
        Kp = 0
        Ki = 0
        Kd = 0
        Ti = 0
        tau = 0
        beta = 0
        zeta = 0
        wc = 0
        phim = 0
        
        % Additional settings
        Compare_changes_flag = true
        ShowInputSignal_flag = true
    end
    
    methods
        function obj = ControllerParameters()
            % Constructor can be empty as properties have default values
        end
        
        function copyFromApp(obj, app)
            % Copy all controller parameters from app
            obj.P_controller_flag = app.P_controller_flag;
            obj.PI_controller_flag = app.PI_controller_flag;
            obj.PID_controller_flag = app.PID_controller_flag;
            
            obj.PI_control_flag = app.PI_control_flag;
            obj.PI_design_flag = app.PI_design_flag;
            obj.PID_control_flag = app.PID_control_flag;
            obj.PID_design_flag = app.PID_design_flag;
            
            obj.Kp = app.Kp;
            obj.Ki = app.Ki;
            obj.Kd = app.Kd;
            obj.Ti = app.Ti;
            obj.tau = app.tau;
            obj.beta = app.beta;
            obj.zeta = app.zeta;
            obj.wc = app.wc;
            obj.phim = app.phim;
            
            obj.Compare_changes_flag = app.Compare_changes_flag;
            obj.ShowInputSignal_flag = app.ShowInputSignal_flag;
        end
        
        function copyToApp(obj, app)
            % Copy all controller parameters to app
            app.P_controller_flag = obj.P_controller_flag;
            app.PI_controller_flag = obj.PI_controller_flag;
            app.PID_controller_flag = obj.PID_controller_flag;
            
            app.PI_control_flag = obj.PI_control_flag;
            app.PI_design_flag = obj.PI_design_flag;
            app.PID_control_flag = obj.PID_control_flag;
            app.PID_design_flag = obj.PID_design_flag;
            
            app.Kp = obj.Kp;
            app.Ki = obj.Ki;
            app.Kd = obj.Kd;
            app.Ti = obj.Ti;
            app.tau = obj.tau;
            app.beta = obj.beta;
            app.zeta = obj.zeta;
            app.wc = obj.wc;
            app.phim = obj.phim;
            
            app.Compare_changes_flag = obj.Compare_changes_flag;
            app.ShowInputSignal_flag = obj.ShowInputSignal_flag;
        end
        
        function saveToFile(obj, filename)
            % Save controller parameters to MAT file
            try
                % Create a struct with all parameters
                params = struct();
                props = properties(obj);
                for i = 1:length(props)
                    params.(props{i}) = obj.(props{i});
                end
                
                % Save to file
                save(filename, '-struct', 'params');
            catch ME
                error('Failed to save controller parameters: %s', ME.message);
            end
        end
        
        function loadFromFile(obj, filename)
            % Load controller parameters from MAT file
            try
                % Load the data
                data = load(filename);
                
                % Update properties from loaded data
                props = properties(obj);
                for i = 1:length(props)
                    if isfield(data, props{i})
                        obj.(props{i}) = data.(props{i});
                    end
                end
            catch ME
                error('Failed to load controller parameters: %s', ME.message);
            end
        end
        
        function resetToDefaults(obj)
            % Reset all parameters to default values
            props = properties(obj);
            for i = 1:length(props)
                if isflagproperty(props{i})
                    obj.(props{i}) = false;
                else
                    obj.(props{i}) = 0;
                end
            end
        end
        
        function value = getActiveControllerType(obj)
            % Return the currently active controller type
            if obj.P_controller_flag
                value = 'P';
            elseif obj.PI_controller_flag || obj.PI_control_flag || obj.PI_design_flag
                value = 'PI';
            elseif obj.PID_controller_flag || obj.PID_control_flag || obj.PID_design_flag
                value = 'PID';
            else
                value = 'None';
            end
        end
    end
    
    methods (Access = private)
        function isFlag = isflagproperty(propName)
            % Helper method to check if a property is a flag
            isFlag = contains(propName, '_flag');
        end
    end
end