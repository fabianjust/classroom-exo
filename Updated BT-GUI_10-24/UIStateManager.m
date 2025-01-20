classdef UIStateManager < handle
    properties (Access = private)
        App  % Reference to the main app
        ProgressDialog % Store the progress dialog reference
        
    end
    
    methods
        function obj = UIStateManager(app)
            obj.App = app;
        end

        % Progress Dialog Management Methods
        function showProgress(obj, title, message)
            if nargin < 3
                message = 'Processing...';
            end
            if nargin < 2
                title = 'Please Wait';
            end
            
            % Create progress dialog
            obj.ProgressDialog = uiprogressdlg(obj.App.CLASSROOMEXO_Figure,...
                'Title', title,...
                'Message', message,...
                'Icon', 'info');
        end
        
        function updateProgress(obj, progress, message)
            % Update progress and optionally the message
            if isvalid(obj.ProgressDialog)
                obj.ProgressDialog.Value = progress;
                if nargin > 2
                    obj.ProgressDialog.Message = message;
                end
            end
        end

        function hideProgress(obj)
            % Close the progress dialog
            if isvalid(obj.ProgressDialog)
                close(obj.ProgressDialog);
            end
        end
        
         %% En/Disable all Tab buttons
        function changeConnectionTab(obj, state)
            pathtest = fileparts(mfilename('fullpath'));
            switch state
                case 'disable'
                    obj.App.BTDeviceDropDown.Enable = "off";
                    obj.App.RefreshButton_Connection_List.Enable = "off";
                    obj.App.ConnectButton.Enable = "off";
                    obj.App.EmergnecySTOPButton.Enable = "off";
                case 'enable'
                    obj.App.BTDeviceDropDown.Enable = "on";
                    obj.App.RefreshButton_Connection_List.Enable = "on";
                    obj.App.ConnectButton.Enable = "on";
                    obj.App.EmergnecySTOPButton.Enable = "on";
                case 'connected'
                    obj.App.BTDeviceDropDown.Enable = "off";
                    obj.App.RefreshButton_Connection_List.Enable = "off";
                    obj.App.ConnectButton.Enable = "on";
                    obj.App.EmergnecySTOPButton.Enable = "on";
                case 'initial'
                    obj.App.BTDeviceDropDown.Enable = "off";
                    obj.App.RefreshButton_Connection_List.Enable = "off";
                    obj.App.ConnectButton.Enable = "off";
                    obj.App.EmergnecySTOPButton.Enable = "off";
                    obj.App.BTDeviceDropDown.Items = {};
                    obj.App.BTDeviceDropDown.Enable = "off";
                    obj.App.BTDeviceDropDown.Value = {};
                    obj.App.ConnectButton.Enable = 'off';
                    obj.App.ConnectButton.Text = "Connect";
                    obj.App.BatteryPercentage.Text = "";
                    obj.App.BTStatusImage.ImageSource = fullfile(pathtest, 'photos', 'mat-bluetooth-disabled.png');
                    obj.App.StatusLamp.Color = [0,0,0]./255;
                    obj.App.Label.Text = 'not connected';
            end
        end
        
        function changeTransferFunctionTab(obj, state)
            switch state
                case 'disable'
                    obj.App.PIDControllerPanel.Enable = "off";
                    obj.App.PControllerPanel.Enable = "off";
                    obj.App.PIControllerPanel.Enable = "off";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "off";
                    obj.App.TuningPIPanel_Control_Params.Enable = "off";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Design_Params.Enable = "off";
                    obj.App.Gs_PlantPanel.Enable = "off";
                    obj.App.PLOTButton.Enable = "off";
                    obj.App.OptimizePIButton.Enable = "off";
                    obj.App.OptimizePIDButton.Enable = "off";
                    obj.App.RefreshButton.Enable = "off";
                    obj.App.RefreshButton_3.Enable = "off";
                    obj.App.ServoAngleLimitsSlider.Enable = "off"; 
                    obj.App.LowRefDegreeEditField.Enable = "off";
                    obj.App.HighRefDegreeEditField.Enable = "off";
                    obj.App.InputResponseButtonGroup.Enable = "off";
    
                case 'enable'
                    obj.App.PIDControllerPanel.Enable = "on";
                    obj.App.PControllerPanel.Enable = "on";
                    obj.App.PIControllerPanel.Enable = "on";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "on";
                    obj.App.TuningPIPanel_Control_Params.Enable = "on";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "on";
                    obj.App.TuningPIPanel_Design_Params.Enable = "on";
                    obj.App.Gs_PlantPanel.Enable = "off";
                    obj.App.PLOTButton.Enable = "on";
                    obj.App.OptimizePIButton.Enable = "on";
                    obj.App.OptimizePIDButton.Enable = "on";
                    obj.App.RefreshButton.Enable = "on";
                    obj.App.RefreshButton_3.Enable = "on";
                    obj.App.ServoAngleLimitsSlider.Enable = "on";
                    obj.App.LowRefDegreeEditField.Enable = "on";
                    obj.App.HighRefDegreeEditField.Enable = "on";
                    obj.App.InputResponseButtonGroup.Enable = "on"; 
                    obj.App.Gs_PlantPanel.Enable = "on";
    
                case 'initial'
                    obj.App.PLOTButton.Enable = "on";
                    obj.App.ControlDropDown.Enable = "on";
                    obj.App.OptimizePIButton.Enable = "on";
                    obj.App.ServoAngleLimitsSlider.Enable = "on";
                    obj.App.TuningPIPanel_Control_Params.Enable = "on";
                    obj.App.LowRefDegreeEditField.Enable = "on";
                    obj.App.HighRefDegreeEditField.Enable = "on";
                    obj.App.InputResponseButtonGroup.Enable = "on";  
                    obj.App.Gs_PlantPanel.Enable = "off";
                    obj.App.RefreshButton.Enable = "off";
                    obj.App.RefreshButton_3.Enable = "off";
            end
        end

        function changeControllerDropDownMenu(obj, mode)
            switch mode
                case '1' % PI Control Parameters 
                    obj.App.TuningPIPanel_Control_Params.Visible = "on";
                    obj.App.TuningPIPanel_Control_Params.Enable = "on";
                    obj.App.OptimizePIButton.Visible = "on";
                    obj.App.OptimizePIButton.Enable = "on";
                    obj.App.omega_G150Label.Visible = 'on';
                    obj.App.TuningPIDPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIDPanel_Design_Params.Visible = "off";
                    obj.App.TuningPIPanel_Design_Params.Visible = "off";
                    obj.App.OptimizePIDButton.Visible = "off";
                    obj.App.PControllerPanel.Visible = "off";
                    obj.App.PIControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Enable = "off";
                    obj.App.PIControllerPanel.Enable = "off";
                    obj.App.PControllerPanel.Enable = "off";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "off";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Design_Params.Enable = "off";

                    obj.App.ControllerParams.PI_control_flag = true;
                    obj.App.ControllerParams.PID_control_flag = false;
                    obj.App.ControllerParams.PI_design_flag = false;
                    obj.App.ControllerParams.PID_design_flag = false;
                    obj.App.ControllerParams.P_controller_flag = false;
                    obj.App.ControllerParams.PI_controller_flag = false;
                    obj.App.ControllerParams.PID_controller_flag = false;
                    
               
                case '2' % PI Design Parameters 
                    obj.App.TuningPIPanel_Design_Params.Visible = "on";
                    obj.App.TuningPIPanel_Design_Params.Enable = "on";
                    obj.App.OptimizePIButton.Visible = "on";
                    obj.App.OptimizePIButton.Enable = "on";
                    obj.App.omega_G150Label.Visible = 'on';
                    obj.App.TuningPIDPanel_Design_Params.Visible = "off";
                    obj.App.TuningPIDPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Control_Params.Visible = "off";
                    obj.App.OptimizePIDButton.Visible = "off";
                    obj.App.PControllerPanel.Visible = "off";
                    obj.App.PIControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Enable = "off";
                    obj.App.PIControllerPanel.Enable = "off";
                    obj.App.PControllerPanel.Enable = "off";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "off";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Control_Params.Enable = "off";

                    obj.App.ControllerParams.PI_design_flag = true;
                    obj.App.ControllerParams.PID_control_flag = false;
                    obj.App.ControllerParams.PI_control_flag = false;
                    obj.App.ControllerParams.PID_design_flag = false;
                    obj.App.ControllerParams.P_controller_flag = false;
                    obj.App.ControllerParams.PI_controller_flag = false;
                    obj.App.ControllerParams.PID_controller_flag = false;
                    

                case '3' % PID Control Parameters 
                    obj.App.TuningPIDPanel_Control_Params.Visible = "on";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "on";
                    obj.App.OptimizePIDButton.Visible = "on";
                    obj.App.OptimizePIDButton.Enable = "on";
                    obj.App.omega_G150Label.Visible = 'on';
                    obj.App.TuningPIDPanel_Design_Params.Visible = "off";
                    obj.App.TuningPIPanel_Design_Params.Visible = "off";
                    obj.App.TuningPIPanel_Control_Params.Visible = "off";
                    obj.App.OptimizePIButton.Visible = "off";
                    obj.App.PControllerPanel.Visible = "off";
                    obj.App.PIControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Enable = "off";
                    obj.App.PIControllerPanel.Enable = "off";
                    obj.App.PControllerPanel.Enable = "off";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Control_Params.Enable = "off";

                    obj.App.ControllerParams.PID_control_flag = true;
                    obj.App.ControllerParams.PI_design_flag = false;
                    obj.App.ControllerParams.PI_control_flag = false;
                    obj.App.ControllerParams.PID_design_flag = false;
                    obj.App.ControllerParams.P_controller_flag = false;
                    obj.App.ControllerParams.PI_controller_flag = false;
                    obj.App.ControllerParams.PID_controller_flag = false;
                    
                    

                case '4' % PID Design Parameters 
                    obj.App.TuningPIDPanel_Design_Params.Visible = "on";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "on";
                    obj.App.OptimizePIDButton.Visible = "on";
                    obj.App.OptimizePIDButton.Enable = "on";
                    obj.App.omega_G150Label.Visible = 'on';
                    obj.App.TuningPIDPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Design_Params.Visible = "off";
                    obj.App.OptimizePIButton.Visible = "off";
                    obj.App.PControllerPanel.Visible = "off";
                    obj.App.PIControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Enable = "off";
                    obj.App.PIControllerPanel.Enable = "off";
                    obj.App.PControllerPanel.Enable = "off";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "off";
                    obj.App.TuningPIPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Control_Params.Enable = "off";

                    obj.App.ControllerParams.PID_design_flag = true;
                    obj.App.ControllerParams.PID_control_flag = false;
                    obj.App.ControllerParams.PI_design_flag = false;
                    obj.App.ControllerParams.PI_control_flag = false;
                    obj.App.ControllerParams.P_controller_flag = false;
                    obj.App.ControllerParams.PI_controller_flag = false;
                    obj.App.ControllerParams.PID_controller_flag = false;

                case '5' % P Controller
                    obj.App.PControllerPanel.Visible = "on";
                    obj.App.PControllerPanel.Enable = "on";
                    obj.App.PIControllerPanel.Visible = "off";
                    obj.App.PIDControllerPanel.Visible = "off";
                    obj.App.TuningPIDPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Design_Params.Visible = "off";
                    obj.App.TuningPIDPanel_Design_Params.Visible = "off";
                    obj.App.PIControllerPanel.Enable = "off";
                    obj.App.PIDControllerPanel.Enable = "off";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "off";
                    obj.App.TuningPIPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Control_Params.Enable = "off";
                    obj.App.OptimizePIButton.Visible = "off";
                    obj.App.OptimizePIDButton.Visible = "off";
                    obj.App.omega_G150Label.Visible = 'off';

                    obj.App.ControllerParams.P_controller_flag = true;
                    obj.App.ControllerParams.PID_design_flag = false;
                    obj.App.ControllerParams.PID_control_flag = false;
                    obj.App.ControllerParams.PI_design_flag = false;
                    obj.App.ControllerParams.PI_control_flag = false;
                    obj.App.ControllerParams.PI_controller_flag = false;
                    obj.App.ControllerParams.PID_controller_flag = false;

                case '6' % PI Controller
                    obj.App.PIControllerPanel.Visible = "on";
                    obj.App.PIControllerPanel.Enable = "on";
                    obj.App.PIDControllerPanel.Visible = "off";
                    obj.App.PControllerPanel.Visible = "off";
                    obj.App.TuningPIDPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Design_Params.Visible = "off";
                    obj.App.TuningPIDPanel_Design_Params.Visible = "off";
                    obj.App.PIDControllerPanel.Enable = "off";
                    obj.App.PControllerPanel.Enable = "off";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "off";
                    obj.App.TuningPIPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Control_Params.Enable = "off";
                    obj.App.OptimizePIButton.Visible = "off";
                    obj.App.OptimizePIDButton.Visible = "off";
                    obj.App.omega_G150Label.Visible = 'off';

                    obj.App.ControllerParams.PI_controller_flag = true;
                    obj.App.ControllerParams.PID_design_flag = false;
                    obj.App.ControllerParams.PID_control_flag = false;
                    obj.App.ControllerParams.PI_design_flag = false;
                    obj.App.ControllerParams.PI_control_flag = false;
                    obj.App.ControllerParams.P_controller_flag = false;
                    obj.App.ControllerParams.PID_controller_flag = false;

                case '7' % PID Controller
                    obj.App.PIDControllerPanel.Visible = "on";
                    obj.App.PIDControllerPanel.Enable = "on";
                    obj.App.PIControllerPanel.Visible = "off";
                    obj.App.PControllerPanel.Visible = "off";
                    obj.App.TuningPIDPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Control_Params.Visible = "off";
                    obj.App.TuningPIPanel_Design_Params.Visible = "off";
                    obj.App.TuningPIDPanel_Design_Params.Visible = "off";
                    obj.App.PIControllerPanel.Enable = "off";
                    obj.App.PControllerPanel.Enable = "off";
                    obj.App.TuningPIDPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIDPanel_Control_Params.Enable = "off";
                    obj.App.TuningPIPanel_Design_Params.Enable = "off";
                    obj.App.TuningPIPanel_Control_Params.Enable = "off";

                    obj.App.OptimizePIButton.Visible = "off";
                    obj.App.OptimizePIDButton.Visible = "off";
                    obj.App.omega_G150Label.Visible = 'off';
                    obj.App.ControllerParams.PID_controller_flag = true;
                    obj.App.ControllerParams.PID_design_flag = false;
                    obj.App.ControllerParams.PID_control_flag = false;
                    obj.App.ControllerParams.PI_design_flag = false;
                    obj.App.ControllerParams.PI_control_flag = false;
                    obj.App.ControllerParams.P_controller_flag = false;
                    obj.App.ControllerParams.PI_controller_flag = false;
            end

        end
    
        function changeServoControlPanel(obj, state)
            switch state
                case 'disable'
                    obj.App.Servo_Plot_Reset.Enable = "off";
                    obj.App.ExecuteButton.Enable = "off";
                    obj.App.ServoPowerSwitch.Enable = "off";
                    obj.App.ServoPowerSwitch.Value = "Off";
                    obj.App.ServoSpeedKnob.Enable = "off";
                    
                case 'enable'
                    obj.App.Servo_Plot_Reset.Enable = "on";
                    obj.App.ExecuteButton.Enable = "on";
                    obj.App.ServoPowerSwitch.Enable = "on";
                    obj.App.ServoSpeedKnob.Enable = "on";

                case 'initial'
                    obj.App.Servo_Plot_Reset.Enable = "off";
                    obj.App.ExecuteButton.Enable = "on";
                    obj.App.ServoPowerSwitch.Enable = "on";
                    obj.App.ServoSpeedKnob.Enable = "on";
            end
        end
    
        function changeEMGControlTab(obj, state)
            switch state
                case 'disable'
                    obj.App.EMGControlParameters_2.Enable = "off";
                    obj.App.StartRecordingSwitch_DualThresh_EMG.Enable = "off";
                    obj.App.CH1DropDown.Enable = "off";
                    obj.App.CH2DropDown.Enable = "off";
                    obj.App.EMGModeSwitch.Enable = "off";
                    obj.App.ApplyThresholdsButton_EMG.Enable = "off";
                    obj.App.SpeedSetting_EMG.Enable = "off";
                    obj.App.EMGLowerThreshold.Enable = "off";
                    obj.App.EMGUpperThreshold.Enable = "off";
                    obj.App.EMGLowerThresholdSlider.Enable = "off";
                    obj.App.EMGUpperThresholdSlider.Enable = "off";
                    obj.App.AppliedFilterButtonGroup_EMG.Enable = "off";
    
                case 'enable'
                    obj.App.EMGControlParameters_2.Enable = "on";   
                    obj.App.StartRecordingSwitch_DualThresh_EMG.Enable = "on";
                    obj.App.CH1DropDown.Enable = "on";
                    obj.App.CH2DropDown.Enable = "on";
                    obj.App.EMGModeSwitch.Enable = "on";
                    obj.App.ApplyThresholdsButton_EMG.Enable = "on";
                    obj.App.SpeedSetting_EMG.Enable = "on";
                    obj.App.EMGLowerThreshold.Enable = "on";
                    obj.App.EMGUpperThreshold.Enable = "on";
                    obj.App.EMGLowerThresholdSlider.Enable = "on";
                    obj.App.EMGUpperThresholdSlider.Enable = "on";
                    obj.App.AppliedFilterButtonGroup_EMG.Enable = "on";
    
                case 'initial'
                    obj.App.EMGControlParameters_2.Enable = "on"; 
                    obj.App.StartRecordingSwitch_DualThresh_EMG.Enable = "on";
                    obj.App.CH1DropDown.Enable = "on";
                    obj.App.CH2DropDown.Enable = "on";
                    obj.App.EMGModeSwitch.Enable = "on";
                    obj.App.ApplyThresholdsButton_EMG.Enable = "off";
                    obj.App.SpeedSetting_EMG.Enable = "on";
                    obj.App.EMGLowerThreshold.Enable = "on";
                    obj.App.EMGUpperThreshold.Enable = "on";
                    obj.App.EMGLowerThresholdSlider.Enable = "on";
                    obj.App.EMGUpperThresholdSlider.Enable = "on";
                    obj.App.AppliedFilterButtonGroup_EMG.Enable = "on";
    
                case 'running'
                    obj.App.EMGControlParameters_2.Enable = "on"; 
                    obj.App.StartRecordingSwitch_DualThresh_EMG.Enable = "on";
                    obj.App.CH1DropDown.Enable = "off";
                    obj.App.CH2DropDown.Enable = "off";
                    obj.App.EMGModeSwitch.Enable = "off";
                    obj.App.ApplyThresholdsButton_EMG.Enable = "on";
                    obj.App.SpeedSetting_EMG.Enable = "off";
                    obj.App.EMGLowerThreshold.Enable = "on";
                    obj.App.EMGUpperThreshold.Enable = "on";
                    obj.App.EMGLowerThresholdSlider.Enable = "on";
                    obj.App.EMGUpperThresholdSlider.Enable = "on";
                    obj.App.AppliedFilterButtonGroup_EMG.Enable = "on";

                case 'mono-emg'
                    obj.App.CH2DropDown.Visible = "off";
                    obj.App.CH1DropDown.Visible = "on";
                    obj.App.CH2DropDown.Enable = "off";
                    obj.App.CH1DropDown.Enable = "on";
                    obj.App.UpperLabel.Text = 'Upper';
                    obj.App.LowerLabel.Text = 'Lower';

                case 'bi-emg'
                    obj.App.CH2DropDown.Visible = "on";
                    obj.App.CH1DropDown.Visible = "on";
                    obj.App.CH2DropDown.Enable = "on";
                    obj.App.CH1DropDown.Enable = "on";
                    obj.App.UpperLabel.Text = 'Thresh1';
                    obj.App.LowerLabel.Text = 'Thresh2';

            end
        end
    
        function changeForceControlTab(obj, state)
            switch state
                case 'disable'
                    obj.App.ForceParameters.Enable = "off";
                    obj.App.StartRecordingSwitch_DualThresh_Force.Enable = "off";
                    obj.App.ApplyThresholdsButton_Force.Enable = "off";
                    obj.App.SpeedSetting_Force.Enable = "off";
                    obj.App.ForceLowerThresholdSlider.Enable = "off";
                    obj.App.ForceUpperThresholdSlider.Enable = "off";
                    obj.App.AppliedFilterButtonGroup_EMG.Enable = "off";
                    obj.App.proportionalgainSlider.Enable = "off";
    
                case 'enable'
                    obj.App.ForceParameters.Enable = "on";
                    obj.App.StartRecordingSwitch_DualThresh_Force.Enable = "on";
                    obj.App.ApplyThresholdsButton_Force.Enable = "on";
                    obj.App.SpeedSetting_Force.Enable = "on";
                    obj.App.ForceLowerThresholdSlider.Enable = "on";
                    obj.App.ForceUpperThresholdSlider.Enable = "on";
                    obj.App.AppliedFilterButtonGroup_Force.Enable = "on";
    
                case 'initial'
                    obj.App.ForceParameters.Enable = "on";
                    obj.App.StartRecordingSwitch_DualThresh_Force.Enable = "on";
                    obj.App.ApplyThresholdsButton_Force.Enable = "off";
                    obj.App.SpeedSetting_Force.Enable = "on";
                    obj.App.ForceLowerThresholdSlider.Enable = "on";
                    obj.App.ForceUpperThresholdSlider.Enable = "on";
                    obj.App.AppliedFilterButtonGroup_Force.Enable = "on";
                    obj.App.proportionalgainSlider.Enable = "on";
    
                case 'running'
                    obj.App.ForceParameters.Enable = "on";
                    obj.App.StartRecordingSwitch_DualThresh_Force.Enable = "on";
                    obj.App.ApplyThresholdsButton_Force.Enable = "on";
                    obj.App.SpeedSetting_Force.Enable = "off";
                    obj.App.ForceLowerThresholdSlider.Enable = "on";
                    obj.App.ForceUpperThresholdSlider.Enable = "on";
                    obj.App.AppliedFilterButtonGroup_Force.Enable = "on";
                    obj.App.proportionalgainSlider.Enable = "off";

                case 'standard'
                    obj.App.proportionalgainSlider.Visible = "off";
                    obj.App.proportionalgainSlider.Enable = "off";
                    obj.App.proportionalgainSliderLabel. Visible = "off";

                case 'proportional'
                    obj.App.proportionalgainSlider.Visible = "on";
                    obj.App.proportionalgainSlider.Enable = "on";
                    obj.App.proportionalgainSliderLabel. Visible = "on";
            end
        end

        function AdjustThresholdsGUI(obj, type, source)
            % type should be either 'EMG' or 'Force'
            upper_slider = obj.App.([type 'UpperThresholdSlider']);
            lower_slider = obj.App.([type 'LowerThresholdSlider']);
            if strcmpi(type, 'EMG')
                upper_display = obj.App.([type 'UpperThreshold']);
                lower_display = obj.App.([type 'LowerThreshold']);
            end
            
            % Get values based on source
            if strcmpi(source, 'Slider')
                thresh_upper = round(upper_slider.Value);
                thresh_lower = round(lower_slider.Value);
            else % Display
                thresh_upper = round(upper_display.Value);
                thresh_lower = round(lower_display.Value);
            end
        
            if thresh_lower >= thresh_upper && (thresh_upper >= 1)
                thresh_lower = thresh_upper - 1;
            elseif (thresh_lower >= thresh_upper) && (thresh_upper == 0)
                thresh_lower = 0;
                thresh_upper = 1;
            end
        
            if thresh_lower < thresh_upper
                upper_slider.Value = thresh_upper;
                lower_slider.Value = thresh_lower;
                upper_display.Value = thresh_upper;
                lower_display.Value = thresh_lower;
            end
        end


        function resetPlotsGUI(obj, type, source)

        end

    end
end