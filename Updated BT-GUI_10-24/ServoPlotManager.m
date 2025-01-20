classdef ServoPlotManager < handle
    properties
        App  % Single reference to the main app
        TrajectoryLine  
        FeedbackLine   
        CurrentIndex
        
        % Controller parameters
        ControllerParams

        % Danger Limits
        Danger_degree = 120;
    end
    
    methods
        function obj = ServoPlotManager(app)
            % Store reference to app
            obj.App = app;

            % Initialize servo trajectory plot
            obj.inititalizeServoPlot();
            
            % Initialize both step plots
            obj.initializeStepPlot(obj.App.StepReference, 'reference');
            obj.initializeStepPlot(obj.App.StepDisturbance, 'disturbance');
            
        end

        function inititalizeServoPlot(obj)
            % Reset axes first
            cla(obj.App.ServoTrajectory);
            hold(obj.App.ServoTrajectory, 'on');
            
            % Apply configuration
            PlotConfig.Servo.applyTo(obj.App.ServoTrajectory);
            
            % Create plot lines
            obj.TrajectoryLine = animatedline(obj.App.ServoTrajectory,'LineStyle', '-');
            obj.FeedbackLine = animatedline(obj.App.ServoTrajectory,'LineStyle', '--', 'Color', 'cyan');
            
            % Set up legend
            legend(obj.App.ServoTrajectory, [obj.TrajectoryLine obj.FeedbackLine], ...
                {'Servo Trajectory', 'Servo Feedback'}, 'Location', 'northeast');
        end

        function initializeStepPlot(obj, plotHandle, plotType)
            % Reset axes first
            cla(plotHandle);
            hold(plotHandle, 'off');

            % Get and apply appropriate configuration
            stepConfig = PlotConfig.Step;
            if strcmp(plotType, 'reference')
                stepConfig.Title = 'Step Plot: Reference Tracking';
            else
                stepConfig.Title = 'Step Plot: Disturbance Response';
            end
            stepConfig.applyTo(plotHandle);            
        end
        
        function updateStepResponse(obj, time, Plot_Value, input_signal)
            try
                % Configure plot
                hold(obj.App.StepReference,'on');
                obj.App.StepReference.XLim = [0 length(time)];
                obj.App.StepReference.XTick = 0:50:length(time);
                obj.App.StepReference.XTickLabel = 0:(0.02*50):max(time);
                obj.App.StepReference.YGrid = 'on';
                
                % Create time vector and plot response
                time_vector = linspace(1,length(time), length(Plot_Value));
                line(obj.App.StepReference, time_vector', Plot_Value);
                
                % Plot input signal if enabled
                if obj.App.ControllerParams.ShowInputSignal_flag
                    line(obj.App.StepReference, time_vector', input_signal, 'LineStyle','--');
                end
                
                % Add controller parameters text if enabled
                if obj.App.ControllerParams.Compare_changes_flag
                    [max_value, max_index] = max(Plot_Value);
                    obj.addControllerText(max_index, max_value);
                end
                
                hold(obj.App.StepReference,'off');
            catch ME
                report = getReport(ME);
                uialert(obj.App.CLASSROOMEXO_Figure, report, "Error", "Icon", "error");
            end
        end
        
        function addControllerText(obj, max_index, max_value)
            params = obj.App.ControllerParams;
            if params.PI_control_flag
                text(obj.App.StepReference, max_index, max_value, sprintf('Ti_{i} = %.1f \nK_{i} = %.1f', params.Ti, params.Ki), 'Interpreter','tex');
            elseif params.PI_design_flag
                text(obj.App.StepReference, max_index, max_value, sprintf('\\omega_{c} = %.1f \\phi_{m} = %.1f', params.wc, params.phim), 'Interpreter','tex');
            elseif params.PID_control_flag
                text(obj.App.StepReference, max_index, max_value, sprintf('K_{i} = %.1f \\tau = %f \n\\beta = %.1f \\zeta = %.1f', params.Ki, params.tau, params.beta, params.zeta), 'Interpreter','tex');
            elseif params.PID_design_flag
                text(obj.App.StepReference, max_index, max_value, sprintf('\\omega_{c} = %.1f \\phi_{m} = %.1f \n\\beta = %.1f \\zeta = %.1f', params.wc, params.phim, params.beta, params.zeta), 'Interpreter','tex');
            elseif params.P_controller_flag
                text(obj.App.StepReference, max_index, max_value, sprintf('\\K_{p} = %.1f', params.Kp), 'Interpreter','tex');
            elseif params.PI_controller_flag
                text(obj.App.StepReference, max_index, max_value, sprintf('\\K_{p} = %.1f \\K_{i} = %.1f \n', params.Kp, params.Ki), 'Interpreter','tex');
            elseif params.PID_controller_flag
                text(obj.App.StepReference, max_index, max_value, sprintf('\\K_{p} = %.1f \\K_{i} = %.1f \n \\K_{d} = %.1f', params.Kp, params.Ki, params.Kd), 'Interpreter','tex');
            end
        end

        function updateStepDisturbance(obj, time, Plot_Value, disturbance_rejection)
            try
                % Configure plot
                hold(obj.App.StepDisturbance,'on');
                obj.App.StepDisturbance.XLim = [0 length(time)];
                obj.App.StepDisturbance.XTick = 0:50:length(time);
                obj.App.StepDisturbance.XTickLabel = 0:(0.02*50):max(time);
                obj.App.StepDisturbance.YGrid = 'on';
                
                % Create time vector and plot response
                time_vector = linspace(1,length(time), length(Plot_Value));
                line(obj.App.StepDisturbance, time_vector', Plot_Value);
                
                % Plot input signal if enabled
                if obj.App.ControllerParams.ShowInputSignal_flag
                    line(obj.App.StepDisturbance, time_vector', disturbance_rejection, 'LineStyle','--');
                end
                
                % Add controller parameters text if enabled
                if obj.App.ControllerParams.Compare_changes_flag
                    [max_value, max_index] = max(Plot_Value);
                    obj.addControllerText(max_index, max_value);
                end
                
                hold(obj.App.StepDisturbance,'off');
            catch ME
                report = getReport(ME);
                uialert(obj.App.CLASSROOMEXO_Figure, report, "Error", "Icon", "error");
            end
        end

        function updateServoTrajectory(obj, time, Plot_Value, setpoint)
            try
                % Configure plot
                hold(obj.App.ServoTrajectory,'on');
                obj.App.ServoTrajectory.XLim = [0 length(time)];
                obj.App.ServoTrajectory.YLim = [0 obj.Danger_degree+10];
                obj.App.ServoTrajectory.XTick = 0:50:length(time);
                obj.App.ServoTrajectory.YTick = 0:10:(obj.Danger_degree+10);
                obj.App.ServoTrajectory.XTickLabel = 0:(0.02*50):max(time);
                obj.App.ServoTrajectory.YTickLabel = 0:10:(obj.Danger_degree+10);
                obj.App.ServoTrajectory.YGrid = 'on';

                %%Ploting the Red zone
                X_End_point = obj.App.ServoTrajectory.XTick(end);
                Y_End_point = obj.App.ServoTrajectory.YTick(end);
                X_values = 1:X_End_point;
                Danger_degree_threshold = obj.Danger_degree.* ones(1,X_End_point);
                Max_Value = Y_End_point.* ones(1,X_End_point);
                Ref_point_line = Plot_Value(end).* ones(1,X_End_point);

                yline(obj.App.ServoTrajectory, setpoint, 'LineStyle', ':', 'Color', 'black');
                yline(obj.App.ServoTrajectory, Danger_degree_threshold, 'LineWidth', 2, 'Color', 'red');
                   
                % check if any element in the Plot vector exceeds the danger degree
                % Fill the Danger area red if true
                if any(Plot_Value >= obj.Danger_degree)
                    obj.App.Danger = 1;    
                    %%Red zone fill
                    x = 0 : X_End_point;
                    curve1 = obj.Danger_degree.* ones(1,length(x));
                    curve2 = Y_End_point.* ones(1,length(x));

                    x2 = [x, fliplr(x)];
                    inBetween = [curve1, fliplr(curve2)];
                    fill(obj.App.ServoTrajectory,x2, inBetween, 'r','FaceAlpha',0.5, 'EdgeColor','none'); 
                end

                hold(obj.App.ServoTrajectory,'off');
                
                
            catch ME
                report = getReport(ME);
                uialert(obj.App.CLASSROOMEXO_Figure, report, "Error", "Icon", "error");
            end
        end
        
        function clearStepResponse(obj)
            cla(obj.App.StepReference);
            cla(obj.App.StepDisturbance);
            cla(obj.App.ServoTrajectory);
        end
    end
end