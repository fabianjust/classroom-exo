classdef SensorVisualization < handle
    properties
        Axes
        WindowSize
        RawLine
        FilteredLine
        RawLine2
        FilteredLine2
        ThresholdLines
        Thresholds
        ShowThresholds
        FilterType % 'centered', 'leftsided', 'ewma'
        PlotConfig % Struct for plot-specific configurations
        DataFilter
        ShowSecondSensor = false     
        DefaultPlotSettings % Store default plot settings
    end

    properties (Constant)
        % Define filter modes as a constant property
        FilterModes = struct(...
            'FastResponse', 5,    ... % 250ms window
            'Balanced', 20,       ... % 1s window
            'Smooth', 40,         ... % 2s window
            'CurrentMode', 'Balanced'  ... % Default mode
        )
    end

    methods (Access = public)
        function obj = SensorVisualization(ax, sensorType, varargin)
            obj.Axes = ax;
            obj.ShowThresholds = false;
            obj.WindowSize = obj.FilterModes.Balanced;  % Default to balanced mode
            obj.FilterType = 'leftsided';
            obj.ThresholdLines = [];
            obj.Thresholds = [];
            
            % Parse optional parameters
            p = inputParser;
            validModes = {'FastResponse', 'Balanced', 'Smooth'};
            checkMode = @(x) any(strcmp(x, validModes));
            
            addParameter(p, 'FilterMode', 'Balanced', checkMode);  % Use FilterMode instead of WindowSize
            addParameter(p, 'FilterType', 'centered');  % Default filter type
            addParameter(p, 'YLabel', '');
            addParameter(p, 'Title', '');
            addParameter(p, 'MaxPoints', 400);
            parse(p, varargin{:});

            % Initialize the data filter
            % Set window size based on mode
            obj.WindowSize = obj.FilterModes.(p.Results.FilterMode);
            obj.FilterType = p.Results.FilterType;
            
            % Initialize filter with selected window size
            obj.DataFilter = SensorFilter(obj.WindowSize);
            
            % Set configuration based on sensor type
            obj.configurePlot(sensorType, p.Results);
            obj.initializePlot();
        end

        function setFilterMode(obj,type, mode)
            % Dynamically change filter window size
            if isfield(obj.FilterModes, mode)
                obj.WindowSize = obj.FilterModes.(mode);
                obj.DataFilter = SensorFilter(obj.WindowSize);
                obj.FilterType = type;
            else
                error('Invalid filter mode. Use FastResponse, Balanced, or Smooth');
            end
        end

        function configurePlot(obj, sensorType, config)
            % Default configurations for different sensor types
            switch lower(sensorType)
                case 'force'
                    obj.PlotConfig = struct(...
                        'YLabel', 'Force (mN)', ...
                        'Title', 'Force Reading', ...
                        'RawLineStyle', '--', ...
                        'RawColor', 'red', ...
                        'FilteredColor', 'black', ...
                        'ThresholdColor', 'blue', ...
                        'MaxPoints', 400, ...
                        'YLimits', [-1000 1000], ...
                        'ConversionFactor', 17/4); % mN conversion
                    
                case 'emg'
                    obj.PlotConfig = struct(...
                        'YLabel', 'EMG (mV)', ...
                        'Title', 'EMG Signal', ...
                        'RawLineStyle', '-', ...
                        'RawColor', 'red', ...
                        'FilteredColor', 'black', ...
                        'RawColor2', 'red', ...
                        'FilteredColor2', 'black', ...
                        'ThresholdColor', 'blue', ...
                        'MaxPoints', 400, ...
                        'YLimits', [0 400], ...
                        'ConversionFactor', 1/4);
                otherwise
                    error('Unsupported sensor type: %s', sensorType);
            end

            % Override defaults with provided configurations
            if ~isempty(config.YLabel)
                obj.PlotConfig.YLabel = config.YLabel;
            end
            if ~isempty(config.Title)
                obj.PlotConfig.Title = config.Title;
            end
            % obj.WindowSize = config.WindowSize;
            % obj.FilterType = config.FilterType;
            obj.PlotConfig.MaxPoints = config.MaxPoints;
        end

        function initializePlot(obj)
            cla(obj.Axes, "reset");
            
            % Setup axes
            obj.Axes.XLim = [0 400];
            if isfield(obj.PlotConfig, 'YLimits')
                obj.Axes.YLim = obj.PlotConfig.YLimits;
            end
            % Calculate appropriate time step for x-ticks
            tickStep = 10;
            tickInterval = 1*tickStep;
            startTick = 0;
            endTick = 400;
            ticks = ceil(startTick / tickInterval) * tickInterval : tickInterval : endTick;
            
            obj.Axes.XTick = ticks;
            obj.Axes.XTickLabel = arrayfun(@(t) sprintf('%.1f', t), ticks./tickStep, 'UniformOutput', false);
            
            obj.Axes.NextPlot = 'add';
            obj.Axes.YLabel.String = obj.PlotConfig.YLabel;
            obj.Axes.Title.String = obj.PlotConfig.Title;
            obj.Axes.YGrid = 'on';
            
            % Create animated lines
            obj.RawLine = animatedline(obj.Axes, ...
                'Color', obj.PlotConfig.RawColor, ...
                'LineStyle', obj.PlotConfig.RawLineStyle, ...
                'MaximumNumPoints', obj.PlotConfig.MaxPoints);
            
            obj.FilteredLine = animatedline(obj.Axes, ...
                'Color', obj.PlotConfig.FilteredColor, ...
                'MaximumNumPoints', obj.PlotConfig.MaxPoints);
            
            % Initialize legend
            if obj.ShowThresholds
                obj.updateThresholdLines();
            end
            obj.updateLegend();
            
            hold(obj.Axes, "off");
        end

        function setSecondSensorVisible(obj, visible)
            obj.ShowSecondSensor = visible;
            
            if obj.ShowSecondSensor && (isempty(obj.RawLine2) || ~isvalid(obj.RawLine2))
                % Create second sensor lines if they don't exist
                obj.RawLine2 = animatedline(obj.Axes, ...
                    'Color', obj.PlotConfig.RawColor2, ...
                    'LineStyle', obj.PlotConfig.RawLineStyle, ...
                    'MaximumNumPoints', obj.PlotConfig.MaxPoints);
                
                obj.FilteredLine2 = animatedline(obj.Axes, ...
                    'Color', obj.PlotConfig.FilteredColor2, ...
                    'MaximumNumPoints', obj.PlotConfig.MaxPoints);
            elseif ~obj.ShowSecondSensor && ~isempty(obj.RawLine2)
                % Remove second sensor lines
                if isvalid(obj.RawLine2)
                    clearpoints(obj.RawLine2);
                    delete(obj.RawLine2);
                end
                if isvalid(obj.FilteredLine2)
                    clearpoints(obj.FilteredLine2);
                    delete(obj.FilteredLine2);
                end
                obj.RawLine2 = [];
                obj.FilteredLine2 = [];
            end

            % Update legend
            if obj.ShowSecondSensor
                legend(obj.Axes, [obj.RawLine, obj.FilteredLine, obj.RawLine2, obj.FilteredLine2], ...
                    {'Raw', 'Filtered', 'Raw 2', 'Filtered 2'}, 'Location', 'northeast');
            else
                legend(obj.Axes, [obj.RawLine, obj.FilteredLine], ...
                    {'Raw', 'Filtered'}, 'Location', 'northeast');
            end
        end

        function setThresholds(obj, thresholds)
            obj.Thresholds = thresholds;
            if obj.ShowThresholds
                obj.updateThresholdLines();
            end
        end

        function setShowThresholds(obj, show)
            obj.ShowThresholds = show;
            if show
                obj.updateThresholdLines();
            else
                if ~isempty(obj.ThresholdLines)
                    delete([obj.ThresholdLines{:}]);
                    obj.ThresholdLines = [];
                end
            end
            obj.updateLegend();
        end

        function filtered_data = filterData(obj, raw_data)
            % Now uses the buffer-based filter
            filtered_data = obj.DataFilter.filterData(raw_data, obj.FilterType);
        end

        function update(obj, counter, raw_data, filtered_data, raw_data2, filtered_data2)
            % Convert data using sensor-specific conversion factor
            raw_data = raw_data * obj.PlotConfig.ConversionFactor;
            filtered_data = filtered_data * obj.PlotConfig.ConversionFactor;
            
            % Update plots

            addpoints(obj.RawLine, counter, raw_data);
            addpoints(obj.FilteredLine, counter, filtered_data);

            % check if we have two emg sensor streams
            % Update second sensor only if visible and data provided
            if obj.ShowSecondSensor && nargin >= 6 && ...
               ~isempty(obj.RawLine2) && isvalid(obj.RawLine2)
                raw_data2 = raw_data2 * obj.PlotConfig.ConversionFactor;
                filtered_data2 = filtered_data2 * obj.PlotConfig.ConversionFactor;
                
                addpoints(obj.RawLine2, counter, raw_data2);
                addpoints(obj.FilteredLine2, counter, filtered_data2);
            end
            
            % Adjust x-axis limits if necessary
            if counter > obj.PlotConfig.MaxPoints
                xlim(obj.Axes, [max(0, counter - obj.PlotConfig.MaxPoints), counter+50]);
                startTick = max(0, counter - obj.PlotConfig.MaxPoints);
                endTick = counter+50;
            else
                obj.Axes.XLim = [0, 400];  % Show first 400 points
                startTick = 0;
                endTick = 400;
            end

            % Calculate appropriate time step for x-ticks
            tickStep = 10;
            tickInterval = 1*tickStep;
            
            ticks = ceil(startTick / tickInterval) * tickInterval : tickInterval : endTick;
            
            obj.Axes.XTick = ticks;
            obj.Axes.XTickLabel = arrayfun(@(t) sprintf('%.1f', t), ticks./tickStep, 'UniformOutput', false);

            drawnow limitrate;
        end
    end

    methods (Access = private)
        function updateThresholdLines(obj)
            if ~isempty(obj.ThresholdLines)
                delete([obj.ThresholdLines{:}]);
            end
            obj.ThresholdLines = cell(size(obj.Thresholds));
            for i = 1:length(obj.Thresholds)
                obj.ThresholdLines{i} = yline(obj.Axes, obj.Thresholds(i), ...
                    'Color', obj.PlotConfig.ThresholdColor);
            end
        end

        function updateLegend(obj)

            % Update legend
            if obj.ShowSecondSensor
                legendItems = [obj.RawLine, obj.FilteredLine, obj.RawLine2, obj.FilteredLine2];
                legendLabels = {'Raw', 'Filtered', 'Raw 2', 'Filtered 2'};
            else
                legendItems = [obj.RawLine, obj.FilteredLine];
                legendLabels = {'Raw', 'Filtered'};
            end
            
            if obj.ShowThresholds && ~isempty(obj.ThresholdLines)
                legendItems = [legendItems, [obj.ThresholdLines{:}]];
                for i = 1:length(obj.ThresholdLines)
                    legendLabels{end+1} = sprintf('Threshold %d', i);
                end
            end
            
            legend(obj.Axes, legendItems, legendLabels, 'Location', 'northeast');
        end
    end

end