% Servo Plot Configuration
classdef ServoPlotConfig
    properties
        Title = 'Servo Trajectory'
        XLabel = 'Time [s]'
        YLabel = 'Angle [deg]'
        XLim = [0 200]
        YLim = [0 140]
        XTick = 0:20:200
        XTickLabel = {'0','20','40','60','80','100','120','140','160','180','200'}
        YTick = 0:20:140
        YTickLabel = {'0','20','40','60','80','100','120','140'}
        GridColor = [0.502 0.502 0.502]
        YGrid = 'on'
        FontSize = 14
    end
    
    methods
        function applyTo(obj, axesHandle)
            title(axesHandle, obj.Title);
            xlabel(axesHandle, obj.XLabel);
            ylabel(axesHandle, obj.YLabel);
            set(axesHandle, 'XLim', obj.XLim);
            set(axesHandle, 'YLim', obj.YLim);
            set(axesHandle, 'XTick', obj.XTick);
            set(axesHandle, 'YTick', obj.YTick);
            set(axesHandle, 'XTickLabel', obj.XTickLabel);
            set(axesHandle, 'YTickLabel', obj.YTickLabel);
            set(axesHandle, 'GridColor', obj.GridColor);
            set(axesHandle, 'YGrid', obj.YGrid);
            set(axesHandle, 'FontSize', obj.FontSize);
        end
    end
end