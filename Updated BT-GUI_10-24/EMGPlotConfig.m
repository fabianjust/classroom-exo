% EMG Plot Configuration
classdef EMGPlotConfig
    properties
        XLabel = 'Time [s]'
        YLabel = 'ENV Amplitude [µV]'
        XLim = [0 400]
        YLim = [0 0.35]
        XTick = 0:50:400
        XTickLabel = {'0', '5', '10', '15', '20', '25', '30', '35', '40'}
        YTick = [0 0.05 0.1 0.15 0.2 0.25 0.3 0.35]
        YTickLabel = {'0', '50', '100', '150', '200', '250', '300', '350'}
        ZTick = []
        GridColor = [0.502 0.502 0.502]
        YGrid = 'on'
        FontSize = 14
        Position = [17 14 1296 585]
    end
    
    methods
        function applyTo(obj, axesHandle)
            xlabel(axesHandle, obj.XLabel);
            ylabel(axesHandle, obj.YLabel);
            set(axesHandle, 'XLim', obj.XLim);
            set(axesHandle, 'YLim', obj.YLim);
            set(axesHandle, 'XTick', obj.XTick);
            set(axesHandle, 'YTick', obj.YTick);
            set(axesHandle, 'XTickLabel', obj.XTickLabel);
            set(axesHandle, 'YTickLabel', obj.YTickLabel);
            set(axesHandle, 'ZTick', obj.ZTick);
            set(axesHandle, 'GridColor', obj.GridColor);
            set(axesHandle, 'YGrid', obj.YGrid);
            set(axesHandle, 'FontSize', obj.FontSize);
            set(axesHandle, 'Position', obj.Position);
        end
    end
end