% Force Plot Configuration
classdef ForcePlotConfig
    properties
        Title = 'Force Measurement'
        XLabel = 'Time (s)'
        YLabel = 'Force (mN)'
        XLim = [0 400]
        YLim = [-1000 1000]
        XTick = 0:50:400
        XTickLabel = {'0', '5', '10', '15', '20', '25', '30', '35', '40'}
        YTick = [-1000 0 1000]
        YTickLabel = {'-1000', '0', '1000'}
        ZTick = []
        GridColor = [0.502 0.502 0.502]
        YGrid = 'on'
        FontSize = 14
        Position = [4 226 1296 585]
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
            set(axesHandle, 'ZTick', obj.ZTick);
            set(axesHandle, 'GridColor', obj.GridColor);
            set(axesHandle, 'YGrid', obj.YGrid);
            set(axesHandle, 'FontSize', obj.FontSize);
            set(axesHandle, 'Position', obj.Position);
        end
    end
end
