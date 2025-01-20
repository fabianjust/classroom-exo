% IMUPlotManager Class
classdef IMUPlotManager < handle
    properties
        Plot
        DataAcquisitionTimer
        IsPlotting
        CommunicationFunctions
    end
    
    methods
        function obj = IMUPlotManager(ax, commFunctions)
            obj.Plot = IMUAnimation(ax);
            obj.CommunicationFunctions = commFunctions;
            obj.DataAcquisitionTimer = timer('ExecutionMode', 'fixedRate', ...
                                             'Period', 1/20, ... % 20 Hz
                                             'TimerFcn', @(~,~)obj.acquireAndUpdateData());
        end
        
        function start(obj)
            obj.IsPlotting = true;
            obj.CommunicationFunctions.startAcquisition(0x44);
            start(obj.DataAcquisitionTimer);
        end
        
        function stop(obj)
            obj.IsPlotting = false;
            obj.CommunicationFunctions.stopAcquisition();
            stop(obj.DataAcquisitionTimer);
        end
        
        function acquireAndUpdateData(obj)
            if obj.IsPlotting
                [newdata, angle_deg] = obj.CommunicationFunctions.getIMU_raw(2);
                obj.Plot.update(newdata, angle_deg);
            end
        end
    end
end