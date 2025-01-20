classdef UIInitializer < handle
    properties (Access = private)
        App
    end
    
    methods
        function obj = UIInitializer(app)
            obj.App = app;
        end
        
        function initializeAll(obj)
            % Initialize plots
            obj.initializePlots();
            
            % Initialize any other complex components
            obj.initializeSliders();
            % ... other initializations
        end
        
        % Keep plots separate
        function initializePlots(obj)
            % Initialize EMG plot
            if isfield(obj.App, 'EMG_OneCH') && isvalid(obj.App.EMG_OneCH)
                delete(obj.App.EMG_OneCH);
            end
            obj.App.EMG_OneCH = uiaxes(obj.App.ElectromyographyControlGraphs_2);
            obj.configureEMGPlot();
        end
        
        % Initialize only complex sliders
        function initializeSliders(obj)
            % Only configure properties, don't recreate
            if isvalid(obj.App.EMGUpperThresholdSlider)
                config = UIConfig.EMG_Sliders.UpperThreshold;
                obj.App.EMGUpperThresholdSlider.Limits = config.Limits;
                obj.App.EMGUpperThresholdSlider.Value = config.Value;
                % ... other properties
            end
        end
    end
end