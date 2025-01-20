function CheckBattery(obj, mode)
% CHECKBATTERY Monitors battery voltage and updates UI indicators
% This function reads the battery voltage, calculates the battery percentage,
% and updates the UI with appropriate battery icons and status indicators.
% The function also provides visual warnings when battery levels are low.
%
% Parameters:
%   obj  - The app object containing UI components
%   mode - Operating mode (2 for connected state updates)
%
% Battery voltage ranges:
%   >= 7600mV : Full battery    (green)
%   >= 7200mV : High battery    (yellow-green)
%   >= 6800mV : Medium battery  (orange)
%   >= 6200mV : Low battery     (red)
%   <  6200mV : Empty battery   (error alert)
    obj.pathtest = fileparts(mfilename('fullpath'));
    HIGH_BAT_VOLT = 7700;
    LOW_BAT_VOLT = 6400;
    
    % Get the Battery Voltage measurement
    [obj.batteryFlag, obj.battery_voltage] = obj.CommFunctions.getBatteryVoltage();
    
    % Battery percentage trend line
    obj.battery_percentage = ((obj.battery_voltage-LOW_BAT_VOLT)/(HIGH_BAT_VOLT-LOW_BAT_VOLT))*100;  %% Battery_Percentage = (Battery_Voltage-6.4)*(Percentagem_max-Percentagem_min)/(8.4-6.4) + Percentagem_min;
    obj.BatteryPercentage.Text = [num2str(obj.battery_percentage,2), '%'];
    
    if obj.battery_voltage >=7400
        obj.BatteryImage.ImageSource = fullfile(obj.pathtest, 'photos','fa-battery-4.png');
        if mode == 2
            obj.StatusLamp.Color = [0,255,0]./255; %green
            obj.Label.Text = 'connected';
        end
        obj.charge_status=1;
    elseif obj.battery_voltage>=7200 && obj.battery_voltage<7400
        obj.BatteryImage.ImageSource = fullfile(obj.pathtest, 'photos','fa-battery-3.png');
        if mode == 2
            obj.StatusLamp.Color = [255,255,0]./255; %green-yellowish
            obj.Label.Text = 'connected';
        end
        obj.charge_status=1;
    elseif obj.battery_voltage>=6800 && obj.battery_voltage<7200
        obj.BatteryImage.ImageSource = fullfile(obj.pathtest, 'photos','fa-battery-2.png');
        if mode == 2
            obj.StatusLamp.Color = [255,130,0]./255; %orange
            obj.Label.Text = 'connected';
        end
        obj.charge_status=1;
    elseif obj.battery_voltage>=6400 && obj.battery_voltage<6800
        obj.BatteryImage.ImageSource = fullfile(obj.pathtest, 'photos','fa-battery-1.png');
        if mode == 2
            obj.StatusLamp.Color = [255,47,0]./255; %red
            obj.Label.Text = 'connected';
        end
        obj.charge_status=1;
    else
        obj.BatteryImage.ImageSource = fullfile(obj.pathtest, 'photos','fa-battery-0.png');
        if mode == 2
            obj.StatusLamp.Color = [255,0,0]./255;
            obj.Label.Text = 'connected';
        end
        obj.charge_status=0;
        obj.battery_voltage = 6200;
        report = ["Opps, seems like the battery is empty."; "Please recharge using the provided charger!"];
        uialert(obj.CLASSROOMEXO_Figure, report, "BATTERY EMPTY", "ICON", "error")
    end
end