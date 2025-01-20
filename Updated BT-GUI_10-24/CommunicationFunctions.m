classdef CommunicationFunctions
    % COMMUNICATIONFUNCTIONS Handles communication with Arduino-based device
    %   This class provides methods for communicating with an Arduino-based device
    %   for servo control, sensor readings, and system configuration. It implements
    %   a protocol for sending commands and receiving responses.
    %
    % Properties:
    %   CommManager - Handle to the communication manager object
    %
    % Methods:
    %   testConnection       - Test if device is responding
    %   stopConnection       - Terminate the connection
    %   emergencyStop        - Trigger emergency shutdown
    %   getBatteryVoltage    - Read battery voltage
    %   RGB_Colour           - Set LED color
    %   setServoState        - Enable/disable servo
    %   startAcquisition     - Begin data acquisition
    %   stopAcquisition      - Stop data acquisition
    %   getOneEMG_IMU_Angle  - Get single set of sensor readings
    %   getServoAngle        - Read current servo position
    %   moveServoPID         - Control servo with PID
    %
    % Example:
    %   commMgr = CommunicationManager(); % Create communication manager
    %   comm = CommunicationFunctions(commMgr);
    %   if comm.testConnection()
    %       disp('Device connected successfully');
    %   end
    %
    % See also CommunicationManager

    properties (Constant, Access = private)
        % Headers
        HEADER = 0xFF
        EMG_STREAM_HEADER = 0x9F
        FORCE_STREAM_HEADER = 0x8F
        ANGLE_STREAM_HEADER = 0x7F
        
        % Basic Commands (0x01-0x09)
        CMD_TEST_CONNECT = 0x01
        CMD_SET_LED = 0x02
        CMD_GET_BAT_VOLTAGE = 0x03
        CMD_ACTV_SERVO = 0x04
        CMD_MOVE_SERVO = 0x05
        CMD_GET_SERVO_ANGLE = 0x06
        CMD_GET_FORCE = 0x07
        CMD_GET_EMG = 0x08
        CMD_GET_ALL_SENSORS = 0x09

        % Extended Commands (0x10-0x19)
        CMD_RECV_SERVO_POS = 0x10
        CMD_SET_SPEED = 0x11
        CMD_MOVE_PID = 0x12
        CMD_ZERO_SERVO = 0x13
        CMD_STOP_CONNECT = 0x15
        CMD_SET_PACKAGE_COUNTER = 0x16
        CMD_START_PICONTROLLER = 0x17
        CMD_SET_INPUT_RESPONSE_PARAMS = 0x18
        CMD_SET_PI_CONTROLLER_PARAMS = 0x19
        
        % System Configuration (0x20-0x24)
        CMD_SET_THRESHOLDS = 0x20
        CMD_STOP_ACQUISITION = 0x21
        CMD_START_ACQUISITION = 0x22
        CMD_EMERGENCY_STOP = 0x23
        CMD_SET_ANALOG_PINS = 0x24
        
        % Response Codes (0x50-0x56)
        RESP_VERIF_CONNECTION = 0x50
        RESP_LED_SET = 0x51
        RESP_SERVO_ACTIVATED = 0x52
        RESP_SERVO_DEACTIVATED = 0x53
        RESP_PID_RECEIVED = 0x54
        RESP_SPEED_SET = 0x55
        RESP_PID_FINISHED = 0x56

        % Status Codes
        RESP_STATE_ANSWER = 0xAA
        RESP_COMMAND_OK = 0xBB
        RESP_DATA_ANSWER = 0xCC
        
        % Error Codes (0x80-0x84)
        ERR_WRONG_MSG = 0x80
        ERR_WRONG_HEADER = 0x81
        ERR_WRONG_OPCODE = 0x82
        ERR_WRONG_LENGTH = 0x83
        ERR_WRONG_CHECKSUM = 0x84
        
        % Special Commands
        CMD_SET_COSINE_PARAMS = 0x60

        % RGB Colors for LED
        color_green = 0x03;
        color_white = 0x01;
        color_orange = 0x02;
        color_blue = 0x04;
        color_yellow = 0x06;
        color_off = 0x00;
        color_more_green = 0x07
        color_red = 0x05;
    end

    properties
        CommManager
    end
    
    methods (Access = public)

        function obj = CommunicationFunctions(commManager)
            obj.CommManager = commManager;
        end
        
        %% Test connection
        %------------------------------------------------------------------
        function flag = testConnection(obj)
            flag = false;
            flush(obj.CommManager.connectedDevice, "input");
            % send command!
            obj.CommManager.sendCommand(obj.CMD_TEST_CONNECT, 0, 0);
            % receive answer
            [reply, ~, msg_type] = obj.CommManager.receiveCommand();
            if reply == 0x50
                flag = true;
            end
            return
        end

        %% Stop connection
        %------------------------------------------------------------------
        function flag = stopConnection(obj)
            flag = false;
            flush(obj.CommManager.connectedDevice, "input");
            % send command!
            obj.CommManager.sendCommand(obj.CMD_STOP_CONNECT, 0, 0);
            % receive answer
            [reply, ~,msg_type] = obj.CommManager.receiveCommand();
            if reply == 0x50
                flag = true;
            end
            return
        end

        %% Trigger Emergency Shutoff /restart of Arduino Board
        %------------------------------------------------------------------
        function emergencyStop(obj)
            flag = false;
            obj.CommManager.sendCommand(obj.CMD_EMERGENCY_STOP, 0, 0);
            [reply new_data msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
                voltage = typecast(new_data(1:2), 'uint16')
                disp(['Battery voltage: ', num2str(voltage,2) ' mV']);
            end
            
            return
        end

        %% Get battery voltage
        %------------------------------------------------------------------
        function [flag, voltage] = getBatteryVoltage(obj)
            flag = false;
            flush(obj.CommManager.connectedDevice, "input");
            pause(0.01);
            obj.CommManager.sendCommand(obj.CMD_GET_BAT_VOLTAGE, 0, 0);
            [reply, new_data, msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
                voltage = typecast(new_data(1:4), 'single');
                disp(['Battery voltage: ', num2str(voltage,2) ' mV']);
            end
            return
        end

        %% Set LED Color
        %-----------------------------------------------------------------
        function [flag, led_state] = RGB_Colour(obj, color)
            flag = false;
            flush(obj.CommManager.connectedDevice, "input");
            obj.CommManager.sendCommand(obj.CMD_SET_LED, 1, color);
            [reply data, msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
                if(data == 0x51)
                    led_state = true;
                end
            end
            return
        end

        %% Activate Servo Motor
        %------------------------------------------------------------------
        function [flag, servo_state] = setServoState(obj, set_servo_state)
            flag = false;
            flush(obj.CommManager.connectedDevice, "input");
            obj.CommManager.sendCommand(obj.CMD_ACTV_SERVO, 1, set_servo_state);
            [reply data, msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
                if(data == 0x52)
                    servo_state = true;
                elseif(data == 0x53)
                    servo_state = false;
                end
            end
            return
        end

         %% Start acquisition
        %------------------------------------------------------------------
        function flag = startAcquisition(obj, cmd)
            flag = false;
            flush(obj.CommManager.connectedDevice, "input");
            obj.CommManager.sendCommand(obj.CMD_START_ACQUISITION, 1, cmd);
            [reply, ~ ,msg_type] = obj.CommManager.receiveCommand();
            if reply == 0x22
                flag = true;
            end
            return
        end
    
        %% Stop acquisition
        %------------------------------------------------------------------
        function flag = stopAcquisition(obj)
            flag = false;
            flush(obj.CommManager.connectedDevice, "input");
            obj.CommManager.sendCommand(obj.CMD_STOP_ACQUISITION,0,0);
            [reply, ~ ,msg_type] = obj.CommManager.receiveCommand();
            if reply == 0x21
                flag = true;
            end
            return
        end

        %% Get one emg, imu and angle set 
        %------------------------------------------------------------------
        function [emg_data, imu_data, angle] = getOneEMG_IMU_Angle(obj, numsamples)
            samples_counter = 0;
            new_data = zeros(1,38);
            emg_data = single(zeros(1,1));
            imu_data = single(zeros(1,6));
            while samples_counter < numsamples
                
                    [~, new_data, ~] = obj.CommManager.receiveCommand();
               
                if ((isempty(new_data)) | (new_data == obj.RESP_COMMAND_OK) | (length(new_data)<2))
                    emg_data = single(0);
                    imu_data = single(zeros(1,6));
                    angle = uint16(1000);
                    disp('default vals');  
                else
                        emg_data = typecast(new_data(5:8), 'single');
                        imu_data(1) = typecast(new_data(13:16), 'single');
                        imu_data(2) = typecast(new_data(17:20), 'single');
                        imu_data(3) = typecast(new_data(21:24), 'single');
                        imu_data(4) = typecast(new_data(25:28), 'single');
                        imu_data(5) = typecast(new_data(29:32), 'single');
                        imu_data(6) = typecast(new_data(33:36), 'single');
                        angle = typecast(new_data(37:38), 'uint16');
                end
                samples_counter = samples_counter + 1;
            end
            return
        end

        %% Get one emg, imu and angle set 
        %------------------------------------------------------------------
        function [emg_data, imu_data, angle] = getTwoEMG_IMU_Angle(obj, numsamples)
            samples_counter = 0;
            new_data = zeros(1,38);
            emg_data = single(zeros(1,2));
            imu_data = single(zeros(1,6));
            while samples_counter < numsamples
                
                    [~, new_data, ~] = obj.CommManager.receiveCommand();
               
                if ((isempty(new_data)) | (new_data == obj.RESP_COMMAND_OK) | (length(new_data)<2))
                    emg_data = single(zeros(1,2));
                    imu_data = single(zeros(1,6));
                    angle = uint16(1000);
                    disp('default vals')
                else
                        emg_data(1) = typecast(new_data(5:8), 'single');
                        emg_data(2) = typecast(new_data(9:12), 'single');
                        imu_data(1) = typecast(new_data(13:16), 'single');
                        imu_data(2) = typecast(new_data(17:20), 'single');
                        imu_data(3) = typecast(new_data(21:24), 'single');
                        imu_data(4) = typecast(new_data(25:28), 'single');
                        imu_data(5) = typecast(new_data(29:32), 'single');
                        imu_data(6) = typecast(new_data(33:36), 'single');
                        angle = typecast(new_data(37:38), 'uint16');
                end
                samples_counter = samples_counter + 1;
            end
            return
        end

        %% Get current servo position 
        %------------------------------------------------------------------
        function [flag, servo_angle] = getServoAngle(obj)
            flag = false;
            obj.CommManager.sendCommand(obj.CMD_GET_SERVO_ANGLE, 0, 0);
            [reply data, msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
                servo_angle = typecast(data(1:2), 'uint16');
            %     Analog_degree = (180/(266-738)).*(voltage-738);
            %     servo_feedback = 180-Analog_degree;
            %     disp(['Servo: ', num2str(servo_feedback,2) ' °']);
            end
            return
        end

        %% Set PID package counter
        function [flag] = setPIDpackagecounter(obj, uint8_packageCounter)
            flag = false;
            % uint8_packageCounter = typecast(packageCounter, 'uint8'); 
            obj.CommManager.sendCommand(obj.CMD_SET_PACKAGE_COUNTER, 1, uint8_packageCounter);
            [reply data, msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
        end
        %% Move Servo alonng PID curve
        %------------------------------------------------------------------
        function [flag] = moveServoPID(obj, total_length, PID_angles)
            flag = false;
            obj.CommManager.sendCommand(obj.CMD_MOVE_PID, total_length, PID_angles);
            [reply data, msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
                %voltage = uint16(data(1)) * 256 + uint16(data(2));
                %Analog_degree = (180/(266-738)).*(voltage-738);
                %servo_feedback = 180-Analog_degree;
                %disp(['Servo: ', num2str(servo_feedback,2) ' °']);
            elseif reply == 0x84
                flag = false;
            end
            return
        end

        %% Set PID Controller Params
        %------------------------------------------------------------------
       
        function [flag] = setPIDControlParams(obj,Kp, Ki, Kd)
            flag = false;
            uint8_Kp = typecast(Kp, 'uint8');
            uint8_Ki = typecast(Ki, 'uint8');
            uint8_Kd = typecast(Kd, 'uint8');
            settings = [uint8_Kp uint8_Ki uint8_Kd];
            obj.CommManager.sendCommand(obj.CMD_SET_PI_CONTROLLER_PARAMS, 12, settings);
            [reply data msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
        end

        %% Set Input Response Params
        %------------------------------------------------------------------
       
        function [flag] = setInputResponseParams(obj, type, initialStep, finalStep, period, totalIterations, holdTime)
            flag = false;
            uint8_type = typecast(type, 'uint8');
            uint8_initialStep = typecast(initialStep, 'uint8');
            uint8_finalStep = typecast(finalStep, 'uint8');
            uint8_totalIterations = typecast(totalIterations, 'uint8');
            uint32_period = typecast(period, 'uint8');
            uint32_waitTime = typecast(uint32(1*1000), 'uint8');
            uint32_holdTime = typecast(holdTime, 'uint8');
            settings = [uint8_type uint8_initialStep uint8_finalStep uint8_totalIterations uint32_waitTime uint32_period uint32_holdTime];
            obj.CommManager.sendCommand(obj.CMD_SET_INPUT_RESPONSE_PARAMS, length(settings), settings);
            [reply data msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
        end
        

        %% Set Input Response Params
        %------------------------------------------------------------------
       
        function [flag] = setCosineResponseParams(obj, type, initialStep, finalStep, period, totalIterations, holdTime)
            flag = false;
            uint8_type = typecast(type, 'uint8');
            uint8_initialStep = typecast(initialStep, 'uint8');
            uint8_finalStep = typecast(finalStep, 'uint8');
            uint8_totalIterations = typecast(totalIterations, 'uint8');
            uint32_period = typecast(period, 'uint8');
            uint32_waitTime = typecast(uint32(1*1000), 'uint8');
            uint32_holdTime = typecast(holdTime, 'uint8');
            settings = [uint8_type uint8_initialStep uint8_finalStep uint8_totalIterations uint32_waitTime uint32_period uint32_holdTime];
            obj.CommManager.sendCommand(obj.CMD_SET_COSINE_PARAMS, length(settings), settings);
            [reply data msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
        end

        %% Start PID Controller 
        %------------------------------------------------------------------
       
        function [flag] = startPIDController(obj)
            flag = false;
            obj.CommManager.sendCommand(obj.CMD_START_PICONTROLLER, 0, 0);
            [reply data msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
        end

        %% Move Servo to zero Position
        %------------------------------------------------------------------
        function [flag, zeroPosition] = resetServoPosition(obj, new_ref_degree, speed, stepsize)
            flag = false;
            uint8_new_ref_degree = typecast(new_ref_degree, 'uint8'); 
            settings = [uint8_new_ref_degree speed stepsize];
            obj.CommManager.sendCommand(obj.CMD_ZERO_SERVO, 4, settings);
            [reply data, msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
        end
    
        %% Set Servo Speed
        %------------------------------------------------------------------
        function [flag] = setServoSpeed(obj, speed)
            flag = false;
            obj.CommManager.sendCommand(obj.CMD_SET_SPEED, 1, speed);
            [reply data, msg_type] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
        end

        %% get getServoAngleStream
        %------------------------------------------------------------------
        function [flag, servo_angle] = getServoAngleStream(obj)
            flag = false;
                    
            [reply new_data, msg_type] = obj.CommManager.receiveCommand();
            if reply == 0x7F
                flag = true;
                servo_angle = typecast(new_data(1:2), 'uint16');
            %     Analog_degree = (180/(266-738)).*(voltage-738);
            %     servo_feedback = 180-Analog_degree;
            %     disp(['Servo: ', num2str(servo_feedback,2) ' °']);
            elseif reply == 0x56
                % stopped servo stream
                flag = false;
                servo_angle = uint16(0);
            end
            return
        end

        %% get getServoAngleStreamFloat
        %------------------------------------------------------------------
        function [flag, servo_angle] = getServoAngleStreamFloat(obj)
            flag = false;
                    
            [reply new_data, msg_type] = obj.CommManager.receiveCommand();
            if reply == 0x7F
                flag = true;
                servo_angle = typecast(new_data(1:4), 'single');
            %     Analog_degree = (180/(266-738)).*(voltage-738);
            %     servo_feedback = 180-Analog_degree;
            %     disp(['Servo: ', num2str(servo_feedback,2) ' °']);
            elseif reply == 0x56
                % stopped servo stream
                flag = false;
                servo_angle = single(0);
            end
            return
        end

        %% Set Threshold Values
        %------------------------------------------------------------------
        function flag = setThresholds(obj, mode, thresh1, thresh2, gain)
            flag = false;
            % If gain is not provided, use default value
            if nargin < 7
                gain = single(1);  % default value
            end

            if ((mode == 0x41) | (mode == 0x40))
                uint16_thresh1 = typecast(thresh1, 'uint8');
                uint16_thresh2 = typecast(thresh2, 'uint8');                
                data = [mode uint16_thresh1(1) uint16_thresh1(2)  uint16_thresh2(1) uint16_thresh2(2)];
                obj.CommManager.sendCommand(obj.CMD_SET_THRESHOLDS, 5, data);
            elseif (mode == 0x42)
                uint16_thresh1 = typecast(thresh1, 'uint8'); 
                data = [mode uint16_thresh1(1) uint16_thresh1(2)];
                obj.CommManager.sendCommand(obj.CMD_SET_THRESHOLDS, 3, data);
            elseif (mode == 0x43)
                uint16_thresh1 = typecast(thresh1, 'uint8');
                uint16_thresh2 = typecast(thresh2, 'uint8');
                uint8_gain = typecast(gain, 'uint8');
                data = [mode uint16_thresh1(1) uint16_thresh1(2)  uint16_thresh2(1) uint16_thresh2(2) uint8_gain];
                obj.CommManager.sendCommand(obj.CMD_SET_THRESHOLDS, 9, data);
            end
            [reply, ~] = obj.CommManager.receiveCommand();
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
            return
        end

        %% Set Analog Pin on Arduino
        function flag = setAnalogPin(obj, mode, ChP);
            flag = false;
            if (mode == 0x40)
                uint8_ChP = typecast(ChP, 'uint8');
                data = [mode uint8_ChP];
                obj.CommManager.sendCommand(obj.CMD_SET_ANALOG_PINS, 2, data);
                [reply ~] = obj.CommManager.receiveCommand();
            end
            if reply == obj.RESP_COMMAND_OK
                flag = true;
            end
            return
        end


        %% Get Force, imu and angle set 
        %------------------------------------------------------------------
        function [force_data, imu_data, servo_angle] = getForce_IMU_Angle(obj, numsamples)
            %pause(0.2);
            % start sampling
            % startAcquisition(obj);
            % tic;
            % flush(obj.CommManager.connectedDevice);
            samples_counter = 0;
            new_data = zeros(1,38);
            imu_data = zeros(1,6);
            while samples_counter < numsamples
                
                [~, new_data, ~] = obj.CommManager.receiveCommand();
                
                if ((isempty(new_data)) | (new_data == obj.RESP_COMMAND_OK) | (length(new_data)<2)) 
                    force_data = single(10);
                    imu_data = single(zeros(1,6));
                    servo_angle = uint16(1000);
                    disp('default vals')  
                else
                    if samples_counter == 0
                        force_data = typecast(new_data(1:4), 'single');
                        imu_data(1) = typecast(new_data(13:16), 'single');
                        imu_data(2) = typecast(new_data(17:20), 'single');
                        imu_data(3) = typecast(new_data(21:24), 'single');
                        imu_data(4) = typecast(new_data(25:28), 'single');
                        imu_data(5) = typecast(new_data(29:32), 'single');
                        imu_data(6) = typecast(new_data(33:36), 'single');
                        servo_angle = typecast(new_data(37:38), 'uint16');
                    end
                end
                samples_counter = samples_counter + 1;
               
            end
            % toc;
            % stop acquisition
            % stopAcquisition(obj);
            return
        
        end

        %% Get IMU Values
        %-------------------------------------------------------------------
        function [imu_data, servo_angle] = getIMU_raw(obj, numsamples)     
            %pause(0.2);
            % start sampling
            %startAcquisition(obj);
            
            samples_counter = 0;
            new_data = zeros(1,12);
            % imu_data = zeros(1,6);
            % tic
            while samples_counter < numsamples
                [~, new_data, ~] = obj.CommManager.receiveCommand();
                if ((isempty(new_data)) | (new_data == obj.RESP_COMMAND_OK))
                    imu_data = single(340);                
                else
                    if samples_counter == 0
                        % check endianness!!
                        imu_data(1) = typecast(new_data(13:16), 'single');
                        imu_data(2) = typecast(new_data(17:20), 'single');
                        imu_data(3) = typecast(new_data(21:24), 'single');
                        imu_data(4) = typecast(new_data(25:28), 'single');
                        imu_data(5) = typecast(new_data(29:32), 'single');
                        imu_data(6) = typecast(new_data(33:36), 'single');
                        servo_angle = typecast(new_data(37:38), 'uint16');
                    end
                    
                
                
                end
                samples_counter = samples_counter+1;
            end
            % toc
            
            % stop acquisition
            %stopAcquisition(obj);
            return
        end

        %% Get Force Values
        %-------------------------------------------------------------------
        function force_data = getForce_raw(obj, numsamples)     
            %pause(0.2);
            % start sampling
            %startAcquisition(obj);
            
            samples_counter = 0;
            new_data = zeros(1,38);
            % tic;
            while samples_counter < numsamples
                [~, new_data, ~] = obj.CommManager.receiveCommand();
                if ((isempty(new_data)) | (new_data == obj.RESP_COMMAND_OK))
                    force_data = single(340);                
                else
                    force_data = typecast(new_data(1:4), 'single');
                end
                samples_counter = samples_counter +1;
            end
            % toc;
            
            % stop acquisition
            %stopAcquisition(obj);
            return
        end
        

        %% Get raw EMG envelope 
        %-------------------------------------------------------------------
        function [raw_data_CH1, raw_data_CH2] = getEMG_raw(obj, numsamples)     
            %pause(0.2);
            % start sampling
            % startAcquisition(obj);
            % tic;
            % flush(obj.CommManager.connectedDevice);
            samples_counter = 0;
            new_data = zeros(1,38);
            while samples_counter < numsamples
                [~, new_data, ~] = obj.CommManager.receiveCommand();
                if ((isempty(new_data)) | (new_data == obj.RESP_COMMAND_OK))
                    raw_data_CH1 = uint16(340);
                    raw_data_CH2 = uint16(340);
                end
                raw_data_CH1 = typecast(new_data(5:8), 'single');
                raw_data_CH2 = typecast(new_data(9:12), 'single');
                samples_counter = samples_counter + 1;
            end
            % toc;
            % stop acquisition
            % stopAcquisition(obj);
            return
        end

    end
end
