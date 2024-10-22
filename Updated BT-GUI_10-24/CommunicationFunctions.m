classdef CommunicationFunctions
    methods (Static)

        %% Test connection
        %------------------------------------------------------------------
        function flag = testConnection(app)
            flag = false;
            flush(app.connected_device, "input");
            % send command!
            app.CommManager.sendCommand(0x01, 0, 0);
            % receive answer
            [reply, ~, msg_type] = app.CommManager.receiveCommand();
            if reply == 0x50
                flag = true;
            end
            return
        end

        %% Stop connection
        %------------------------------------------------------------------
        function flag = stopConnection(app)
            flag = false;
            flush(app.connected_device, "input");
            % send command!
            app.CommManager.sendCommand(0x15, 0, 0);
            % receive answer
            [reply, ~,msg_type] = app.CommManager.receiveCommand();
            if reply == 0x50
                flag = true;
            end
            return
        end

        %% Trigger Emergency Shutoff /restart of Arduino Board
        %------------------------------------------------------------------
        function emergencyStop(app)
            flag = false;
            app.CommManager.sendCommand(0x23, 0, 0);
            [reply new_data msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
                flag = true;
                voltage = typecast(new_data(1:2), 'uint16')
                disp(['Battery voltage: ', num2str(voltage,2) ' mV']);
            end
            
            return
        end

        %% Get battery voltage
        %------------------------------------------------------------------
        function [flag, voltage] = getBatteryVoltage(app)
            flag = false;
            flush(app.connected_device, "input");
            pause(0.01);
            app.CommManager.sendCommand(0x03, 0, 0);
            [reply, new_data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
                flag = true;
                voltage = typecast(new_data(1:4), 'single');
                disp(['Battery voltage: ', num2str(voltage,2) ' mV']);
            end
            return
        end

        %% Set LED Color
        %-----------------------------------------------------------------
        function [flag, led_state] = RGB_Colour(app, color)
            flag = false;
            flush(app.connected_device, "input");
            app.CommManager.sendCommand(0x02, 1, color);
            [reply data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
                flag = true;
                if(data == 0x51)
                    led_state = true;
                end
            end
            return
        end

        %% Activate Servo Motor
        %------------------------------------------------------------------
        function [flag, servo_state] = setServoState(app, set_servo_state)
            flag = false;
            flush(app.connected_device, "input");
            app.CommManager.sendCommand(0x04, 1, set_servo_state);
            [reply data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
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
        function flag = startAcquisition(app, cmd)
            flag = false;
            flush(app.connected_device, "input");
            app.CommManager.sendCommand(0x22, 1, cmd);
            [reply, ~ ,msg_type] = app.CommManager.receiveCommand();
            if reply == 0x22
                flag = true;
            end
            return
        end
    
        %% Stop acquisition
        %------------------------------------------------------------------
        function flag = stopAcquisition(app)
            flag = false;
            app.CommManager.sendCommand(0x21,0,0);
            [reply, ~ ,msg_type] = app.CommManager.receiveCommand();
            if reply == 0x21
                flag = true;
            end
            return
        end

        %% Get one emg, imu and angle set 
        %------------------------------------------------------------------
        function [emg_data, imu_data, angle] = getOneEMG_IMU_Angle(app, numsamples)
            samples_counter = 0;
            new_data = zeros(1,38);
            emg_data = single(zeros(1,1));
            imu_data = single(zeros(1,6));
            while samples_counter < numsamples
                if app.EMGisPlotting
                    [~, new_data, ~] = app.CommManager.receiveCommand();
                end
                if ((isempty(new_data)) | (new_data == 0xBB) | (length(new_data)<2)) | ~app.EMGisPlotting
                    emg_data = uint16(10);
                    imu_data = single(zeros(1,6));
                    angle = uint16(1000);
                    disp('default vals')
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

        %% Get current servo position 
        %------------------------------------------------------------------
        function [flag, servo_angle] = getServoAngle(app)
            flag = false;
            app.CommManager.sendCommand(0x06, 0, 0);
            [reply data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
                flag = true;
                servo_angle = typecast(data(1:2), 'uint16');
            %     Analog_degree = (180/(266-738)).*(voltage-738);
            %     servo_feedback = 180-Analog_degree;
            %     disp(['Servo: ', num2str(servo_feedback,2) ' °']);
            end
            return
        end

        %% Set PID package counter
        function [flag] = setPIDpackagecounter(app, uint8_packageCounter)
            flag = false;
            % uint8_packageCounter = typecast(packageCounter, 'uint8'); 
            app.CommManager.sendCommand(0x16, 1, uint8_packageCounter);
            [reply data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
                flag = true;
            end
        end
        %% Move Servo alonng PID curve
        %------------------------------------------------------------------
        function [flag] = moveServoPID(app, total_length, PID_angles)
            flag = false;
            app.CommManager.sendCommand(0x12, total_length, PID_angles);
            [reply data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
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

        %% Move Servo to zero Position
        %------------------------------------------------------------------
        function [flag, zeroPosition] = resetServoPosition(app, new_ref_degree, speed, stepsize)
            flag = false;
            uint8_new_ref_degree = typecast(new_ref_degree, 'uint8'); 
            settings = [uint8_new_ref_degree speed stepsize];
            app.CommManager.sendCommand(0x13, 4, settings);
            [reply data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
                flag = true;
            end
        end
    
        %% Set Servo Speed
        %------------------------------------------------------------------
        function [flag] = setServoSpeed(app, speed)
            flag = false;
            app.CommManager.sendCommand(0x11, 1, speed);
            [reply data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0xBB
                flag = true;
            end
        end

        %% get getServoAngleStream
        %------------------------------------------------------------------
        function [flag, servo_angle] = getServoAngleStream(app)
            flag = false;
                    
            [reply new_data, msg_type] = app.CommManager.receiveCommand();
            if reply == 0x7F
                flag = true;
                servo_angle = typecast(new_data(1:2), 'uint16');
            %     Analog_degree = (180/(266-738)).*(voltage-738);
            %     servo_feedback = 180-Analog_degree;
            %     disp(['Servo: ', num2str(servo_feedback,2) ' °']);
            elseif reply ~= 0x7F
                [reply new_data, msg_type] = app.CommManager.receiveCommand();
                servo_angle = typecast(new_data(1:2), 'uint16');
            end
            return
        end

        %% Set Threshold Values
        %------------------------------------------------------------------
        function flag = setThresholds(app, mode, thresh1, thresh2, speed, stepSize)
            flag = false;
            if ((mode == 0x41) | (mode == 0x40))
                uint16_thresh1 = typecast(thresh1, 'uint8');
                uint16_thresh2 = typecast(thresh2, 'uint8');
                uint16_stepStize = typecast(stepSize, 'uint8');
                uint8_speed = typecast(speed, 'uint8');
                data = [mode uint16_thresh1(1) uint16_thresh1(2)  uint16_thresh2(1) uint16_thresh2(2) speed stepSize];
                app.CommManager.sendCommand(0x20, 7, data);
            elseif (mode == 0x42)
                uint16_thresh1 = typecast(thresh1, 'uint8');
                uint16_stepStize = typecast(stepSize, 'uint8');
                uint8_speed = typecast(speed, 'uint8');
                data = [mode uint16_thresh1(1) uint16_thresh1(2) speed stepSize];
                app.CommManager.sendCommand(0x20, 5, data);
            end
            [reply, ~] = app.CommManager.receiveCommand();
            if reply == 0xBB
                flag = true;
            end
            return
        end

        %% Set Analog Pin on Arduino
        function flag = setAnalogPin(app, mode, ChP);
            flag = false;
            if (mode == 0x40)
                uint8_ChP = typecast(ChP, 'uint8');
                data = [mode uint8_ChP];
                app.CommManager.sendCommand(0x24, 2, data);
                [reply ~] = app.CommManager.receiveCommand();
            end
            if reply == 0xBB
                flag = true;
            end
            return
        end


        %% Get Force, imu and angle set 
        %------------------------------------------------------------------
        function [force_data, imu_data, servo_angle] = getForce_IMU_Angle(app, numsamples)
            %pause(0.2);
            % start sampling
            % startAcquisition(app);
            % tic;
            % flush(app.connected_device);
            samples_counter = 0;
            new_data = zeros(1,38);
            imu_data = zeros(1,6);
            while samples_counter < numsamples
                if app.ForceisPlotting
                [~, new_data, ~] = app.CommManager.receiveCommand();
                end
                if ((isempty(new_data)) | (new_data == 0xBB) | (length(new_data)<2)) | ~app.ForceisPlotting
                    force_data = uint16(10);
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
            % stopAcquisition(app);
            return
        
        end

        %% Get IMU Values
        %-------------------------------------------------------------------
        function [imu_data, servo_angle] = getIMU_raw(app, numsamples)     
            %pause(0.2);
            % start sampling
            %startAcquisition(app);
            
            samples_counter = 0;
            new_data = zeros(1,12);
            % imu_data = zeros(1,6);
            % tic
            while samples_counter < numsamples
                [~, new_data, ~] = app.CommManager.receiveCommand();
                if ((isempty(new_data)) | (new_data == 0xBB))
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
            %stopAcquisition(app);
            return
        end

        %% Get Force Values
        %-------------------------------------------------------------------
        function force_data = getForce_raw(app, numsamples)     
            %pause(0.2);
            % start sampling
            %startAcquisition(app);
            
            samples_counter = 0;
            new_data = zeros(1,38);
            % tic;
            while samples_counter < numsamples
                [~, new_data, ~] = app.CommManager.receiveCommand();
                if ((isempty(new_data)) | (new_data == 0xBB))
                    force_data = single(340);                
                else
                    force_data = typecast(new_data(1:4), 'single');
                end
                samples_counter = samples_counter +1;
            end
            % toc;
            
            % stop acquisition
            %stopAcquisition(app);
            return
        end
        

        %% Get raw EMG envelope 
        %-------------------------------------------------------------------
        function [raw_data_CH1, raw_data_CH2] = getEMG_raw(app, numsamples)     
            %pause(0.2);
            % start sampling
            % startAcquisition(app);
            % tic;
            % flush(app.connected_device);
            samples_counter = 0;
            new_data = zeros(1,38);
            while samples_counter < numsamples
                [~, new_data, ~] = app.CommManager.receiveCommand();
                if ((isempty(new_data)) | (new_data == 0xBB))
                    raw_data_CH1 = uint16(340);
                    raw_data_CH2 = uint16(340);
                end
                raw_data_CH1 = typecast(new_data(5:8), 'single');
                raw_data_CH2 = typecast(new_data(9:12), 'single');
                samples_counter = samples_counter + 1;
            end
            % toc;
            % stop acquisition
            % stopAcquisition(app);
            return
        end

    end
end
