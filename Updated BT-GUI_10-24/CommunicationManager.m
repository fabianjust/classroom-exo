classdef CommunicationManager < handle
    properties (Access = public)
        connectedDevice
        config
        % defines for communication protocol
        startMarker = 0x3C;
        endMarker = 0x3E;
        header = 0xFF; %binary: 1111 1111, dec: 255
        force_header = 0x8F;
        max_data_length = 200;
        data = [];
        BAUDrate = 115200;
    end

    methods
        function obj = CommunicationManager(portName)
            if nargin < 1
                error('Port name must be specified');
            end

            % Initialize config
            obj.config = struct(...
                'max_data_length', obj.max_data_length, ...
                'header', obj.header, ...
                'startMarker', obj.startMarker, ...
                'endMarker', obj.endMarker, ...
                'baudRate', obj.BAUDrate, ...
                'timeout', 1 ...
            );

            obj.establishConnection(portName);

             
        end

        function success = establishConnection(obj, portName)
            %  Establishes serial connection with specified settings
            success = false;
            
            try
                % Clean up any existing connection first
                obj.cleanup();
                
                % Create and configure new serial connection
                obj.connectedDevice = serialport(portName, obj.config.baudRate, "Timeout",obj.config.timeout);
                warning('OFF','serialport:serialport:ReadWarning')
                
                % Flush any existing data
                flush(obj.connectedDevice);
                
                success = true;
                disp(['Successfully connected to ' portName]);
            catch ME
                error('Failed to establish connection: %s', ME.message);
            end
        end

        function cleanup(obj)
            % CLEANUP Properly closes and cleans up the serial connection
            try
                if ~isempty(obj.connectedDevice) && isvalid(obj.connectedDevice)
                    if obj.connectedDevice.Status == "open"
                        % Disable any callbacks
                        configureCallback(obj.connectedDevice, "off");
                        
                        % Flush the device
                        flush(obj.connectedDevice);
                        pause(0.1); % Small delay to ensure flush completes
                        
                        % Delete the device
                        delete(obj.connectedDevice);
                    end
                    disp('Serial connection cleaned up successfully');
                end
                obj.connectedDevice = [];
                
            catch ME
                warning(ME.identifier, 'Error during cleanup: %s', ME.message);
            end
        end

        function sendCommand(obj, opcode, dataLength, data)
            % SENDCOMMAND Sends a command/data package to Arduino via Serial Port Connection
            %
            % Inputs:
            %   obj - The serial port object
            %   opcode - The operation code
            %   dataLength - Length of the data
            %   data - The data to send
        
            if (dataLength > obj.config.max_data_length)
                error('Message too long!')
            end
        
            % Create checksum
            checksum = double(obj.config.header) + double(opcode) + double(dataLength);
        
            % Create structure of message
            message2send = struct(...
                'startMarker', uint8(obj.config.startMarker), ...
                'header', uint8(obj.config.header), ...
                'opcode', uint8(opcode), ...
                'dataLength', uint8(dataLength), ...
                'data', uint8(data), ...
                'checksum', mod(checksum, 256), ...
                'endMarker', uint8(obj.config.endMarker));
        
            % Transmit!
            flush(obj.connectedDevice);
            if message2send.dataLength > 0
                write(obj.connectedDevice, [message2send.startMarker message2send.header message2send.opcode message2send.dataLength message2send.data message2send.checksum message2send.endMarker], 'uint8');
            else
                write(obj.connectedDevice, [message2send.startMarker message2send.header message2send.opcode message2send.dataLength message2send.checksum message2send.endMarker], 'uint8');
            end
        end

        %% receive Answer/Data package from Arduino via Serial Port Connection
        function [reply, data, message_type] = receiveCommand(obj)
            % RECEIVECOMMAND Receives an Answer/Data package from Arduino via Serial Port Connection
            %
            % Input:
            %   obj - The serial port object
            %
            % Outputs:
            %   reply - The received reply
            %   data - The received data
            %   message_type - The type of the received message
            %
            % This function reads incoming data from the Arduino through the specified
            % serial port object and parses it into reply, data, and message type.
            
            % create structure of message
            message2receive = struct('header',0,'opcode',0,'dataLength',0,'data',0,'checksum',0);
            % receive!
            try
            
                temp = read(obj.connectedDevice,1, 'uint8');
                % tic
                while((temp ~= obj.config.startMarker))
                    temp = read(obj.connectedDevice,1, 'uint8');
                end
                % disp('receiving package');
                % toc
                tStart = tic;
                message2receive.header = read(obj.connectedDevice,1, 'uint8');
                tEnd = toc(tStart);
                if tEnd > 1
                    message2receive.header = 0;
                    message2receive.opcode = 0xBB;
                    message2receive.data = 0;
                    message2receive.checksum = 0;
                    message_type = 'CMD';
                    reply = message2receive.opcode;
                    data = uint8(message2receive.data);
                    disp(tEnd);
                    return
                end
                % toc
                % tic
                message2receive.opcode = read(obj.connectedDevice,1,'uint8');
                % toc
        
                switch message2receive.opcode
                    case 0x7F
                        % Servo Streaming Message
                        message2receive.dataLength = read(obj.connectedDevice,1,'uint8');
                        message_type = 'Servo';
                        reply = message2receive.opcode;
                    case 0x8F
                        % Force Streaming Message
                        message2receive.dataLength = read(obj.connectedDevice,1,'uint8');
                        message_type = 'Force';
                        reply = message2receive.opcode;
                    case 0x9F
                        % EMG Steaming Message
                        message2receive.dataLength = read(obj.connectedDevice,1,'uint8');
                        message_type = 'EMG';
                        reply = message2receive.opcode;
                    case 0x6F
                        % IMU Steaming Message
                        message2receive.dataLength = read(obj.connectedDevice,1,'uint8');
                        message_type = 'IMU';
                        reply = message2receive.opcode;
                    otherwise
                        % CMD message
                        message2receive.dataLength = read(obj.connectedDevice, 1, 'uint8');
                        message_type = 'CMD';
                        reply = message2receive.opcode;
                end
                % tic
                if message2receive.dataLength > 0
                   message2receive.data = read(obj.connectedDevice,message2receive.dataLength,'uint8');
                % for i= 1:message2receive.dataLength
                %    message2receive.data(i) = read(obj.connected_device, 1, 'uint8');
                % end
                end
                % toc
                % tic
                message2receive.checksum = read(obj.connectedDevice,1,'uint8');
                % check message for integrity
                checksum = mod(sum([message2receive.header, message2receive.opcode, typecast(uint16(message2receive.dataLength),'uint8')]), 256);
                % toc
                if message2receive.checksum == checksum
                    reply = message2receive.opcode;
                    data = uint8(message2receive.data);
                    return
                else
                    reply = -1;
                    data = [];
                    %error('wrong checksum of message!')
                
                end
                
                
            catch e
                if exist('self.connected_device','var')
                   delete(obj.connectedDevice);
                end
        
                
                 error(e.message)
                return
            end
        end
    

    end

end
