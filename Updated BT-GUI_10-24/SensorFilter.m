classdef SensorFilter < handle
    properties (Access = private)
        BufferSize
        DataBuffer
        BufferIndex
        IsBufferFull
        LastFiltered
        Alpha  % For EWMA
        Fs = 20  % 20Hz sampling rate
        IsArrayInput = false
        ArraySize = 1
    end
    
    methods
        function obj = SensorFilter(bufferSize)
            obj.BufferSize = bufferSize;
            obj.DataBuffer = zeros(1, bufferSize);
            obj.BufferIndex = 1;
            obj.IsBufferFull = false;
            obj.LastFiltered = 0;
            obj.Alpha = 0.1;  % EWMA parameter
        end

        
        
        function filtered = filterData(obj, newValue, filterType)
            % Check if input is array and initialize/resize buffer if needed
            if ~isscalar(newValue) && ~obj.IsArrayInput
                obj.IsArrayInput = true;
                obj.ArraySize = length(newValue);
                obj.DataBuffer = zeros(obj.ArraySize, obj.BufferSize);
                obj.LastFiltered = zeros(obj.ArraySize, 1);
            elseif isscalar(newValue) && obj.IsArrayInput
                % Reset to scalar mode if needed
                obj.IsArrayInput = false;
                obj.ArraySize = 1;
                obj.DataBuffer = zeros(1, obj.BufferSize);
                obj.LastFiltered = 0;
            end
            
            % Add new value(s) to buffer
            if obj.IsArrayInput
                obj.DataBuffer(:, obj.BufferIndex) = newValue;
            else
                obj.DataBuffer(obj.BufferIndex) = newValue;
            end
            
            % Update buffer status
            if obj.BufferIndex == obj.BufferSize
                obj.IsBufferFull = true;
            end
            obj.BufferIndex = mod(obj.BufferIndex, obj.BufferSize) + 1;
            
            % Apply selected filter
            switch filterType
                case 'centered'
                    if obj.IsBufferFull
                        if obj.IsArrayInput
                            filtered = mean(obj.DataBuffer, 2);
                        else
                            filtered = mean(obj.DataBuffer);
                        end
                    else
                        if obj.IsArrayInput
                            validData = obj.DataBuffer(:, 1:max(1, obj.BufferIndex-1));
                            filtered = mean(validData, 2);
                        else
                            validData = obj.DataBuffer(1:max(1, obj.BufferIndex-1));
                            filtered = mean(validData);
                        end
                    end
                    
                case 'leftsided'
                    if obj.IsBufferFull
                        currentIdx = obj.BufferIndex - 1;
                        if currentIdx == 0
                            currentIdx = obj.BufferSize;
                        end
                        indices = mod((currentIdx:-1:currentIdx-obj.BufferSize+1) - 1, obj.BufferSize) + 1;
                        
                        if obj.IsArrayInput
                            filtered = mean(obj.DataBuffer(:, indices), 2);
                        else
                            filtered = mean(obj.DataBuffer(indices));
                        end
                    else
                        if obj.BufferIndex == 1
                            indices = 1:obj.BufferSize;
                        else
                            indices = 1:(obj.BufferIndex-1);
                        end
                        
                        if obj.IsArrayInput
                            filtered = mean(obj.DataBuffer(:, indices), 2);
                        else
                            filtered = mean(obj.DataBuffer(indices));
                        end
                    end
                    
                case 'ewma'
                    alpha = 2/(obj.BufferSize + 1);
                    if obj.IsArrayInput
                        filtered = alpha * newValue + (1 - alpha) * obj.LastFiltered;
                        obj.LastFiltered = filtered;
                    else
                        filtered = alpha * newValue + (1 - alpha) * obj.LastFiltered;
                        obj.LastFiltered = filtered;
                    end
                    
                case 'bandpass'
                    if obj.IsBufferFull
                        if obj.IsArrayInput
                            % Apply bandpass to each channel
                            filtered = zeros(size(newValue));
                            for i = 1:obj.ArraySize
                                temp = bandpass(obj.DataBuffer(i,:), [0.1 8], obj.Fs);
                                filtered(i) = temp(end);
                            end
                        else
                            filtered = bandpass(obj.DataBuffer, [0.1 8], obj.Fs);
                            filtered = filtered(end);
                        end
                    else
                        filtered = newValue;
                    end
            end
        end
        
        function reset(obj)
            % Reset the filter
            if obj.IsArrayInput
                obj.DataBuffer = zeros(obj.ArraySize, obj.BufferSize);
                obj.LastFiltered = zeros(obj.ArraySize, 1);
            else
                obj.DataBuffer = zeros(1, obj.BufferSize);
                obj.LastFiltered = 0;
            end
            obj.BufferIndex = 1;
            obj.IsBufferFull = false;
        end
    end
end