classdef SignalSimulator
    methods (Static)
        %% Response Input Functions
        %------------------------------------------------------------------
        function [signal, t] = symmetric_ramp_input(app,min_angle, max_angle, period, num_repeats)
            T = period + (num_repeats * period);% Total time duration of motion in seconds
            dt = 0.02; % Time step
            t = 0:dt:T; % Time vector
            % Generate a repeating symmetric ramp input signal with plateaus
            signal = zeros(size(t));
            quarter_period = period / 4;
            amplitude = max_angle - min_angle;
            
            for i = 0:num_repeats-1
                start = period/2 + (i * period);
                ramp_up_end = start + period/4;
                plateau_top_end = start + period/2;
                ramp_down_end = start + 3*period/4;
                ending = start + period;
                
                % Ramp up
                segment = t(t >= start & t < ramp_up_end);
                signal(t >= start & t < ramp_up_end) = (amplitude * (segment - start)/quarter_period);
                
                % Top plateau
                signal(t >= ramp_up_end & t < plateau_top_end) = amplitude;
                
                % Ramp down
                segment = t(t >= plateau_top_end & t < ramp_down_end);
                signal(t >= plateau_top_end & t < ramp_down_end) = amplitude - (amplitude * (segment - plateau_top_end)/quarter_period);
                
                % Bottom plateau
                signal(t >= ramp_down_end & t < ending) = 0;
            end
            % signal(t>=ending & t<(ending+period/2)) = 0;
        end

      function [signal, t] = step_input(app,min_angle, max_angle, period, num_repeats)
            period = period*2;
            T = 2 + (num_repeats * period);% Total time duration of motion in seconds, add half period at start and end of the input signal
            dt = 0.02; % Time step
            t = 0:dt:T; % Time vector
            % Generate a repeating step input signal
            signal = zeros(size(t));
            amplitude = max_angle - min_angle;
            for i = 0:num_repeats-1
                start = 1 + (i * period);
                ending = start + period/2;
                signal(t >= start & t < ending) = amplitude;
            end
            signal(t>=ending & t<(ending+period/2)) = 0;
      end 


      function [angle, t] = natural_elbow_movement(app,min_angle, max_angle, period, num_repeats)
            T = period + (num_repeats * period);% Total time duration of motion in seconds
            dt = 0.02; % Time step
            t = 0:dt:T; % Time vector

            frequency = 1/period;
            % Simulate natural elbow flexion/extension
            amplitude = (max_angle - min_angle) / 2;
            offset = (max_angle - min_angle) / 2;
            angle = amplitude * cos(2 * pi * frequency * t) + offset;
            angle(t >= 0 & t < period/2) = 0;
            angle(t >= ((num_repeats*period)+period/2) & t <= ((num_repeats*period)+period)) = 0;
        end

        function [angle, t] = stroke_elbow_movement(app,min_angle, max_angle, period, num_repeats)
            T = num_repeats * period;% Total time duration of motion in seconds
            dt = 0.1; % Time step
            t = 0:dt:T; % Time vector
            frequency = 1/period;
            % Simulate stroke patient elbow movement
            natural_angle = natural_elbow_movement(app, t,min_angle, max_angle, frequency);
            
            % Add realistic tremor
            tremor = realistic_tremor(app,t);
            
            % Add jerkiness and reduced range of motion
            jerkiness = 10 * sin(2 * pi * 0.2 * t);
            reduced_rom_factor = 0.7;
            
            angle = (natural_angle - min_angle) * reduced_rom_factor + min_angle + tremor + jerkiness;
            
            % Clip to ensure angles stay within bounds
            angle = max(min_angle, min(max_angle, angle));
        end

        function tremor = realistic_tremor(app,t)
            % Generate a realistic tremor
            % Combine multiple frequencies with random phases
            f1 = 8 + randn(1); % ~8 Hz, varying between individuals
            f2 = 12 + randn(1); % ~12 Hz, varying between individuals
            phase1 = 2*pi*rand(1);
            phase2 = 2*pi*rand(1);
            
            % Amplitude modulation to simulate varying tremor intensity
            amp_mod = 1 + 0.5*sin(2*pi*0.1*t);
            
            % Combine components
            tremor = amp_mod .* (2*sin(2*pi*f1*t + phase1) + sin(2*pi*f2*t + phase2));
            
            % Add some noise
            tremor = tremor + 0.5*randn(size(t));
            
            % Normalize amplitude
            tremor = tremor / max(abs(tremor)) * 3; % Adjust the 3 to change overall tremor magnitude
        end
    end
end
