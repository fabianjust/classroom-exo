classdef ControlFunctions < handle
    properties
        % Add any instance properties needed
        LastComputedG
        LastComputedF
        LastComputedParameters
    end
    
    methods
        function obj = ControlFunctions()
            % Constructor
            obj.LastComputedG = [];
            obj.LastComputedF = [];
            obj.LastComputedParameters = struct();
        end

        function G = analyze_transfer_function(obj, tf_num, tf_den)
            % Preprocess the string
            num_str = obj.preprocess_tf_string(tf_num);
            den_str = obj.preprocess_tf_string(tf_den);
            
            % Convert to symbolic expressions
            syms s
            num = str2sym(num_str);
            den = str2sym(den_str);
            
            % Convert to MATLAB transfer function
            numerator = double(coeffs(num, s, 'all'));
            denominator = double(coeffs(den, s, 'all'));
            G = tf(numerator, denominator, 0);
            
            % Store result
            obj.LastComputedG = G;
        end

        function wc_150 = calulateCrossoverFrequency(obj, G)
            [G_mag,G_phi, G_wc] = bode(G);
            G_phi = squeeze(G_phi);
            w150=interp1(G_phi,G_wc,-150); %% ERROR!! sample points must be unique! -> anders berechnen

            % display wc to G(s) Plant Tab
            % see "coice of crossover fre" in book!
            % 0.6 for PID and 0.4 for PI
            if obj.PID_control_flag | obj.PID_design_flag
                obj.omega_G150Label.Text = '0.6\omega_{G150} = ' + string(round(w150*0.6,2));  
            elseif obj.PI_design_flag || obj.PI_control_flag 
                obj.omega_G150Label.Text = '0.4\omega_{G150} = ' + string(round(w150*0.4,2));
            end

        end

        function [F_PID, Kp, Ki, Kd] = pid_control(obj, tau, Ki ,beta ,zeta) 
            % calculate dimensioning parameters according to eq 8.33
            Tf=tau/beta;
            Ti=2*zeta*tau-Tf;
            Td=((tau^2)/Ti)-Tf;
            Kp=Ki*Ti;
            Kd=Kp*Td;

            % calculate PID controller with Kp, Ki, and Kp parameters
            F_PID = pid(Kp,Ki,Kd);

        end   
        
        function [F, Ki, Kinf, tau, err] = pid_design(obj, G, wc, phim, beta, zeta)
            % PID controller design method
            % Returns controller parameters for a PID controller
            % F=Ki*(1+2*zeta*s*tau+(s*tau)^2)/(s*(1+s*tau/beta))
            % Kinf=Ki*tau*beta
            % Jv=1/Ki
            %
            % err=1 => desired phase margin not achievable   OK R 020223
            
            err = 0;
            [Gwc, phiGwc] = bode(G, [wc]);
            if (phiGwc > 90)
                phiGwc = phiGwc - 360;
            end
            
            phiL = -180 + phim;
            phiF = phiL - phiGwc;
            [wctau, Kinf, phiF0] = obj.pid_design_beta(Gwc, phiF, beta, zeta);
            
            if (abs(phiF - phiF0) > 1e-4)
                err = 1;
                warning('Desired stability margin not achievable');
            end
            
            tau = wctau/wc;
            Ki = Kinf/tau/beta;
            F = tf([Ki*tau^2 Ki*2*zeta*tau Ki], [tau/beta 1 0]);
            
            if err == 0
                err = abs(norm(feedback(G*F, 1))) == inf;
                if err
                    warning('Unstable closed loop system');
                end
            end
            
            % Store results
            obj.LastComputedF = F;
            obj.LastComputedParameters.Ki = Ki;
            obj.LastComputedParameters.Kinf = Kinf;
            obj.LastComputedParameters.tau = tau;
        end

        function [F_PI, Kp, Ki, Kd] = pi_control(obj, Ti, Ki)
                Kp = Ki*Ti;
                Kd = 0;

                % calculate PI controller with Kp, Ki, and Kp parameters
                % F_PI=tf([Ki*Ti Ki],[1 0]); is the same as:
                F_PI = pid(Kp, Ki);
        end
        

        function [F_PI, Kp, Ki, Kd] = pi_design(obj, G, wc,phim)
                % calculate the phase at borderfrequency wc according to eq 8.7
                % ∠F(jωc) = −180◦ + ϕm − ∠G(jωc) 
                % with ∠G(jωc) = −90◦ − arctan(ωcT)
                
                [gain_G,phase_G]=bode(G,wc);
                phase_F_PI = -180+phim-phase_G; 

                Ti = cotd(-phase_F_PI)/wc; % calculate Ti according to eq. 8.11
                Ki = wc/(gain_G*sqrt(1+(Ti*wc)^2)); % calculate Ki according to eq. 8.12
                
                Kp = Ki*Ti;
                Kd = 0;
                % calculate PI controller with Kp, Ki, and Kp parameters
                % F_PI=tf([Ki*Ti Ki],[1 0]);
                F_PI = pid(Kp, Ki);
        end
        
        
        function [F, wc, phim, Ki, Ti, MS, MT] = pi_opt(obj, G, wc_min, wc_max, phim_min, phim_max)
            Nwc = 20;
            Nphim = 10;
            wc = [wc_min:(wc_max-wc_min)/(Nwc-1):wc_max];
            phim = [phim_min:(phim_max-phim_min)/(Nphim-1):phim_max];
            
            % Get PI controllers for the grid
            [F, Ki, Ti] = obj.pi_wc_phim(G, wc, phim);
            
            % Calculate sensitivity peaks
            MS = norm(feedback(1, G*F), inf, 1e-3);
            MT = norm(feedback(G*F, 1), inf, 1e-3);
            stab = norm(feedback(G*F, 1));
            
            % Find optimal controller
            [Ki, iopt] = max(Ki'.*(MS <= 1.7).*(MT <= 1.3).*(stab < inf));
            
            % Select the optimal controller and its parameters
            F = F(:,:,iopt);
            MS = MS(iopt);
            MT = MT(iopt);
            Ti = Ti(iopt);
            
            % Calculate final margins
            [Am, phim, wpi, wc] = margin(G*F);
        end

         function [F,Ki,Ti]=pi_wc_phim(obj, G,wc,phim)
            Nwc = length(wc); 
            Nphim = length(phim);
            for k=1:Nphim
                for i=1:Nwc
                    j=i+(k-1)*Nwc;
                    [gain_G,phase_G]=bode(G,wc(i));
                    phase_F=-180+phim(k)-phase_G;
                    Ti(j)=cotd(-phase_F)/wc(i);
                    Ki(j)=gain_G/sqrt(1+(Ti(j)*wc(i))^2);
                    F(:,:,j)=tf([Ki(j)*Ti(j) Ki(j)],[1 0]);
                end
            end
        end
    end    
        
    
    methods (Access = private)
        function processed_str = preprocess_tf_string(obj, tf_string)
            % Remove spaces
            tf_string = strrep(tf_string, ' ', '');
            
            % Replace ^ with .^
            tf_string = strrep(tf_string, '^', '.^');
            
            % Add multiplication symbols where needed
            tf_string = regexprep(tf_string, '(\d+)([a-zA-Z])', '$1*$2');
            tf_string = regexprep(tf_string, ')(',  ')*(' );
            tf_string = regexprep(tf_string, '([a-zA-Z0-9)])(', '$1*(');
            
            processed_str = tf_string;
        end
        
        function [wctau, Kinf, phiF] = pid_design_beta(obj, Gwc, phiF, beta, zeta)
            
        %
        %  [wctau,Kinf,phiF0]=pid_design_beta(Gwc,phiF,beta,zeta)   OK R 020223
        %
        
            wctau=logspace(-1,1,200);
            
            for iter=1:2
            
               r=sqrt((1-wctau.^2).^2+(2*zeta*wctau).^2);
               res = acos((1-wctau.^2)./r) - atan(wctau/beta) - phiF*pi/180;
               wctau=obj.inversefcn1(wctau,res,pi/2);
            
               if isempty(wctau)
                   phiF0=phiF+1;
                   Kinf=1;
                   wctau=1;    
                   return
               end
               if iter==1
                   wctau=wctau*(0.9805:0.001:1.02);
               end
            end
            
            r=sqrt((1-wctau^2)^2+(2*zeta*wctau)^2);
            Kinf=beta*wctau*sqrt(1+(wctau/beta)^2)/Gwc/r;
            
            phiF0=(acos((1-wctau^2)/r) - atan(wctau/beta))*180/pi-90;
        
        end

        function [xval,xival,zval]=inversefcn1(obj,x,y,yconst,z)  % OKR o test 020302
        
            if y(1)<=yconst
                ix=find(y>=yconst);
            else
                ix=find(y<=yconst);
            end
            N=length(x);
            
            if isempty(ix) | (N<2)
                xval=[];
                zval=[];
                xival=[];
                return
            else
                i=min(ix(1),N-1);
                xival=i;
            end
            
            if abs(y(i+1)-y(i))>=1e-12
                xval=x(i)+(x(i+1)-x(i))/(y(i+1)-y(i))*(yconst-y(i));
            else
                xval=x(i);
            end
            
            if nargin==4 & nargout==3
                zval=z(i)+(z(i+1)-z(i))/(x(i+1)-x(i))*(xval-x(i));
            end
        end
    
    end
    
    methods (Access = public)
        function results = getLastResults(obj)
            % Utility method to get last computed results
            results = struct(...
                'G', obj.LastComputedG, ...
                'F', obj.LastComputedF, ...
                'Parameters', obj.LastComputedParameters);
        end
        
        function clearResults(obj)
            % Utility method to clear stored results
            obj.LastComputedG = [];
            obj.LastComputedF = [];
            obj.LastComputedParameters = struct();
        end
    end
end