classdef Spring
    properties
        % Required:
        k1 % Primary Spring Rate
        l1 % Primary Static / Free Length
        s1 % Primary Stroke / Available Compression
        p % Total Preload Displacement
        f % Force Through Springs

        % Optional:
        k2 = [] % Secondary Spring Rate
        l2 = [] % Secondary Static / Free Length
        s2 = [] % Secondary Stroke / Available Compression

        hasHelper % 8 Args = True - 5 Args = False
    end

    methods
        function obj = Spring(k1,l1,s1,p,f,k2,l2,s2)
            % Constructor
            obj.k1 = k1;
            obj.l1 = l1;
            obj.s1 = s1;
            obj.p = p;
            obj.f = f;

            obj.hasHelper = (nargin == 8);

            if obj.hasHelper
                obj.k2 = k2;
                obj.l2 = l2;
                obj.s2 = s2;
            end
        end

        function results = seriesSpring(obj)
            % Pull Values From Object
            k1 = obj.k1;
            l1 = obj.l1;
            s1 = obj.s1;
            p = obj.p;
            f = obj.f;
            k2 = obj.k2;
            l2 = obj.l2;
            s2 = obj.s2;

            if obj.hasHelper % Including Helper Spring
                % Parallel Constant
            
                % Series Constant (Preload)
                ks = (k1 * k2) / (k1 + k2);
            
                % Preload Force 
                fp = p * ks;
            
                % Helper Spring Compression at force f
                x1_unbound = (fp + f) / k1;
                x2_unbound = (fp + f) / k2;
            
                % Capping Spring Compression under Spring Stroke
                x1 = min(x1_unbound, s1);
                x2 = min(x2_unbound, s2);
            
                %
                if x1_unbound < s1 && x2_unbound < s2
                    kc = ks; % Both Active
                elseif x1_unbound >= s1 && x2_unbound < s2
                    kc = k2; % Primary Solid, Helper Compressing
                elseif x1_unbound < s1 && x2_unbound >= s2
                    kc = k1; % Helper Solid, Primary Compression
                else
                    kc = 0; % Both Solid
                end
            else % Not Including Helper Spring (Single Spring)
                l2 = 0;
                fp = p * k1;
                x1_unbound = (fp + f) / k1;
                x1 = min(x1_unbound, s1);
                x2 = 0;
                kc = k1;
                if x1_unbound >= s1
                    kc = 0;
                end
            end
      
            xtotal = x1 + x2;
            ltotal = (l1 + l2) - xtotal;
        
            % Packing Results
            results.kc = kc;
            results.x1 = x1;
            results.x2 = x2;
            results.ltotal = ltotal;
        
            % Print Results
            fprintf('Current Spring Constant, kc = %.4f N/mm\n', kc);
            fprintf('Primary Spring Compression, x1 = %.4f mm (max s1 = %.4f mm)\n', x1, s1);
            if obj.hasHelper
                fprintf('Secondary Spring Compression, x2 = %.4f mm (max s2 = %.4f mm)\n', x2, s2);
            else
                fprintf('Secondary Spring: none (single-spring mode)\n');
            end
            fprintf('Total Spring Length, ltotal = %.4f mm\n', ltotal);
        
        end
    end
end
