classdef SpringEX
    properties
        % I assigned initial values of zero. If no input is given, this
        % doesn't change and whether the spring is single or series is
        % determined by the k2 value being zero

        % Required:
        k = 0 % Total spring constant
        x = 0 % Total spring compression

        k1 = 0 % Primary Spring Rate
        l1 = 0 % Primary Static / Free Length
        s1 = 0 % Primary Stroke / Available Compression
        p = 0 % Total Preload Displacement

        % Optional:
        k2 = 0 % Secondary Spring Rate
        l2 = 0 % Secondary Static / Free Length
        s2 = 0 % Secondary Stroke / Available Compression
    end

    methods
        function obj = SpringEX(p,k1,l1,s1,k2,l2,s2)
            % Constructor
            obj.p = p;
            obj.k1 = k1;
            obj.l1 = l1;
            obj.s1 = s1;

            if nargin == 7
                obj.k2 = k2;
                obj.l2 = l2;
                obj.s2 = s2;
            end
        end
        function obj = solve(obj, F)
                if obj.k2 == 0                                             
                    obj.k = obj.k1;
                else
                    obj.k = (obj.k1*obj.k2)/(obj.k1+obj.k2);
                end
                % Preload Force 
                Fp = -obj.p*obj.k
                
                obj.x = obj.displace(Fp+F,obj.k1,obj.s1) + Fp/obj.k1;                   % I should have specified, the change in length we're looking for excludes preload, hence the pre
                                                                                    % The point of preload here is purposely compressing the spring before any damper motion happens
                if obj.k2 ~= 0                                                      % to ensure the helper is fully compressed at static, and the compression we are measuring is
                    obj.x = obj.x + obj.displace(Fp+F,obj.k2,obj.s2) + Fp/obj.k2;       % effectively shock travel.
                    if obj.displace(Fp+F,obj.k2,obj.s2) == obj.s2
                        obj.k = obj.k1;
                    end
                end     
        end
    end
    methods (Static)
        function x = displace(F,k,s)
            x = -F/k;
            if x > s
               x = s ;
            end
        end
    end
end
