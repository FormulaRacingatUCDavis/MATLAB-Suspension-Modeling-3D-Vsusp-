classdef Frame
    properties
        F = Half()
        R = Half()
        m = 0
        pFront = 0
        hCG = 0
        WB = 0
        
        pitch = 0
    end
    methods
        function obj = Frame(carParams)
            obj.F = Half(carParams.outboardF,carParams.inboardF,carParams.wheelF,carParams.springF,carParams.preloadF);
            obj.R = Half(carParams.outboardR,carParams.inboardR,carParams.wheelR,carParams.springR,carParams.preloadR);
            obj.m = carParams.m;
            obj.pFront = carParams.pFront;
            obj.hCG = carParams.hCG;
            obj.WB = obj.F.L.contactPt(1)-obj.R.L.contactPt(1)/10000;
            obj = obj.align();
        end
        function framePlot(obj)
            hold on
            obj.F.krcPlot()
            obj.R.krcPlot()
            plot3([obj.F.krc(1),obj.R.krc(1)],[obj.F.krc(2),obj.R.krc(2)],[obj.F.krc(3),obj.R.krc(3)], '--', 'LineWidth', 3, 'Color', 'y')

            plot3([obj.R.L.inboard(1,1), obj.F.L.inboard(2,1)], [obj.R.L.inboard(1,2), obj.F.L.inboard(2,2)], [obj.R.L.inboard(1,3), obj.F.L.inboard(2,3)], 'Color', 'k', 'LineWidth', 4)
            plot3([obj.R.R.inboard(1,1), obj.F.R.inboard(2,1)], [obj.R.R.inboard(1,2), obj.F.R.inboard(2,2)], [obj.R.R.inboard(1,3), obj.F.R.inboard(2,3)], 'Color', 'k', 'LineWidth', 4)

            plot3([obj.R.L.inboard(3,1), obj.F.L.inboard(4,1)], [obj.R.L.inboard(3,2), obj.F.L.inboard(4,2)], [obj.R.L.inboard(3,3), obj.F.L.inboard(4,3)], 'Color', 'k', 'LineWidth', 4)
            plot3([obj.R.R.inboard(3,1), obj.F.R.inboard(4,1)], [obj.R.R.inboard(3,2), obj.F.R.inboard(4,2)], [obj.R.R.inboard(3,3), obj.F.R.inboard(4,3)], 'Color', 'k', 'LineWidth', 4)
            hold off

            xlim([-1500,1500])
            ylim([-2000,2000])
            zlim([-300,700])
            view(0,0)
            
            daspect([1 1 1])
            drawnow
        end
        function obj = pitchFrame(obj,pitch)
            obj.F = obj.F.pitchHalf(obj.F.R.contactPt,obj.F.L.contactPt,pitch,'ZXY');
            obj.R = obj.R.pitchHalf(obj.F.R.contactPt,obj.F.L.contactPt,pitch,'ZXY');
        end
        function obj = solveLongLT(obj,dFz,a)
            FzF = obj.m*obj.pFront*9.8-dFz;
            FzR = obj.m*(1-obj.pFront)*9.8+dFz;
            obj.F = obj.F.solve(a*FzF/2,0,FzF/2);
            obj.R = obj.R.solve(a*FzR/2,0,FzR/2);
        end
        function obj = solveLatLT(obj)
        end
        function p = projectPoint(~,p,p0,A)
            v = p-p0;
            n = A(1,1:3);
            v = v-dot(v,n)*n/(norm(n))^2;
            p = p0+v;
        end
        function obj = align(obj,staticToeF,staticCamberF,staticToeR,staticCamberR)
            if nargin == 1
                staticToeF = obj.F.L.params(1);
                staticCamberF = obj.F.L.params(2);
                staticToeR = obj.R.L.params(1);
                staticCamberR = obj.R.L.params(2);
            end
            for i = 1:3
                obj.F = obj.F.solve(0,0,obj.m*obj.pFront*9.8*0.5);
                obj.R = obj.R.solve(0,0,obj.m*(1-obj.pFront)*9.8*0.5);
                obj.F = obj.F.align(staticToeF,staticCamberF);
                obj.R = obj.R.align(staticToeR,staticCamberR);
                obj = obj.level();
            end
        end
        function obj = level(obj)
            while norm(obj.F.L.contactPt(3) - obj.R.L.contactPt(3)) > 2^-32
                theta = -atand((obj.F.L.contactPt(3) - obj.R.L.contactPt(3))/...
                norm(obj.F.L.contactPt(1:2) - obj.R.L.contactPt(1:2)));
                obj = obj.pitchFrame(theta);
                obj.pitch = obj.pitch+theta;
            end
            obj.R = obj.R.ground(obj.F.L.contactPt);
            obj.F = obj.F.ground(obj.F.L.contactPt);
        end
    end
end

% function findAngle(p1, p2, p3, dz)
%     v1 = p1-p2
% 
%     % v1 dot v2 = 0
%     % (p1 - p2)
% end

