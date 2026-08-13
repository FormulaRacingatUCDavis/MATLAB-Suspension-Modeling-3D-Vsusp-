% Object file defining one corner of a car's suspension

classdef Corner                                                            % Class name: Corner
    properties                                                             % Properties are variables (data) stored by the class, they're initialized (generally with zeroes) to the type, number, and arrangement of variable they store     
        outboard = zeros(6,3)                                              % Outboard static hardpoints, upper ball joint (1:2,:), lower ball joint (3:4,:), tie/toe rod (5,:), and pushrod (6,:)
        inboard = zeros(9,3)                                               % Inboard static hardpoints, up fore/aft ball joint (1/2,:), low fore/aft ball joint (3/4,:), tie/toe rod (5,:) and pushrod (6,:)
                                                                           % Inboard also contains the rocker pivot point (7,:), the shock to rocker connection (8,:), and the shock end (9.:)
        wheel = zeros(3,3)                                                 % Wheel center axis. Includes outside (1,:), wheel face (2,:), and inside(3,:)
        contactPt = zeros(1,3)                                             % Tire contact point, located directly below center of wheel
        tire = zeros(51,3,4)                                               % Points that create tire surfaces on the graph
                                                                           
        links = zeros(6,3)                                                 % Each of the 6 suspension links represented in vector notation
        upright = zeros(9,3)                                               % Locations of outboard points and wheel points relative to each other
        apex = zeros(3,4)                                                  % Location of our apex relative to the control arm it's on, the extra length (:,4) stores a 0 or 1 to signal a lower or upper control arm apex
        rocker = zeros(3,3)                                                % Stores the relationship of the rocker (outboard(6:8,:)) relative to each other
        shock = zeros(1,3)                                                 % Stores the vector outboard(8,:) - outboard(9,:) which represents the shock. The length of this vector is the overall shock length
        
        params = zeros(1,5)                                                % Stores wheel parameters toe, camber, caster, mechanical trail, and scrub radius respectively
        forces = zeros(1,10)                                               % Stores forces through each link (1:6), the force through the shock (7), and the X, Y, and Z wheel loads (8:10)                     

        ic = zeros(1,3)                                                    % Stores the location of the instant center of the corner
        
        Spring = Spring()                                                  % Stores the spring of the corner
    end

    methods                                                                % Methods are functions included in the class
        function obj = Corner(outboard,inboard,wheel,spring,p)               % Initializes an instance of the class from these inputs
            if nargin == 0
                return
            end
            obj.wheel = wheel;                                             
            obj.outboard = outboard;
            obj.inboard(1:size(inboard,1),:) = inboard;                    % In case an incomplete set of coordinates (missing prs assembly) is inputted
            obj.Spring = Spring(spring,p);                                        % Convert from lb/in to N/mm
            obj = obj.evaluate();
        end

        function obj = evaluate(obj)                                       % Finds the rest of parameters not explicitly given by input
            for i = 1:6                                                    % For each arm
                obj.links(i,:) = obj.outboard(i,:)-obj.inboard(i,:);       % Finds the vector components of each link
            end                                                
            
            obj.upright(1,:) = obj.outboard(1,:)-obj.outboard(3,:);       
            obj.upright(2,:) = obj.outboard(1,:)-obj.outboard(5,:);       
            obj.upright(3,:) = obj.outboard(3,:)-obj.outboard(5,:);       

            obj.upright(4,:) = obj.wheel(1,:)-obj.outboard(1,:);
            obj.upright(5,:) = obj.wheel(1,:)-obj.outboard(3,:);
            obj.upright(6,:) = obj.wheel(1,:)-obj.outboard(5,:);
            obj.upright(7,:) = obj.wheel(3,:)-obj.outboard(1,:);
            obj.upright(8,:) = obj.wheel(3,:)-obj.outboard(3,:);
            obj.upright(9,:) = obj.wheel(3,:)-obj.outboard(5,:);

            obj.rocker(1,:) = obj.inboard(6,:)-obj.inboard(7,:);       
            obj.rocker(2,:) = obj.inboard(8,:)-obj.inboard(7,:);       
            obj.rocker(3,:) = obj.inboard(6,:)-obj.inboard(8,:);       

            obj.shock(:) = obj.inboard(9,:)-obj.inboard(8,:); 

            if norm(obj.outboard(6,:)-obj.outboard(1,:)) < ...             % If your apex is further from your upper ball joint than your lower ball joint
                    norm(obj.outboard(6,:)-obj.outboard(3,:))
                i = 1;
                obj.apex(:,4) = 1;
            else
                i = 3;
                obj.apex(:,4) = 0;
            end
            
            obj.apex(1,1:3) = obj.inboard(i,:)-obj.outboard(6,:);
            obj.apex(2,1:3) = obj.inboard(i+1,:)-obj.outboard(6,:);
            obj.apex(3,1:3) = obj.outboard(i,:)-obj.outboard(6,:);

            obj = obj.findIC();

            obj.params(1) = asind((obj.wheel(1,1)-obj.wheel(3,1))/...      % Finds toe value
                norm((obj.wheel(1,1:2)-obj.wheel(3,1:2))));        
            obj.params(2) = -asind((obj.wheel(1,3)-obj.wheel(3,3))/...     % Finds camber value
                norm((obj.wheel(1,:)-obj.wheel(3,:))));           
            obj.params(3) = asind((obj.outboard(3,1)-obj.outboard(1,1))/...% Finds caster value
                norm(obj.outboard(3,[1,3])-obj.outboard(1,[1,3])));      

            obj.contactPt = (obj.wheel(1,:)+obj.wheel(3,:))/2-[0,0,8.1*...
                25.4*abs(cosd(obj.params(2)))];

            v = obj.outboard(3,:)-obj.outboard(1,:);                       % Finds kingpin axis line equation
            t = (obj.contactPt(3)-obj.outboard(3,3))/v(3);                 % Finds parameter t for the KPI's intersection with the ground 
            x = obj.outboard(3,1)+t*v(1);                                  % Finds x coordinate of the KPI's intersection with the ground
            y = obj.outboard(3,2)+t*v(2);                                  % Finds y coordinate of the KPI's intersection with the ground
            obj.params(4) = (x-obj.contactPt(1));                          % Finds mechanical trail
            obj.params(5) = obj.inboard(1,2)/abs(obj.inboard(1,2))*...     % Finds scrub radius
                (obj.contactPt(2)-y);
            obj = obj.findTire();
        end

        function obj = findTire(obj)                                       
            t = linspace(0,2*pi,51)';                                      % Divides circle into 51 angles
            x = cos(t);                                                    % Plots a circle in the x-z plane
            y = 0*t;
            z = sin(t);
            tireR = 25.4*8;                                                % Outer radius of our 16 in diameter tire
            wheelR = 25.4*5;                                               % Outer radius of our 10 in diameter wheel
            coords = [x y z];

            quat1 = quaternion([obj.params(2)*-obj.inboard(1,2)/...        % Rotates the neutral tire to match camber and toe angles
                abs(obj.inboard(1,2)),0,0],"eulerd","XYZ","point");
            quat2 = quaternion([0,0,obj.params(1)*-obj.inboard(1,2)/...
                abs(obj.inboard(1,2))],"eulerd","XYZ","point");

            obj.tire(:,:,1) = rotatepoint(quat1, coords(:,:));
            obj.tire(:,:,1) = obj.wheel(1,:)+tireR*rotatepoint(quat2, ...
                obj.tire(:,:,1));

            obj.tire(:,:,2) = rotatepoint(quat1, coords(:,:));
            obj.tire(:,:,2) = obj.wheel(3,:)+tireR*rotatepoint(quat2, ...
                obj.tire(:,:,2));

            obj.tire(:,:,3) = rotatepoint(quat1, coords(:,:));
            obj.tire(:,:,3) = obj.wheel(1,:)+wheelR*rotatepoint(quat2, ...
                obj.tire(:,:,3));

            obj.tire(:,:,4) = rotatepoint(quat1, coords(:,:));
            obj.tire(:,:,4) = obj.wheel(3,:)+wheelR*rotatepoint(quat2, ...
                obj.tire(:,:,4));
        end

        function coords = solveSpheres(~, p1, p2, p3, l1, l2, l3, p0)      % Solves for a point in space by triangulating constant distances from three known points
            A = zeros(2,4);
            A(1,1) = (2*p2(1)-2*p1(1));                                    % The proof for this section is in my physical notes, but these equations were derived
            A(1,2) = (2*p2(2)-2*p1(2));                                    % by hand to avoid making the computer do a bunch of calculations. It cut tenths of a 
            A(1,3) = (2*p2(3)-2*p1(3));                                    % second off processing time for this function
            A(1,4) = l1^2-l2^2-p1(1)^2+p2(1)^2-p1(2)^2+p2(2)^2-p1(3)^2+...
                p2(3)^2;
            A(2,1) = (2*p3(1)-2*p1(1));
            A(2,2) = (2*p3(2)-2*p1(2));
            A(2,3) = (2*p3(3)-2*p1(3));
            A(2,4) = l1^2-l3^2-p1(1)^2+p3(1)^2-p1(2)^2+p3(2)^2-p1(3)^2+...
                p3(3)^2;
            A = rref(A);

            a = p1(1)-A(1,4);
            b = p1(2)-A(2,4);
            m = A(1,3)^2+A(2,3)^2+1;
            n = 2*a*A(1,3)+2*b*A(2,3)-2*p1(3);
            % p = a^2+b^2+p1(3)^2-l1^2
            p = (sqrt(a^2+b^2+p1(3)^2)-l1)*(sqrt(a^2+b^2+p1(3)^2)+l1);     % Issue near origin, machine error

            z1 = (-n+sqrt(n^2-4*m*p))/(2*m);
            x1 = (A(1,4)-z1*A(1,3))/A(1,1);
            y1 = (A(2,4)-z1*A(2,3))/A(2,2);
            z2 = (-n-sqrt(n^2-4*m*p))/(2*m);
            x2 = (A(1,4)-z2*A(1,3))/A(1,1);
            y2 = (A(2,4)-z2*A(2,3))/A(2,2);

            if abs(norm([x1, y1, z1]-p0))<abs(norm([x2, y2, z2]-p0))       % The intersection of three spheres produces two points. Since we assume small movements
                coords = [x1, y1, z1];                                     % between each iteration, we choose the point that's closest to our previous point
            else
                coords = [x2, y2, z2];
            end
        end

        function coords = solvePlane(~, p1, p2, l1, l2, p0)                % Solves for a point on a plane by triangulating constant distances from two known points
            v = cross((p1-p0),(p2-p0));
            v = v/norm(v);

            A = zeros(2,4);

            A(1,1) = (2*p2(1)-2*p1(1));
            A(1,2) = (2*p2(2)-2*p1(2));
            A(1,3) = (2*p2(3)-2*p1(3));
            A(1,4) = l1^2-l2^2-p1(1)^2+p2(1)^2-p1(2)^2+p2(2)^2-p1(3)^2+p2(3)^2;
            A(2,1) = v(1);
            A(2,2) = v(2);
            A(2,3) = v(3);
            A(2,4) = dot(v,p0);
            A = rref(A);

            a = p1(1)-A(1,4);
            b = p1(2)-A(2,4);
            m = A(1,3)^2+A(2,3)^2+1;
            n = 2*a*A(1,3)+2*b*A(2,3)-2*p1(3);
            p = (sqrt(a^2+b^2+p1(3)^2)-l1)*(sqrt(a^2+b^2+p1(3)^2)+l1);     % Issue near origin, machine error

            z1 = (-n+sqrt(n^2-4*m*p))/(2*m);
            x1 = (A(1,4)-z1*A(1,3))/A(1,1);
            y1 = (A(2,4)-z1*A(2,3))/A(2,2);
            z2 = (-n-sqrt(n^2-4*m*p))/(2*m);
            x2 = (A(1,4)-z2*A(1,3))/A(1,1);
            y2 = (A(2,4)-z2*A(2,3))/A(2,2);
            if abs(norm([x1, y1, z1]-p0))<abs(norm([x2, y2, z2]-p0))       
                coords = [x1, y1, z1];                                     
            else
                coords = [x2, y2, z2];
            end
        end

        function obj = solveForce(obj,Fx,Fy,Fz)                            % Static 2-force member solver based on Evan Flickinger's 2014 CSUN master's thesis
            Fx
            Fy
            Fz
            if isequal([Fx,Fy,Fz],[0,0,0])                                 % "Design and analysis of formula SAE car suspension members"
                obj.forces(1:10) = 0;                                      % Unloaded case must eventually account for unsprung mass
                return
            end

            R = obj.contactPt-obj.wheel(2,:);                              % Distance from tire contact patch to wheel-upright interface
            r = obj.outboard-obj.wheel(2,:);                               % Distance from outboard points to wheel-upright interface
            A = zeros(6,3);
            for i = 1:6                                                    % For each arm
                A(1:3,i) = obj.links(i,:)/norm(obj.links(i,:));            % Turns each component into a unit vector for force              
                A(4,i) = A(3,i)*r(i,2)-A(2,i)*r(i,3);                      % Calculates each moment component unit vectors
                A(5,i) = A(3,i)*r(i,1)-A(1,i)*r(i,3);
                A(6,i) = A(2,i)*r(i,1)-A(1,i)*r(i,2);
            end
            B(1:3) = [Fx,Fy,Fz];                                                    
            B(4:6) = cross(B(1:3),R);                                      % Moments about wheel-upright interface by wheel loads
            obj.forces(1:6) = (A\B');                                      % Finds forces through the first 6 links

            f = obj.forces(6)*obj.links(6,:)/norm(obj.links(6,:));         % Vectorized force through pushrod
            v = cross(obj.rocker(1,:),obj.rocker(2,:));                    % Rocker plane normal vector
            fp = f-v*dot(f,v)/norm(v)^2;                                   % Projection of pushrod force on rocker plane (should be near 100% of f)

            mp = norm(cross(fp,obj.rocker(1,:)));                          % Moment about rocker bearing by pushrod force
            sin2 = norm(cross(obj.shock(:),obj.rocker(2,:))/...            % Angle formed by shock and rocker lever arm
                (norm(obj.shock(:))*norm(obj.rocker(2,:))));

            % Old code in case something happens w/ the new stuff
            % sin1 = norm(cross(fp,obj.rocker(1,:))/(norm(fp)*norm(obj.rocker(1,:))));          
            % obj.forces(7) = -norm(fp)*sin1*norm(obj.rocker(1,:))/(sin2*norm(obj.rocker(2,:)));
            
            obj.forces(7) = -mp/(norm(obj.rocker(2,:))*sin2);              % Force through shock
            obj.forces(8:10) = [Fx,Fy,Fz];
        end

        function obj = solveBump(obj, bump)
            if ~isequal(obj.inboard(7:9,:),zeros(3))
                if obj.apex(:,4) == 1
                    i = 1;
                    j = 3;
                else
                    i = 3;
                    j = 1;
                end
                if nargin == 1
                    obj.Spring = obj.Spring.solve(obj.forces(7));
                    l = 200 - obj.Spring.x;
                else
                    l = norm(obj.shock(:))-bump;
                end
                if l > 200+2^-16 || l < 143
                    l = norm(obj.shock);
                end
                obj.inboard(8,:) = obj.solvePlane(obj.inboard(9,:), obj.inboard(7,:), l, norm(obj.rocker(2,:)), obj.inboard(8,:));
                obj.inboard(6,:) = obj.solvePlane(obj.inboard(7,:), obj.inboard(8,:), norm(obj.rocker(1,:)), norm(obj.rocker(3,:)), obj.inboard(6,:));  
                obj.outboard(6,:) = obj.solveSpheres(obj.inboard(i,:), obj.inboard(i+1,:), obj.inboard(6,:), norm(obj.apex(1,:)), norm(obj.apex(2,:)), norm(obj.links(6,:)), obj.outboard(6,:));
                obj.outboard(i,:) = obj.solveSpheres(obj.inboard(i,:), obj.inboard(i+1,:), obj.outboard(6,:), norm(obj.links(i,:)), norm(obj.links(i+1,:)), norm(obj.apex(3,1:3)), obj.outboard(i,:));
                obj.outboard(i+1,:) = obj.outboard(i,:);
                obj.outboard(j,:) = obj.solveSpheres(obj.inboard(j,:), obj.inboard(j+1,:), obj.outboard(i,:), norm(obj.links(j,:)), norm(obj.links(j+1,:)), norm(obj.upright(1,:)), obj.outboard(j,:));
                obj.outboard(j+1,:) = obj.outboard(j,:);
            else
                obj.outboard(3,:)
                obj.outboard(3,1) = obj.outboard(3,1);
                norm(obj.outboard(3,:)-obj.inboard(3,:))
                obj.outboard(3,3) = obj.outboard(3,3)+bump;
                obj.outboard(3,2) = obj.inboard(3,2)+sqrt(norm(obj.links(3,:))^2-(obj.outboard(3,3)-obj.inboard(3,3))^2-obj.links(3,1)^2);
                   
                obj.outboard(4,:) = obj.outboard(3,:);
                obj.outboard(3,:)
                norm(obj.outboard(3,:)-obj.inboard(3,:))
                obj.outboard(1,:) = obj.solveSpheres(obj.inboard(1,:),obj.inboard(2,:),obj.outboard(3,:),norm(obj.links(1,:)),norm(obj.links(2,:)),norm(obj.upright(1,:)),obj.outboard(1,:));
                obj.outboard(2,:) = obj.outboard(1,:);
            end
            obj.outboard(5,:) = obj.solveSpheres(obj.outboard(1,:), obj.outboard(3,:), obj.inboard(5,:), norm(obj.upright(2,:)), norm(obj.upright(3,:)), norm(obj.links(5,:)), obj.outboard(5,:));
            obj.wheel(1,:) = obj.solveSpheres(obj.outboard(1,:), obj.outboard(3,:), obj.outboard(5,:), norm(obj.upright(4,:)), norm(obj.upright(5,:)), norm(obj.upright(6,:)), obj.wheel(1,:));
            obj.wheel(3,:) = obj.solveSpheres(obj.outboard(1,:), obj.outboard(3,:), obj.outboard(5,:), norm(obj.upright(7,:)), norm(obj.upright(8,:)), norm(obj.upright(9,:)), obj.wheel(3,:));
            obj.wheel(2,:) = (obj.wheel(1,:)+obj.wheel(3,:))/2;
            obj = obj.evaluate();
        end

        function obj = solveSteer(obj,steer)
            obj.inboard(5,2) = obj.inboard(5,2)+steer;
            obj.outboard(5,:) = obj.solveSpheres(obj.outboard(1,:), obj.outboard(3,:), obj.inboard(5,:), norm(obj.upright(2,:)), norm(obj.upright(3,:)), norm(obj.links(5,:)), obj.outboard(5,:));     % Finds tie rod outboard point
            obj.wheel(1,:) = obj.solveSpheres(obj.outboard(1,:), obj.outboard(3,:), obj.outboard(5,:), norm(obj.upright(4,:)), norm(obj.upright(5,:)), norm(obj.upright(6,:)), obj.wheel(1,:));
            obj.wheel(3,:) = obj.solveSpheres(obj.outboard(1,:), obj.outboard(3,:), obj.outboard(5,:), norm(obj.upright(7,:)), norm(obj.upright(8,:)), norm(obj.upright(9,:)), obj.wheel(3,:));
            obj.outboard
            obj = obj.evaluate();
        end

        function cornerPlot(obj)
            hold on
            plot3([obj.outboard(1,1), obj.inboard(1,1)], [obj.outboard(1,2), obj.inboard(1,2)], [obj.outboard(1,3), obj.inboard(1,3)], '-o', 'Color', 'b', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)
            plot3([obj.outboard(1,1), obj.inboard(2,1)], [obj.outboard(1,2), obj.inboard(2,2)], [obj.outboard(1,3), obj.inboard(2,3)], '-o', 'Color', 'b', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)
            plot3([obj.outboard(3,1), obj.inboard(3,1)], [obj.outboard(3,2), obj.inboard(3,2)], [obj.outboard(3,3), obj.inboard(3,3)], '-o', 'Color', 'b', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)
            plot3([obj.outboard(3,1), obj.inboard(4,1)], [obj.outboard(3,2), obj.inboard(4,2)], [obj.outboard(3,3), obj.inboard(4,3)], '-o', 'Color', 'b', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)
            plot3([obj.outboard(5,1), obj.inboard(5,1)], [obj.outboard(5,2), obj.inboard(5,2)], [obj.outboard(5,3), obj.inboard(5,3)], '-o', 'Color', 'c', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)

            plot3([obj.inboard(1,1), obj.inboard(2,1)], [obj.inboard(1,2), obj.inboard(2,2)], [obj.inboard(1,3), obj.inboard(2,3)], 'Color', 'k', 'LineWidth', 4)
            plot3([obj.inboard(1,1), obj.inboard(3,1)], [obj.inboard(1,2), obj.inboard(3,2)], [obj.inboard(1,3), obj.inboard(3,3)], 'Color', 'k', 'LineWidth', 4)
            plot3([obj.inboard(4,1), obj.inboard(2,1)], [obj.inboard(4,2), obj.inboard(2,2)], [obj.inboard(4,3), obj.inboard(2,3)], 'Color', 'k', 'LineWidth', 4)
            plot3([obj.inboard(4,1), obj.inboard(3,1)], [obj.inboard(4,2), obj.inboard(3,2)], [obj.inboard(4,3), obj.inboard(3,3)], 'Color', 'k', 'LineWidth', 4)

            if ~isequal(obj.inboard(7:9,:),zeros(3))
                plot3([obj.outboard(6,1), obj.inboard(6,1)], [obj.outboard(6,2), obj.inboard(6,2)], [obj.outboard(6,3), obj.inboard(6,3)], '-o', 'Color', 'r', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)
                plot3([obj.inboard(6,1), obj.inboard(7,1)], [obj.inboard(6,2), obj.inboard(7,2)], [obj.inboard(6,3), obj.inboard(7,3)], '-o', 'Color', 'r', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)
                plot3([obj.inboard(8,1), obj.inboard(7,1)], [obj.inboard(8,2), obj.inboard(7,2)], [obj.inboard(8,3), obj.inboard(7,3)], '-o', 'Color', 'r', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)
                plot3([obj.inboard(6,1), obj.inboard(8,1)], [obj.inboard(6,2), obj.inboard(8,2)], [obj.inboard(6,3), obj.inboard(8,3)], '-o', 'Color', 'r', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)
                plot3([obj.inboard(8,1), obj.inboard(9,1)], [obj.inboard(8,2), obj.inboard(9,2)], [obj.inboard(8,3), obj.inboard(9,3)], '-o', 'Color', 'r', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'LineWidth', 4)

                plot3([obj.inboard(1,1), obj.inboard(7,1)], [obj.inboard(1,2), obj.inboard(7,2)], [obj.inboard(1,3), obj.inboard(7,3)], 'Color', 'k', 'LineWidth', 4)
                plot3([obj.inboard(7,1), obj.inboard(2,1)], [obj.inboard(7,2), obj.inboard(2,2)], [obj.inboard(7,3), obj.inboard(2,3)], 'Color', 'k', 'LineWidth', 4)

                plot3([obj.inboard(7,1), obj.inboard(9,1)], [obj.inboard(7,2), obj.inboard(9,2)], [obj.inboard(7,3), obj.inboard(9,3)], 'Color', 'k', 'LineWidth', 4)
            end

            plot3([obj.outboard(1,1), obj.outboard(3,1)], [obj.outboard(1,2), obj.outboard(3,2)], [obj.outboard(1,3), obj.outboard(3,3)], 'Color', 'k', 'LineWidth', 4)
            plot3([obj.outboard(3,1), obj.outboard(5,1)], [obj.outboard(3,2), obj.outboard(5,2)], [obj.outboard(3,3), obj.outboard(5,3)], 'Color', 'k', 'LineWidth', 4)
            plot3([obj.outboard(1,1), obj.outboard(5,1)], [obj.outboard(1,2), obj.outboard(5,2)], [obj.outboard(1,3), obj.outboard(5,3)], 'Color', 'k', 'LineWidth', 4)
            plot3([obj.wheel(1,1), obj.wheel(3,1)], [obj.wheel(1,2), obj.wheel(3,2)], [obj.wheel(1,3), obj.wheel(3,3)], 'Color', 'k', 'LineWidth', 4)
            surf([obj.tire(:,1,1), obj.tire(:,1,2)], [obj.tire(:,2,1), obj.tire(:,2,2)], [obj.tire(:,3,1), obj.tire(:,3,2)], 'FaceColor', 'k', 'FaceAlpha', 0.5)
            surf([obj.tire(:,1,1), obj.tire(:,1,3)], [obj.tire(:,2,1), obj.tire(:,2,3)], [obj.tire(:,3,1), obj.tire(:,3,3)], 'FaceColor', 'k', 'FaceAlpha', 0.5)
            surf([obj.tire(:,1,2), obj.tire(:,1,4)], [obj.tire(:,2,2), obj.tire(:,2,4)], [obj.tire(:,3,2), obj.tire(:,3,4)], 'FaceColor', 'k', 'FaceAlpha', 0.5)
            surf([obj.tire(:,1,4), obj.tire(:,1,3)], [obj.tire(:,2,4), obj.tire(:,2,3)], [obj.tire(:,3,4), obj.tire(:,3,3)], 'FaceColor', 'k', 'FaceAlpha', 0.5)
            plot3(obj.contactPt(1), obj.contactPt(2), obj.contactPt(3), '-o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r')

            plot3([obj.ic(1),obj.wheel(2,1)], [obj.ic(2),obj.wheel(2,2)], [obj.ic(3),obj.wheel(2,3)], '--', 'LineWidth', 3, 'Color', [.5 0 .5])
            plot3(obj.ic(1), obj.ic(2), obj.ic(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'm')
            plot3([obj.ic(1),obj.contactPt(1)], [obj.ic(2),obj.contactPt(2)], [obj.ic(3),obj.contactPt(3)], '--', 'LineWidth', 3, 'Color', 'g', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r')

            plot3([obj.ic(1),obj.outboard(1,1)], [obj.ic(2),obj.outboard(1,2)], [obj.ic(3),obj.outboard(1,3)], '--', 'LineWidth', 3, 'Color', 'm')

            plot3([obj.ic(1),obj.outboard(3,1)], [obj.ic(2),obj.outboard(3,2)], [obj.ic(3),obj.outboard(3,3)], '--', 'LineWidth', 3, 'Color', 'm')
            


            xlim([obj.contactPt(1)-300,obj.contactPt(1)+300])
            if obj.contactPt(2) > 0
                ylim([-450,750])
            else
                ylim([-750,450])
            end
            zlim([-100,650])
            view(90, 0)
            daspect([1 1 1])
        end
        function obj = findIC(obj,A)
            if nargin == 1
                A = zeros(2,4,2);
                v1 = obj.wheel(2,:).*[1,0,1] - obj.wheel(2,:);
                v2 = obj.wheel(2,:).*[0,0,-1];

                A(1,1:3,1) = cross(v1,v2)/norm(cross(v1,v2));
                A(1,1:3,2) = A(1,1:3,1);
                A(1,4,1) = dot(A(1,1:3,1),obj.wheel(2,:));
                A(1,4,2) = dot(A(1,1:3,2),obj.wheel(2,:).*[1,0,1]);
            end
            
            A(2,1:3,1) = cross(obj.links(1,:), obj.links(2,:))/norm(cross(obj.links(1,:), obj.links(2,:)));
            A(2,4,1) = dot(A(2,1:3,1),obj.outboard(1,:));

            A(2,1:3,2) = cross(obj.links(3,:), obj.links(4,:))/norm(cross(obj.links(3,:), obj.links(4,:)));
            A(2,4,2) = dot(A(2,1:3,2),obj.outboard(3,:));

            A(:,:,1) = rref(A(:,:,1));
            A(:,:,2) = rref(A(:,:,2));

            A(1,:,1) = A(2,:,2);

            A(:,:,1) = rref(A(:,:,1));

            obj.ic = [A(1,4,2)-A(1,4,1)*A(1,2,2)-A(2,4,1)*A(1,3,2),A(1,4,1),A(2,4,1)];

        end
        function obj = ground(obj,p)
            obj.outboard = obj.outboard - p.*[0,0,1];
            obj.inboard(1:6,:) = obj.inboard(1:6,:) - p.*[0,0,1];
            if ~isequal(obj.inboard(7:9,:),zeros(3))
                obj.inboard(7:9,:) = obj.inboard(7:9,:) - p.*[0,0,1];
            end
            obj.wheel = obj.wheel - p.*[0,0,1];
            obj = obj.evaluate;
        end
        function point = rotatePoint(~, p, a1, a2, roll, order)
            if order == "ZYX"
                x = 1;
                y = 2;
                z = 3;
                a = -1;
            elseif order == "ZXY"
                x = 2;
                y = 1;
                z = 3;
                a = 1;
            end
            v = a1-a2;
            alpha = acosd((v(x))/norm([v(x),v(y)]));
            beta = asind((v(z))/norm(v));
            quat = quaternion([a*alpha,0,0],"eulerd",order,"point");
            point = rotatepoint(quat, p-a2);
            quat = quaternion([0, -a*beta ,0],"eulerd",order,"point");
            point = rotatepoint(quat, point);
            quat = quaternion([0,0,roll],"eulerd",order,"point");
            point = rotatepoint(quat, point);
            quat = quaternion([0, a*(beta), 0],"eulerd",order,"point");
            point = rotatepoint(quat, point);
            quat = quaternion([-a*alpha, 0,0],"eulerd",order,"point");
            point = a2+rotatepoint(quat, point);
        end
        function obj = rollCorner(obj, a1, a2, gamma, order)
            obj.outboard(:,:) = obj.rotatePoint(obj.outboard(:,:),a1,a2,gamma, order);
            obj.inboard
            obj.inboard(:,:) = obj.rotatePoint(obj.inboard(:,:),a1,a2,gamma, order);
            obj.inboard
            obj.wheel(:,:) = obj.rotatePoint(obj.wheel(:,:),a1,a2,gamma, order);
            obj = obj.evaluate();
        end
    end
end
