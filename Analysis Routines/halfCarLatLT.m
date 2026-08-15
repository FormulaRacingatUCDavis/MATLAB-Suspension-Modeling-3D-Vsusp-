function [Frames, Ay, roll] = halfCarLatLT(carParams,Ay,steer,ForR)
    switch ForR
        case 'F'                                                           % Car params stores both F and R hardpoints, so this indicates which we are modeling
            Frames = Half(carParams.outboardF,carParams.inboardF, ...
                carParams.wheelF,carParams.springF,carParams.preloadF);
            p = carParams.pFront;
        case 'R'
            Frames = Half(carParams.outboardR,carParams.inboardR, ...
                carParams.wheelR,carParams.springR,carParams.preloadR); 
            TW = carParams.TWr;
            p = 1 - carParams.pFront;
        otherwise
            error('Please enter F or R for front or rear')
    end
    n = abs(Ay)*10+1;
    roll = zeros(1,n);
    Fz0 = carParams.m*9.8*p;
    Ay = linspace(0,Ay,n);
    steer = linspace(0,steer,n);
    Frames = Frames.evaluate();
    staticToe = Frames.L.params(1);
    staticCamber = Frames.L.params(2);
    for i = 1:3
        Frames = Frames.solveStatic(Fz0);
        Frames = Frames.align(staticToe,staticCamber);
    end
    Frames(2:n) = Frames(1);
    for i = 2:n  
        dFz = p*Ay(i)*carParams.m*9.8*carParams.hCG/(Frames(n-1).L.contactPt(2)-Frames(n-1).R.contactPt(2))*1000;
        if abs(dFz) > Fz0/2
            dFz = Fz0/2;
            Ay(i) = 0.5*Fz0/(carParams.m*9.8*carParams.hCG)*TW;
        end
        Frames(i) = Frames(i-1).solveLLT(Fz0,dFz,Ay(i));
        Frames(i) = Frames(i).solveSteer(steer(i)-steer(i-1));
        Frames(i) = Frames(i).level();
        roll(i) = Frames(i).roll;
    end
end