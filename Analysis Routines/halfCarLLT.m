function [Frames, Ay, roll] = halfCarLLT(carParams,Ay,steer,ForR)
    switch ForR
        case 'F'                                                           % Car params stores both F and R hardpoints, so this indicates which we are modeling
            Frames = Half(carParams.outboardF,carParams.inboardF, ...
                carParams.wheelF,carParams.springF,carParams.preloadF); 
            TW = carParams.TWf;
            p = carParams.pFront;
        case 'R'
            Frames = Half(carParams.outboardR,carParams.inboardR, ...
                carParams.wheelR,carParams.springR,carParams.preloadR); 
            TW = carParams.TWr;
            p = 1 - carParams.pFront;
        otherwise
            error('Please enter F or R for front or rear')
    end   
    roll = 0;
    n = abs(Ay)*10+1;
    Fz0 = carParams.m*9.8*p;
    Ay = linspace(0,Ay,n);
    steer = linspace(0,steer,n);
    Frames = Frames.evaluate();
    staticToe = Frames.L.params(1);
    staticCamber = Frames.L.params(2);
    for i = 1:3
        Frames = Frames.solveStatic(Fz0);
    end
    Frames = Frames.align(staticToe,staticCamber);
    Frames(2:n) = Frames(1);
    for i = 2:n  
        dFz = 0.5*Ay(i)*carParams.m*9.8*carParams.hCG/TW;
        if abs(dFz) > Fz0/2
            dFz = Fz0/2;
            Ay(i) = 0.5*Fz0/(carParams.m*9.8*carParams.hCG)*TW;
        end
        Frames(i) = Frames(i-1).solveLLT(Fz0,dFz,Ay(i));
        Frames(i) = Frames(i).solveSteer(steer(i)-steer(i-1));
        Frames(i) = Frames(i).level();
    end
end