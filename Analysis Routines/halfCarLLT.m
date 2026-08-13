function Frames = halfCarLLT(carParams,Ay,ForR)
    switch ForR
        case 'F'                                                           % Car params stores both F and R hardpoints, so this indicates which we are modeling
            Frames = Half(carParams.outboardF,carParams.inboardF, ...
                carParams.wheelF,carParams.springF,carParams.preloadF); 
        case 'R'
            Frames = Half(carParams.outboardR,carParams.inboardR, ...
                carParams.wheelR,carParams.springR,carParams.preloadR); 
        otherwise
            error('Please enter F or R for front or rear')
    end    
    n = Ay*10+1;
    Fz0 = carParams.m*9.8*carParams.pFront;
    Ay = linspace(0,Ay,n);
    for i = 1:3
        Frames = Frames.solveStatic(Fz0);
    end
    Frames(2:n) = Frames(1);
    for i = 2:n
        Ay(i)
        dFz = Ay(i)*carParams.m*9.8*carParams.hCG/carParams.TW;
        Frames(i) = Frames(i-1).solveLLT(Fz0,dFz,Ay(i));
        Frames(i) = Frames(i).level();
    end
end