function [Frames,bump,shock,idx] = ForceSweep(carParams, ForR)
    switch ForR
        case 'F'                                                           % Car params stores both F and R hardpoints, so this indicates which we are modeling
            Frames = Corner(carParams.outboardF,carParams.inboardF, ...
                carParams.wheelF,carParams.springF,carParams.preloadF); 
        case 'R'
            Frames = Corner(carParams.outboardR,carParams.inboardR, ...
                carParams.wheelR,carParams.springR,carParams.preloadR); 
        otherwise
            error('Please enter F or R for front or rear')
    end
    m = 310;
    pFront = 0.54;
    n = 51;
    FzStatic = 0.5*pFront*m*9.8;
    Fz = linspace(0,FzStatic*2,n-1)
    for i = 1:3
        Frames = Frames.solveForce(0,0,FzStatic);
        Frames = Frames.solveBump();
    end
    Frames = Frames.ground(Frames.contactPt);
    Frames(2:n) = Frames;
    idx = find(abs(Fz-FzStatic) == min(abs(Fz-FzStatic)));
    Fz(idx(2)+1:end+1) = Fz(idx(2):end);
    Fz(idx(2)) = FzStatic;
    bump = zeros(1,n);
    shock = bump;
    for i = idx(1):-1:1
        for j = 1:3
            Frames(i) = Frames(i+1).solveForce(0,0,Fz(i));
            Frames(i) = Frames(i).solveBump(); 
        end
        bump(i) = Frames(i).wheel(2,3)-Frames(idx(2)).wheel(2,3);
        travel(i) = norm(Frames(i).shock(:))-norm(Frames(idx(2)).shock(:));
    end

    for i = idx(2)+1:n
        for j = 1:3
            Frames(i) = Frames(i-1).solveForce(0,0,Fz(i));
            Frames(i) = Frames(i).solveBump();
        end
        bump(i) = Frames(i).wheel(2,3)-Frames(idx(2)).wheel(2,3);
        travel(i) = norm(Frames(i).shock(:))-norm(Frames(idx(2)).shock(:));
    end
    for i = 1:n
        clf
        Frames(i).cornerPlot()
        drawnow
    end
    idx = idx(2);
end