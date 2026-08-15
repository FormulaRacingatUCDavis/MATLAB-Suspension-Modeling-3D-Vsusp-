function [Frames, sag] = springSweep(carParams, springs, p, ForR)
    n = size(springs,1);
    sag = zeros(2,n);
    switch ForR
        case 'F'                                                           % Car params stores both F and R hardpoints, so this indicates which we are modeling
            Frames(2,1:n) = Corner(carParams.outboardF,carParams.inboardF, ...
                carParams.wheelF,carParams.springF,carParams.preloadF); 
        case 'R'
            Frames(2,1:n) = Corner(carParams.outboardR,carParams.inboardR, ...
                carParams.wheelR,carParams.springR,carParams.preloadR); 
        otherwise
            error('Please enter F or R for front or rear')
    end

    for i = 1:n
        springs(i)
        Frames(1,i) = Corner(carParams.outboardF,carParams.inboardF,carParams.wheelF,springs(i,:),p(i));
        for j = 1:3
            Frames(1,i) = Frames(1,i).solveForce(0,0,9.8*310*0.54*0.5);
            Frames(1,i) = Frames(1,i).solveBump();
        end
        Frames(2,i) = Frames(1,i).align(-0.5,-1.235);
        Frames(2,i).Spring.k
        Frames(2,i) = Frames(2,i).solveForce(0,0,9.8*310*0.54*0.5);
        Frames(2,i) = Frames(2,i).solveBump();
        for j = 1:2
            Frames(2,i) = Frames(2,i).align(-0.5,-1.235);
            Frames(2,i) = Frames(2,i).solveForce(0,0,9.8*310*0.54*0.5);
            Frames(2,i) = Frames(2,i).solveBump();
        end
        sag(1,i) = Frames(2,i).Spring.k;
        sag(2,i) = Frames(2,i).Spring.x/50;
    end
    clf
    hold on
    plot(sag(1,:),sag(2,:),'LineWidth',5)
    scatter(sag(1,:),sag(2,:),100,'filled','o','MarkerFaceColor','#D95319')
    drawnow
    xlim([sag(1,1),sag(1,end)])
    xlabel('Spring Rate (lb/in)','FontSize',16)
    ylabel('Static Sag (percentage of 50mm shock travel)','FontSize',16)
    title('FE13 Sag vs. Spring Rate (with driver, m = 310 kg)','FontSize',24)
    ylim([0,1])
end