
function [bump, Frames] = BumpSweep(carParams, ForR)
    clf
    n = 51;                                                                
    
    switch ForR
        case 'F'                                                           % Car params stores both F and R hardpoints, so this indicates which we are modeling
            Frames(1) = Corner(carParams.outboardF,carParams.inboardF, ...
                carParams.wheelF,carParams.kF); 
        case 'R'
            Frames(1) = Corner(carParams.outboardR,carParams.inboardR, ...
                carParams.wheelR,carParams.kR); 
        otherwise
            error('Please enter F or R for front or rear')
    end
    
    % Our linspace for steerSweep will be lock to look on the steering rack given in
    % travel from neutral

    bump = linspace(norm(Frames(1).shock)-200,norm(Frames(1).shock)-150,n);

    % Everything from here
    idx = find(abs(bump) == min(abs(bump)));

    Frames(1) = Frames(1).solveBump(bump(idx));
    Frames(1) = Frames(1).ground(Frames(1).contactPt);  
    Frames(2:n) = Frames(1);
    
    bump = bump - bump(idx);
    % To here does not have to be included in SteerSweep
    % idx will be our middle index ceil(n/2)
    
    Frames(idx) = Frames(idx).findTire();
    
    wheelParams = zeros(5,n);
    wheelParams(:,idx) = Frames(idx).params;
                                                                           % The program assumes small travel between frames, so we start from static and work
    for i = idx-1:-1:1                                                     % our way toward each extreme from the middle outwards
        Frames(i) = Frames(i+1).solveBump(bump(i)-bump(i+1));
        Frames(i) = Frames(i).findTire();
        wheelParams(:,i) = Frames(i).params;
    end
    for i = idx+1:n
        Frames(i) = Frames(i-1).solveBump(bump(i)-bump(i-1));
        Frames(i) = Frames(i).findTire();
        wheelParams(:,i) = Frames(i).params;
    end
    
    for i = 1:n                                                            % Code for live animation
        subplot(2,3,1)
        Frames(i).cornerPlot()
        
        subplot(2,3,2)
        hold on
        plot(bump(:),wheelParams(2,:))
        plot(bump(i),Frames(i).params(2),'-o','MarkerSize',5,'LineWidth',5)
        xlabel('Shock Compression (mm, from static)')
        ylabel('FL Camber (degrees)')
        title('FL Camber vs. FL Bump')

        subplot(2,3,3)
        hold on
        plot(bump(:),wheelParams(1,:))
        plot(bump(i),Frames(i).params(1),'-o','MarkerSize',5,'LineWidth',5)
        xlabel('Shock Compression (mm, from static)')
        ylabel('FL Toe (degrees)')
        title('FL Toe vs. FL Bump')

        subplot(2,3,4)
        hold on
        plot(bump(:),wheelParams(3,:))
        plot(bump(i),Frames(i).params(3),'-o','MarkerSize',5,'LineWidth',5)
        xlabel('Shock Compression (mm, from static)')
        ylabel('FL Caster (degrees)')
        title('FL Caster vs. FL Bump')

        subplot(2,3,5)
        hold on
        plot(bump(:),wheelParams(4,:))
        plot(bump(i),Frames(i).params(4),'-o','MarkerSize',5,'LineWidth',5)
        xlabel('Shock Compression (mm, from static)')
        ylabel('FL Mechanical Trail (mm)')
        title('FL Mechanical Trail vs. FL Bump')

        subplot(2,3,6)
        hold on
        plot(bump(:),wheelParams(5,:))
        plot(bump(i),Frames(i).params(5),'-o','MarkerSize',5,'LineWidth',5)
        xlabel('Shock Compression (mm, from static)')
        ylabel('FL Scrub Radius (mm)')
        title('FL Scrub Radius vs. FL Bump')
        
        drawnow

        clf
    end

    subplot(2,3,1)
    Frames(idx).cornerPlot()

    subplot(2,3,2)
    hold on
    plot(bump(:),wheelParams(2,:))
    plot(bump(idx),Frames(idx).params(2),'-o','MarkerSize',5,'LineWidth',5)
    xlabel('Shock Compression (mm, from static)')
    ylabel('FL Camber (degrees)')
    title('FL Camber vs. FL Bump')

    subplot(2,3,3)
    hold on
    plot(bump(:),wheelParams(1,:))
    plot(bump(idx),Frames(idx).params(1),'-o','MarkerSize',5,'LineWidth',5)
    xlabel('Shock Compression (mm, from static)')
    ylabel('FL Toe (degrees)')
    title('FL Toe vs. FL Bump')

    subplot(2,3,4)
    hold on
    plot(bump(:),wheelParams(3,:))
    plot(bump(idx),Frames(idx).params(3),'-o','MarkerSize',5,'LineWidth',5)
    xlabel('Shock Compression from Static (mm, at wheel center)')
    ylabel('FL Caster (degrees)')
    title('FL Caster vs. FL Bump')

    subplot(2,3,5)
    hold on
    plot(bump(:),wheelParams(4,:))
    plot(bump(idx),Frames(idx).params(4),'-o','MarkerSize',5,'LineWidth',5)
    xlabel('Shock Compression (mm, from static)')
    ylabel('FL Mechanical Trail (mm)')
    title('FL Mechanical Trail vs. FL Bump')

    subplot(2,3,6)
    hold on
    plot(bump(:),wheelParams(5,:))
    plot(bump(idx),Frames(idx).params(5),'-o','MarkerSize',5,'LineWidth',5)
    xlabel('Shock Compression (mm, from static)')
    ylabel('FL Scrub Radius (mm)')
    title('FL Scrub Radius vs. FL Bump')

    drawnow
end