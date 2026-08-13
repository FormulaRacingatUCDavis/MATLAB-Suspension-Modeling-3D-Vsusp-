function [MR, bump] = FindMR(Frames)
    n = length(Frames);
    bump = zeros(1,n);
    bump(1) = Frames(1).wheel(2,3);
    MR = zeros(1,n-1);
    for i = 1:n-1
        bump(i+1) = Frames(i+1).wheel(2,3);
        MR(i) = abs((norm(Frames(i+1).shock) - norm(Frames(i).shock))/...
            (bump(i+1)-bump(i)));
    end
    bump = (bump(1:end-1)+bump(2:end))/2;
end