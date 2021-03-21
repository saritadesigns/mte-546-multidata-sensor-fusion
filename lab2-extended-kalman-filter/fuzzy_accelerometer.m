function [Xmu,Ymu] = fuzzy_accelerometer(trajectory,xAccel,yAccel)

Xlow = 11;
Xmid = 14;
Xhigh = 17;

Ylow = 8.8;
Ymid = 9.3;
Yhigh = 9.8;

if trajectory==1    
    %mu_original
    Xmu_original = lowmid(xAccel,Xlow,Xmid);
    Ymu_original = midhigh(yAccel,Ymid,Yhigh);
    
    %mu_rotating
    Xmu_rotating = lowhigh(xAccel,Xlow,Xmid,Xhigh);
    Ymu_rotating = lowhigh(yAccel,Ylow,Ymid,Yhigh);
        
    %mu_rotated
    Xmu_rotated = midhigh(xAccel,Xmid,Xhigh);
    Ymu_rotated = lowmid(yAccel,Ylow,Ymid);

elseif trajectory==2
    Xlow = 0;
    Xmid = 0;
    Xhigh = 0;
    
    Ylow = 0;
    Ymid = 0;
    Yhigh = 0;
    
    %mu_original
    Xmu_original = lowmid(xAccel,Xlow,Xmid);
    Ymu_original = midhigh(yAccel,Ymid,Yhigh);
    
    %mu_rotating
    Xmu_rotating = lowhigh(xAccel,Xlow,Xmid,Xhigh);
    Ymu_rotating = lowhigh(yAccel,Ylow,Ymid,Yhigh);
        
    %mu_rotated
    Xmu_rotated = midhigh(xAccel,Xmid,Xhigh);
    Ymu_rotated = lowmid(yAccel,Ylow,Ymid);
    
else
    Xlow = 0;
    Xmid = 0;
    Xhigh = 0;
    
    Ylow = 0;
    Ymid = 0;
    Yhigh = 0;
    
    %mu_original
    Xmu_original = lowmid(xAccel,Xlow,Xmid);
    Ymu_original = midhigh(yAccel,Ymid,Yhigh);
    
    %mu_rotating
    Xmu_rotating = lowhigh(xAccel,Xlow,Xmid,Xhigh);
    Ymu_rotating = lowhigh(yAccel,Ylow,Ymid,Yhigh);
        
    %mu_rotated
    Xmu_rotated = midhigh(xAccel,Xmid,Xhigh);
    Ymu_rotated = lowmid(yAccel,Ylow,Ymid);
end

Xnot_rotating = Xmu_original + Xmu_rotated;
Xrotating = Xmu_rotating;

Ynot_rotating = Ymu_original + Ymu_rotated;
Yrotating = Ymu_rotating;

%mu: [notRotating;rotating]
Xmu = [Xnot_rotating;Xrotating];
Ymu = [Ynot_rotating;Yrotating];

end

