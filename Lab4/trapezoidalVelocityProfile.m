function uref  = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn)
    tramp = vmax/amax;
    sf = dist;
    tf = (sf + vmax^2/amax)/vmax;
    if (t < 0)
        uref = 0;
    elseif (t < tramp)
        uref = amax * t;
    elseif ((t > tramp) && (t < tf - tramp))
        uref = vmax;
    elseif (t > tf)
        uref = 0;
    elseif ((tf - t) < tramp)
        uref = amax * (tf - t);
    else 
        uref = 0;
    end
    if (uref < -1 * 0.25)
        fprintf("!!!! t = %d, tramp = %d, tf = %d",t,tramp,tf);
    end
    uref = sgn * uref;
end

