function uref  = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn)
    tramp = vmax/amax;
    sf = dist;
    tf = (sf + vmax^2/amax)/vmax;
    if (t < tramp)
        uref = amax * t;
    elseif ((tf - t) < tramp)
        uref = -1 * amax * (tf - t);
    elseif ((t > tramp) && (t < tf - tramp))
        uref = vmax;
    else 
        uref = 0;
    end
    uref = sgn * uref;
end

