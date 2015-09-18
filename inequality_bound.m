function sigma = inequality_bound(h,c,eta,epsilon,e)
    sigma = zeros(size(h));
    h2 = h - eta;
    sigma(h2 >= epsilon) = -tan(c*pi/2);
    sigma(h2 >= 0 & h2 < epsilon) = ...
                    -tan(c*pi/2/epsilon*h2(h2 >= 0 & h2 < epsilon));
    sigma(h >= 0 & h2 < 0) = -e*h2(h >= 0 & h2 < 0)/eta;
    sigma(h < 0) = e;
end