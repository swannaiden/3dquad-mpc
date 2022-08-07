function [c, ceq] = constr2(U,Lfh,Lgh,V,eta2, delta)
    ceq = double( Lfh+Lgh*[U(1); U(2)] + eta2*V -delta);
    c = [];
end