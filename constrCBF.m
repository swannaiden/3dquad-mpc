function [c, ceq] = constrCBF(U,h, hdot, eta, delta)
    c = double( -hdot(U) - eta*h +delta);
    ceq = [];
end