function [c, ceq] = constrSimple(U,V,Vdot, delta, eta2)
    c = double( Vdot(U) + eta2*V -U(3));
    ceq = [];
end