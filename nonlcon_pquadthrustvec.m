function [c,ceq] = nonlcon_pquadthrustvec(U,Lfh,Lgh,V,eta)
    ceq = double( Lfh+Lgh*[U(1); U(2)] + eta*V );
    c = [];
end