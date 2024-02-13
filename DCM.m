function V_v = DCM(phi,theta,psi,u,v,w)

C1 = [cos(psi) sin(psi) 0;
   -sin(psi) cos(psi) 0;
   0 0 1];
  
C2 = [cos(theta) 0 -sin(theta);
    0 1 0;
    sin(theta) 0 cos(theta)];

C3 = [1 0 0;
    0 cos(phi) sin(phi);
    0 -sin(phi) cos(phi)];

C_b_v = C3 * C2 * C1;

C_v_b = transpose(C_b_v);

V_v = C_v_b * [u;v;w];

end