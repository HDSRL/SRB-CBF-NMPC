function [b1 b2 b3] = coeffv(dt,mass,g,fop)

   b1 = eye(3);
   b2 = [1 0 0 1 0 0 1 0 0 1 0 0;...
         0 1 0 0 1 0 0 1 0 0 1 0;...
         0 0 1 0 0 1 0 0 1 0 0 1]*dt*(1/mass);
  
%     b2 = b2*dt*(1/mass);
    b3 = [0;0;-g]*dt+b2*fop;




end