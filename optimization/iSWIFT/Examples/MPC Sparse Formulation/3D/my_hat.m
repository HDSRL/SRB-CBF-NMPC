function S = my_hat(s)
S = zeros(3,3);
S(1,2) = -s(3);
S(1,3) = s(2);
S(2,1) = s(3);
S(2,3) = -s(1);
S(3,1) = -s(2);
S(3,2) = s(1);

