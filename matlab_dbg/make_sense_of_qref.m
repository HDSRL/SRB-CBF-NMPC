qref = [  0.152448   0.15483  0.157212  0.159594  0.161976  0.164358   0.16674  0.169123
                0         0         0         0         0         0         0         0
        -0.103118  -0.10473 -0.106341 -0.107952 -0.109563 -0.111175 -0.112786 -0.114397
                0         0         0         0         0         0         0         0];

footPrintTruncated_ =   [   0.31401        inf   0.352123        inf
                         0.0433826        inf   0.017603        inf
                               inf -0.0329336        inf 0.00517856
                               inf  0.0304928        inf 0.00471319
                               inf   0.333066        inf   0.371179
                               inf  -0.233507        inf  -0.259287
                        -0.0519896        inf -0.0138775        inf
                         -0.220617        inf  -0.246397        inf];
                   
com = [ (footPrintTruncated_(1,1)+footPrintTruncated_(7,1))/2; (footPrintTruncated_(2,1)+footPrintTruncated_(8,1))/2 ]

step = [qref(1,8)-com(1); qref(3,8)-com(2)]

second_foot = [footPrintTruncated_(1,1)+step(1); footPrintTruncated_(2,1) + step(2)]

%% VERIFY DISCRETE DYNAMICS
c = 700; m = 12.5; Ts = 0.16/4;
Ar = [0 0 1 0; 0 0 0 1; 0 0 -c/m 0; 0 0 0 -c/m]; 
Br = [0 0; 0 0; 1/m 0; 0 1/m]; Cr = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

ss_r = ss(Ar, Br, Cr, 0);
% Discretize the reference system 
ss_ref_d = c2d(ss_r, Ts, 'zoh');
Ad = ss_ref_d.A 
Bd = ss_ref_d.B 
Cd = ss_ref_d.C; Dd = ss_ref_d.D;

%% LIPM dynamics
g = 9.81; h = 0.26; Ts = 0.05;
Ar = [0 1 0 0; g/h 0 0 0; 0 0 0 1; 0 0 g/h 0]; 
Br = [0 0; -g/h 0; 0 0; 0 -g/h]; Cr = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

ss_r = ss(Ar, Br, Cr, 0);
% Discretize the reference system 
ss_ref_d = c2d(ss_r, Ts, 'tustin');
Ad = ss_ref_d.A 
Bd = ss_ref_d.B 
Cd = ss_ref_d.C; Dd = ss_ref_d.D;
