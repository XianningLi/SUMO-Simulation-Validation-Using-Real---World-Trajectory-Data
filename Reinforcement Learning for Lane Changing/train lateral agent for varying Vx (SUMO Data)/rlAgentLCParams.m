
m = 1360;
Iz = 1993;
lf = 1.45;
lr = 1.06;
% lr = 1.45;
% lf = 1.06;
Cf = 1.51*100000;
Cr = 1.46*100000;
Ts = 0.1;
Vx = 10;

A = [
    0 1 0 0;
    0 -(1/(m*Vx))*(2*Cf+2*Cr) (2*Cf+2*Cr)/(m) -(2*Cf*lf-2*Cr*lr)/(m*Vx);
    0 0 0 1;
    0 -(1/(Iz*Vx))*(2*lf*Cf-2*lr*Cr) (2*Cf*lf-2*Cr*lr)/Iz -(1/(Iz*Vx))*(2*lf^2*Cf+2*lr^2*Cr)
    ];
B = [0 2*Cf/m 0 2*lf*Cf/Iz]';

Ad = eye(4)+A*Ts;
Bd = B*Ts;

Qlat = diag([20 50 2000 3000]);
Rlat = eye(1);

Klo = [4.4721, 110.3821];

Alo = [0 1; 0 0];
Blo = [0; 1/m];

Klat = [4.4458,    1.2388,  123.2048,   53.8168];