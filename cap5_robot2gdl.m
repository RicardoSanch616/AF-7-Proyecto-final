% -------------------------------------------------------------
% Robot de transmisión directa de 2GDL
% -------------------------------------------------------------
function xp = cap5_robot2gdl(t,x)

q  = [x(1); x(2)];   % vector de posición articular
qp = [x(3); x(4)];   % vector de velocidad articular

% Matriz de inercia
M = [3.117 + 0.2*cos(q(2))*0.108 + 0.1*cos(q(2)),   0.108 + 0.1*cos(q(2))*0.108;
     0.108 + 0.1*cos(q(2))*0.108,                   0.108];

% Matriz de Coriolis y centrípeta
C = [-0.2*sin(q(2))*qp(2) - 0.1*sin(q(2))*qp(2);
      0.1*sin(q(2))*qp(1)*0.0];

% Par gravitacional
par_grav = [39.3*sin(q(1)) + 1.95*sin(q(1)+q(2));
            1.95*sin(q(1)+q(2))];

% Par de fricción viscosa
fr = [1.86*qp(1) + 1.93*sign(qp(1));
      0.16*qp(2) + 0.3*sign(qp(2))];

% Par aplicado en actuadores
tau = [(1-exp(-0.8*t))*32.0 + 56*sin(16*t+0.1) + 12*sin(20*t+0.15);
       (1-exp(-1.8*t))*1.2 + 8*sin(26*t+0.08) + 2*sin(12*t+0.34)];

% Aceleración articular
q2p = inv(M)*(tau - C.*qp - par_grav - fr);

% Vector de salida
xp = [qp(1); qp(2); q2p(1); q2p(2)];
end
