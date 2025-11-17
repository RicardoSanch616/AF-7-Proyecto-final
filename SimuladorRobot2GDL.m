% -------------------------------------------------------------
% Simulador del robot planar de 2 gdl
% -------------------------------------------------------------
clear; clc; close all;
% Parámetros de simulación
ti = 0;          % tiempo inicial
h  = 0.0025;     % incremento del tiempo
tf = 10;         % tiempo final de simulación
ts = ti:h:tf;    % vector de tiempo
% Opciones del solver
opciones = odeset('RelTol',1e-3,'InitialStep',2.5e-3,'MaxStep',2.5e-3);
% Ejecutar el solver
[t,x] = ode45(@cap5_robot2gdl, ts, [0;0;0;0], opciones);

% Gráfica de Posición
subplot(2, 1, 1) % Divide la ventana en 2 filas, 1 col, y usa la 1ra
plot(t, (180/pi)*x(:,1), 'b', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('\theta (°)');
title('Posición Articular del Péndulo');
grid on;

% Gráfica de Velocidad
subplot(2, 1, 2) % Usa la 2da parte de la ventana
plot(t, (180/pi)*x(:,2), 'r', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('d\theta/dt (°/s)');
title('Velocidad Articular del Péndulo');
grid on;


