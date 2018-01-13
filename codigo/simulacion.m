function result = simulacion(tstart, tend, dt)
    %Constantes Fisicas.
    g = 9.81; %[m/s^2]
    m = 1.8;  %[Kg]
    L = 0.27; %[m]  
    I = diag([4.72e-3, 3.72e-3, 3.74e-2]); %matriz de inercia.
    display(I)
    fr = 0.25;%Coeficiente de roce para fuerza de roce (Lagrangiano traslacional)
    k = 1.7e-5; %Constante de Empuje
    b = 2e-7;  %Constante de Torque
    %Vector de tiempo
    ts = tstart:dt:tend;    
    %Numero de muestras de la simulacion.
    N = length(ts);
    %Vectores de salida de la simulacion
    x_out = zeros(3, N);         %Vector Posicion respecto a sistema inercial
    xd_out = zeros(3, N);        %Vector Velocidad respecto a sistema inercial
    theta_out = zeros(3, N);     %Vector Angulos respecto al marco del quadrotor
    thetad_out = zeros(3, N);    %Vector Derivada de angulos respecto a marco del quadrotor
    input_out = zeros(4, N);     %Vector Entradas del sistema (velocidad angular de rotores)
    ref_out = zeros(4,N);
    %Inicializacion del estado del sistema
    x = [0; 0; 10];
    xref = [0; 0; 10];
    ref = zeros(4,1);
    xd = zeros(3,1);
    theta = zeros(3,1);
    thetad = zeros(3,1);
    taux = zeros(3,1);
    %Estructura de parametros para el controlador
    ctes = struct('dt', dt, 'I', I, 'k', k, 'L',L, 'b', b, 'm', m, 'g', g);         
    % Deviation is in degrees/sec.
    deviation = 5;
    thetad = deg2rad(2 * deviation * rand(3, 1) - deviation);
    it = 0; %numero de iteraciones
    for t = ts
        it = it + 1;
        ref = Referencias(it);
        i = controlPD(ctes, thetad, theta, xd, x, it, dt, taux, ref);
        i = Perturbaciones(i, it, theta);
        %Calculo de velocidad y aceleracion
        W = Thd2W(thetad, theta);
        a = acceleration(i,theta, xd, m, g, k, fr);
        Wd = angular_acceleration(i, W, I, L, b, k);
        %Estado siguiente
        W = W + dt * Wd;
        thetad = W2Thd(W, theta); 
        theta = theta + dt * thetad;
        xd = xd + dt * a;
        x = x + dt * xd;
        %Vectores de salida
        x_out(:, it) = x;
        xd_out(:, it) = xd;
        theta_out(:, it) = theta;
        thetad_out(:, it) = thetad;
        input_out(:, it) = i;
        ref_out(:,it) = ref;
    end
    %Construccion de estructura de salida
    result = struct('x', x_out, 'theta', theta_out, 'vel', xd_out,'angvel', thetad_out, 't', ts, 'dt', dt, 'input',input_out,'refs', ref_out);
end
%Control PD para altura y orientacion
function in = controlPD(ctes, thetad,theta, xd, x, it, dt, taux, ref)
    Kdz = 4;
    Kpz = 8;
    Kd = 10;
    Kp = 40;
    a = 0.01; %quiero filtrar de 100 pa arriba
    tref = ref(1:3,1);
    zref = ref(4,1);   
    total =  (ctes.g - Kdz*(xd(3)) - Kpz*(x(3) - zref))  * ...//
        ctes.m/(ctes.k*cos(theta(1)))*cos(theta(2));
%     torques = ctes.I*(Kd*thetad + Kp*(theta-tref));
    torques = (dt/(a + dt))*(ctes.I*(Kd*thetad + Kp*(theta-tref))+ (a/dt)*taux);
%     torques = (dt/(1 + a*dt))*(ctes.I*(Kd*thetad + Kp*(theta-tref))+ (1/(a*dt+1))*taux);
    taux = torques;    
    in = zeros(4,1);
    tthe = torques(1);
    tphi = torques(2);
    tpsi = torques(3);   
    in(1) = total/4 - (tthe/(2*ctes.k*ctes.L)) - (tpsi/(4*ctes.b));
    in(2) = total/4 - (tphi/(2*ctes.k*ctes.L)) + (tpsi/(4*ctes.b));
    in(3) = total/4 + (tthe/(2*ctes.k*ctes.L)) - (tpsi/(4*ctes.b));
    in(4) = total/4 + (tphi/(2*ctes.k*ctes.L)) + (tpsi/(4*ctes.b));   
    for k=0:length(in)-1
        if it>900 && it<95
            %GOOD!!
        end
        if in(k+1)> 620^2
            in(k+1) = 620^2;
        end
        if in(k+1) < 80^2;
            in(k+1) = 80^2;
        end
    end

end
%Calculo de empuje
function T = empuje(inputs, k)
    T = [0; 0; k * sum(inputs)];
end
%Calculo de torques
function tau = torques(inputs, L, b, k)
    tau = [
        L * k * (inputs(1) - inputs(3))
        L * k * (inputs(2) - inputs(4))
        b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
    ];
end
%Convertir derivada de angulos en velocidad angular 
function omega = Thd2W(thetad, ang)
    phi = ang(1);
    theta = ang(2);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    omega = W * thetad;
end
%Convertir velocidad angular a derivada de angulos
function thetad = W2Thd(w, ang)
    phi = ang(1);
    theta = ang(2);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    thetad = W\w;
end
%Calculo de aceleracion en marco inercial
function a = acceleration(i,ang,vels,m,g,k,fr)
    grav = [0; 0; -g];
    R = rotation(ang);
    T =  R * empuje(i,k);
    Fd = - fr * vels;
    a = grav + 1 / m * T + Fd;
end
%Calculo de aceleracion angular
function Wd = angular_acceleration(inputs, w, I, L, b, k)
    tau = torques(inputs, L, b, k);
    Wd = I\(tau - cross(w, I * w));
end
%Perturbaciones al sistema
function i = Perturbaciones(i, it, theta)
    if it>600 && it<605
        i(1) = i(1) + 150^2;
        i(3) = i(3) - 150^2;
    end
%     if it>500 && it<505
%         i(2) = i(2) - 150^2;
%         i(4) = i(4) + 150^2;
%     end
%       if it>500 && it<560
%           theta(3) = theta(3)+ 0.0174533;
%           thetad(3) = 0.872665;
%       end
%       if it>1600 && it<1605
%           i(1) = i(1) - 150^2;
%           i(3) = i(3) + 150^2;
%           i(2) = i(2) - 150^2;
%           i(4) = i(4) + 150^2;
%       end
    i(1) = i(1);
    i(2) = i(2);
    i(3) = i(3);
    i(4) = i(4);
end