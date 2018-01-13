%Graficos de datos de simulacion e implementacion
function j = Graficos(data)
    X = data.x(1,:);
    Y = data.x(2,:);
    Z = data.x(3,:);
    P = data.theta(1,:)*(180/pi);
    R = data.theta(2,:)*(180/pi);
    Y = data.theta(3,:)*(180/pi);
    dX = data.vel(1,:);
    dY = data.vel(2,:);
    dZ = data.vel(3,:);
    dP = data.angvel(1,:)*(180/pi);
    dR = data.angvel(2,:)*(180/pi);
    dY = data.angvel(3,:)*(180/pi);
    ESC1 = (data.input(1,:)+754523)/139216;
    ESC2 = (data.input(2,:)+754523)/139216;
    ESC3 = (data.input(3,:)+754523)/139216;
    ESC4 = (data.input(4,:)+754523)/139216;
    REFP = data.refs(1,:)*(180/pi);
    REFR = data.refs(2,:)*(180/pi);
    REFY = data.refs(3,:)*(180/pi);
    REFZ = data.refs(4,:);
%Estabilizacion con perturbacion
figure(1)
subplot(1,2,1)
plot(data.t,P,'r')
hold on
plot(data.t,R,'b')
plot(data.t,Y,'g')
plot(data.t,REFP,'k');
plot(data.t,REFR,'c');
plot(data.t,REFY,'m');
title ('Desplazamiento Angular [\degree]');
xlabel('tiempo [s]');
ylabel('Angulo [\degree]');
hold off
legend('Pitch','Roll','Yaw','RPitch','RRoll','RYaw');
%Velocidad Angular
subplot(1,2,2)
plot(data.t,dP,'r')
hold on
plot(data.t,dR,'b')
plot(data.t,dY,'g')
title ('Velocidad Angular [\degree/s]');
xlabel('tiempo [s]');
ylabel('Velocidad Angular [\degree/s]');
hold off
legend(' dPitch','dRoll','dYaw');
%Altura eje Z [m]
figure(4)
subplot(1,2,1)
plot(data.t,Z,'r')
hold on
plot(data.t,REFZ,'b')
title ('Altura en el Tiempo [m]');
xlabel('tiempo [s]');
ylabel('Altura [m]');
hold off
legend('Altura Z','Ref Z');
subplot(1,2,2)
plot(data.t,ESC1,'r')
hold on
plot(data.t,ESC2,'b');
plot(data.t,ESC3,'c');
plot(data.t,ESC4,'g');
title ('Actuacion Duty Cicle [%]');
xlabel('tiempo [s]');
ylabel('Duty [%]');
hold off
legend('ESC1','ESC2','ESC3','ESC4');
% Seguimiento referencias constantes
figure(2)
plot(rpm1,empuje1,'og')
hold on
plot(rpm2,empuje2,'oc')
plot(rpm3,empuje3,'ob')
plot(rpm4,empuje4,'or')
title ('Velocidad Angular [RPM] v/s Empuje [N]');
xlabel('RMP');
ylabel('Empuje [N]');
plot(x21,y21,'g')
plot(x22,y22,'c')
plot(x23,y23,'b')
plot(x24,y24,'r')
hold off
legend('Motor1','Motor2','Motor3','Motor4','Ajuste Cuadratico 1','Ajuste Cuadratico 2','Ajuste Cuadratico 3', 'Ajuste Cuadratico 4');
%Ajuste Lineal RadS2 v/s empuje
figure(3)
plot(rads21,empuje1,'og')
hold on
plot(rads22,empuje2,'oc')
plot(rads23,empuje3,'ob')
plot(rads24,empuje4,'or')
title ('Velocidad Angular [Rad/s^2] v/s Empuje [N]');
xlabel('[Rad/s^2]');
ylabel('Empuje [N]');
plot(xr11,yr11,'g')
plot(xr12,yr12,'c')
plot(xr13,yr13,'b')
plot(xr14,yr14,'r')
hold off
legend('Motor1','Motor2','Motor3','Motor4','Ajuste Lineal 1','Ajuste Lineal 2','Ajuste Lineal 3', 'Ajuste Lineal 4');
%Ajuste Cuadratico Rads v/s empuje
figure(4)
plot(rads1,empuje1,'og')
hold on
plot(rads2,empuje2,'oc')
plot(rads3,empuje3,'ob')
plot(rads4,empuje4,'or')
title ('Velocidad Angular [Rad/s] v/s Empuje [N]');
xlabel('[Rad/s]');
ylabel('Empuje [N]');
plot(xr21,yr21,'g')
plot(xr22,yr22,'c')
plot(xr23,yr23,'b')
plot(xr24,yr24,'r')
hold off
legend('Motor1','Motor2','Motor3','Motor4','Ajuste Cuadratico 1','Ajuste Cuadratico 2','Ajuste Cuadratico 3', 'Ajuste Cuadratico 4');

end