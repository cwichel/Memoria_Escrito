% Matriz de rotacion
function R = rotation(ang)
    phi = ang(1);
    theta = ang(2);
    psi = ang(3);

    R = zeros(3);
    R(:, 1) = [
        cos(theta)*cos(psi)
        cos(theta)*sin(psi)
        -sin(theta)
    ];
    R(:, 2) = [
        cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi)
        sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi)
        sin(phi)*cos(theta) 
    ];
    R(:, 3) = [
        cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) 
        sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi)
        cos(phi)*cos(theta)
    ];
end