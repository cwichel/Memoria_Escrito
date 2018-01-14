%Archivo de Referencias constantes 
function R = Referencias(it)
    R = zeros(4,1);
    if it < 400
        R(1:3,1) = [0;0;0];   
        R(4,1) = 4;
    elseif (it >= 400 && it < 800)
        R(1:3,1) = [0;0;0];   
        R(4,1) = 4;
    elseif (it >= 800 && it < 1200)
        R(1:3,1) = [0;0;0];   
        R(4,1) = 4;
    elseif (it >= 1200 && it < 1600)
        R(1:3,1) = [0;0;0];   
        R(4,1) = 4;
    elseif (it >= 1600 && it < 19999)
        R(1:3,1) = [0;0;0];   
        R(4,1) = 4;         
    end
end 