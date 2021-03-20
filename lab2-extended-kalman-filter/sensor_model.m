function y = sensor_model(mu)
    %Poly fit
%     y1 = 4.1136*mu(1)^2 -4.7153*mu(1) + 9.9209;
%     y2 = 2.3153*mu(2)^2 -3.0747*mu(2) + 9.6963;  

    %Inverse fit
    y1 = (8.3741*mu(1)+0.2395)/(mu(1)+0.0123);
    y2 = (8.3558*mu(2)+1.3344)/(mu(2)+0.1294);
    
    %Power fit
%     y1 = 8.2486*mu(1)^-0.0619;
%     y2 = 8.4711*mu(2)^-0.0472;
%     
    y = [y1;y2];
end