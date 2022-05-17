function [X_dot] = nonlin_ct_bicycle(X, U, c, wb, m)
    %UNTITLED6 Summary of this function goes here
    %   Detailed explanation goes here
    % X = [x, y, theta, v, omega]'
    % U = [delta, u]'
    arguments
        X (5, 1) % States
        U %(2,1) % Inputs
        c (1,1) = 4700; % Tire Cornering Stiffness(N/deg)
        wb (1,1) = 1550; %Wheelbase(mm)
        m (1,1) = 200; % Mass of vehicle (kg)
    end
    
    %% Defining Dependent Parameters
    a = 0.5 * wb;
    b = 0.5 * wb;
    Iz = (0.5*m)*a^2 + (0.5*m)*b^2;

    % Velocity...
    if size(U) ~= 2
        warning('U not inputed as correct size')
        U = [U; 10];
    end
    u = U(2);
    if u == 0
        warning('need movement for dynamics')
        u = 10;
    end
    u = 10;
    
    %% States & Inputs
    x = X(1);
    y = X(2);
    theta = X(3);
    v = X(4);
    omega = X(5);
    
    delta = U(1);
    
    V = sqrt(v^2 + u^2);
    beta = atan(v/u);
    
    
    %% Computing Bike A Matrix
    A11 = (2*c)/(m*u);
    A12 = (c*a - c*b)/(m*u) + u;
    A21 = (c*a - c*b)/(Iz*u);
    A22 = (c*a^2 + c*b^2)/(Iz*u);

    A = -[A11 A12; A21 A22];
    
    %% Computing Bike B Matrix
    
    B11 = c/m;
    B21 = (c*a)/Iz;
    
    B = [B11; B21];
    
    %% Tyre Model
    bike_dot = A * [v, omega]' + B * [delta];
    v_dot = bike_dot(1);
    omega_dot = bike_dot(2);
    
    
    %% CT Update EQ
    x_dot = V * cos(theta);
    y_dot = V * sin(theta);
    theta_dot = omega;
    
    X_dot = [x_dot, y_dot, theta_dot, v_dot, omega_dot]';
end
