function dz = robot_dynamics(t, z, tau, params, x_ddot_func, theta_ddot_func, x_b_ddot_func)
    
    % Extract states: z = [x; xd; theta; thetad; x_b; x_bd]
    x = z(1);
    xd = z(2);
    theta = z(3);
    thetad = z(4);
    x_b = z(5);
    x_bd = z(6);
    
    % Create the full list of arguments for the functions
    args = {x, xd, theta, thetad, x_b, x_bd, tau, params.M_wheel, params.M_body, params.m_ball, params.h, params.d, params.R, params.r, params.I_body,...
        params.I_wheel, params.I_ball, params.g};
    
    % Calculate accelerations using the derived functions
    xdd = x_ddot_func(args{:});
    thetadd = theta_ddot_func(args{:});
    x_bdd = x_b_ddot_func(args{:});
    
    % Return state derivatives
    dz = [xd; xdd; thetad; thetadd; x_bd; x_bdd];
end