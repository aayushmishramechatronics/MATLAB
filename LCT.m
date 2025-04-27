% System: G(s) = 1 / (s^2 + 2s + 1)
% Objectives of Question:
% 1. we have to design a PI Controller
% 2. after designing it, we have to evaluate the ramp response of the PI
% Controller
% 3. we will analyze the ramp response using Root Locus, Bode, and Nyquist plots

function main()
    % Clear workspace and close all figures
    clear all; close all; clc;
    
    % Initialize Laplace variable
    s = tf('s');
    
    % Define system and get transfer function
    G = define_system(s);
    
    % Design PI controller
    [C, ~, Ki] = design_pi_controller(s);
    
    % Evaluate closed-loop performance
    evaluate_performance(G, C);
    
    % Generate analysis plots
    generate_analysis_plots(G, C);
end

function G = define_system(s)
    % Create transfer function of the plant
    G = 1 / (s^2 + 2*s + 1);
    
    fprintf('System defined:\n');
    disp(G);
end

function [C, Kp, Ki] = design_pi_controller(s)
    % PI controller design parameters
    Kp = 1.5;   % Proportional gain
    Ki = 0.75;  % Integral gain
    
    % Create PI controller transfer function
    C = Kp + Ki/s;
    
    fprintf('\nDesigned PI Controller:\n');
    fprintf('Kp = %.2f, Ki = %.2f\n', Kp, Ki);
    disp(C);
end

function evaluate_performance(G, C)
    % Create closed-loop system
    T = feedback(C*G, 1);
    
    % Time vector for simulation
    t = 0:0.01:10;
    
    % Create ramp input
    ramp_input = t;
    
    % Simulate response to ramp input
    [y, t] = lsim(T, ramp_input, t);
    
    % Plot ramp response
    figure('Name', 'Ramp Response', 'NumberTitle', 'off');
    plot(t, ramp_input, '--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(t, y, 'LineWidth', 2, 'DisplayName', 'System Response');
    title('Closed-Loop Ramp Response');
    xlabel('Time (s)');
    ylabel('Amplitude');
    legend('Location', 'southeast');
    grid on;
    hold off;
    
    % Calculate steady-state error
    ss_error = ramp_input(end) - y(end);
    fprintf('\nSteady-state error for ramp input: %.4f\n', ss_error);
    
    % Step response characteristics
    step_info = stepinfo(T);
    fprintf('\nStep Response Characteristics:\n');
    fprintf('Rise Time: %.3f s\n', step_info.RiseTime);
    fprintf('Settling Time: %.3f s\n', step_info.SettlingTime);
    fprintf('Overshoot: %.2f%%\n', step_info.Overshoot);
end

function generate_analysis_plots(G, C)
    % Open-loop transfer function
    L = C*G;
    
    % Root locus plot
    figure('Name', 'Root Locus', 'NumberTitle', 'off');
    rlocus(L);
    title('Root Locus of PI-Controlled System');
    grid on;
    
    % Bode plot
    figure('Name', 'Bode Plot', 'NumberTitle', 'off');
    bode(L);
    grid on;
    title('Bode Plot of PI-Controlled System');
    
    % Nyquist plot
    figure('Name', 'Nyquist Plot', 'NumberTitle', 'off');
    nyquist(L);
    grid on;
    title('Nyquist Plot of PI-Controlled System');
    
    % Calculate stability margins
    [Gm, Pm, Wcg, Wcp] = margin(L);
    fprintf('\nStability Margins:\n');
    fprintf('Gain Margin: %.2f dB at %.2f rad/s\n', 20*log10(Gm), Wcg);
    fprintf('Phase Margin: %.2f° at %.2f rad/s\n', Pm, Wcp);
end

% Run main function
main();
