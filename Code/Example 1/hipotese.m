clc; clear; close all;

rng(87)
%rng(100)

%% System Matrices:

A_sys = [0.9843,0.0000,0.0251,0.0000;
         0.0000,0.9892,0.0000,0.0175;
         0.0000,0.0000,0.9747,0.0000;
         0.0000,0.0000,0.0000,0.9823];

B_sys = [0.0478,0.0010;
         0.0005,0.0348;
         0.0000,0.0765;
         0.0554,0.0000];

C_sys = [0.5,0.0,0.0,0.0
         0.0,0.5,0.0,0.0];


%% Auxiliary systems:

% number of auxiliary matrices:

n_aux = 2;
% The choice between tau_real_A and tau_im_A must satisfy: 
% n_aux = tau_real_A + 2tau_im_A, based on the number of imaginary
% eigenvalues of A_sys. In this case there are only real eigenvalues
tau_real_A = n_aux;
tau_im_A = 0;
m_aux = 1;
p_aux = 2;
l = 8;
theta_u = 1;
theta_y = 1;

% --- Input Parameters ---
is_stable = false;
attempt = 1;
max_attempts = 500;
tolerance = 1e-6; % Tolerance for strict inequality

% YALMIP settings to use SeDuMi as the solver
ops = sdpsettings('solver', 'sedumi', 'verbose', 0);

while ~is_stable && attempt <= max_attempts
    % 1. Matrix Generation via Truncated Normal Distribution (TND)
    % This function should return the set of l state matrices for the auxiliary system
    [A_coup, B_coup, C_coup, A_aux_set, B_aux, C_aux, ...
     Q_red_y, Q_red_u, Q_exp1_y, Q_exp2_y, Q_exp_u] = ...
     Truncated_Normal_Distribution(A_sys, B_sys, C_sys, tau_real_A, ...
     tau_im_A, n_aux, m_aux, p_aux, theta_u, theta_y, l);

    % 2. Define Decision Variables in YALMIP
    % sdpvar(n, n) automatically creates a symmetric matrix P
    P = sdpvar(n_aux, n_aux); 

    % 3. Define Constraints (LMIs)
    % Condition 1: P must be positive definite (P > 0)
    Constraints = [P >= tolerance * eye(n_aux)];
    
    % Condition 2: Common Lyapunov Function for Arbitrary Switching
    % A_sigma_Aux * P * A_sigma_Aux' - P < 0 must hold for all l modes simultaneously
    for i = 1:l
        Ai = A_aux_set(:,:,i);
        Constraints = [Constraints, Ai * P * Ai' - P <= -tolerance * eye(n_aux)];
    end

    % 4. Solve the Feasibility Problem
    sol = optimize(Constraints, [], ops);

    % 5. Verify the Results
    if sol.problem == 0 % 0 indicates that a feasible solution was found
        is_stable = true;
        fprintf('Success! Stable matrix set found on attempt %d.\n', attempt);
        P_final = value(P); % Extract the numerical value of P
    else
        if mod(attempt, 10) == 0
            fprintf('Attempt %d: No common Lyapunov function found. Retrying...\n', attempt);
        end
        attempt = attempt + 1;
    end
end

if ~is_stable
    error('Could not find a stable matrix set within the maximum number of attempts.');
end



%% Extended Matrices

% --- Matrix Dimensions ---
% Original plant dimensions
n_sys = size(A_sys, 1); % Number of states
p_sys = size(C_sys, 1); % Number of outputs (sensors)
m_sys = size(B_sys, 2); % Number of inputs (actuators)

% Auxiliary system dimensions
n_aux = size(A_aux_set, 1); % Number of auxiliary states
p_aux = size(C_aux, 1);     % Number of auxiliary outputs
m_aux = size(B_aux, 2);     % Number of auxiliary inputs

% --- Initialization of Extended Matrices ---
% The state matrix A changes according to the switching signal sigma (l modes)
A_comp = zeros(n_sys + n_aux, n_sys + n_aux, l);

for i = 1:l
    % Assembly of the extended A matrix for each mode i
    % Structure: [ A_sys    0   ]
    %            [ A_coup   A_i ]
    % A_comp(:,:,i) = [A_sys,               zeros(n_sys, n_aux); 
    %                  zeros(n_aux, n_sys),              A_aux_set(:,:,i)];
    A_comp(:,:,i) = [A_sys,               zeros(n_sys, n_aux); 
                     A_coup,              A_aux_set(:,:,i)];
end

% --- Constant Extended B and C Matrices ---
% Note: In this model, B_aux and C_aux are constant across all modes.

% Extended Input Matrix (B_comp)
% Structure: [ B_sys    0     ]
%            [ B_coup   B_aux ]
B_comp = [B_sys,               zeros(n_sys, m_aux);
          zeros(n_aux, m_sys), B_aux];
% B_comp = [B_sys,               zeros(n_sys, m_aux);
%           B_coup,              B_aux];

% Extended Output Matrix (C_comp)
% Structure: [ C_sys    0     ]
%            [ C_coup   C_aux ]
C_comp = [C_sys,               zeros(p_sys, n_aux);
          zeros(p_aux, n_sys), C_aux];
% C_comp = [C_sys,               zeros(p_sys, n_aux);
%           C_coup,              C_aux];

fprintf('Extended System matrices (A_comp, B_comp, C_comp) successfully built.\n');

% --- Dimensions ---
n_total = n_sys + n_aux;
p_total = p_sys + p_aux;

%%

% --- Attacker Definitions ---
% The attacker assumes perfect knowledge of the first mode
A_att = A_comp(:,:,1);
B_att = B_comp;
C_att = C_comp;

% Function calls to calculate the attacker's controller and observer gains
%[Ka1, Ka2] = calculateAttackerController(A_att, B_att, C_att);
%La_set = calculateAttackerObserver(A_att, C_att);

% Malicious reference
% Can be a step input to cause overflow or depletion
ra = 0.2 * ones(p_total, 1); 

% Initialization of the attacker's internal states
xa = zeros(n_total, 1);     % Covert model state
x_hat_a = zeros(n_total, 1); % Attacker's estimate via their own Kalman Filter

%%

% --- Time Parameters ---
T_sim = 10000; % Simulation horizon

% --- State Initialization ---
x_real = zeros(n_total, T_sim);    % Real state of the extended system
x_hat  = zeros(n_total, T_sim);    % Estimated state by the observer
x_a    = zeros(n_total, T_sim);    % Internal state of the attacker model

% x_a(:,1) = 0.5*ones(n_total,1);
% x_real(:,1) = 0.5*ones(n_total,1);
% x_hat(:,1) = 0.5*ones(n_total,1);

%% Markov Chain Generation

theta_0 = 1;
k_markov_start = 3000;    % Start of Markov chain switching
%k_markov_end = 7000;

% Transition Probability Matrix for modes 1 and 2 only
% P_matrix = [0.77, 0.23; 
%             0.36, 0.64];

P_matrix = [
    0.1238, 0.3142, 0.2419, 0.1978, 0.0516, 0.0515, 0.0192;
    0.2057, 0.1428, 0.1682, 0.0049, 0.2303, 0.1977, 0.0504;
    0.0719, 0.0725, 0.1203, 0.2075, 0.1708, 0.1151, 0.2419;
    0.0507, 0.1061, 0.1331, 0.1656, 0.2852, 0.0725, 0.1868;
    0.1744, 0.0137, 0.1789, 0.0502, 0.0191, 0.2794, 0.2843;
    0.2738, 0.1032, 0.0331, 0.2318, 0.1491, 0.0413, 0.1677;
    0.0106, 0.2804, 0.0798, 0.2043, 0.0961, 0.1603, 0.1685
];
 
% P_matrix = [0.999, 0.001; 
%             0.001, 0.999];

% P_matrix = [0.99, 0.01; 
%             0.01, 0.99];
            
% S is the cumulative probability matrix to facilitate sampling 
S = cumsum(P_matrix, 2);
num_modes = size(P_matrix,2);
theta = zeros(1, T_sim);
theta(1:k_markov_start-1) = theta_0; 

modo_atual = 1; % We start in indice 1 of the probability transition matrix

for k = k_markov_start:T_sim

    theta(k) = modo_atual + 1;
    
    if k < T_sim
        R_k = rand();
        modo_atual = find(R_k <= S(modo_atual, :), 1, 'first');
    end
end

%%
% --- Definition of Inputs (u_sys and u_aux) ---
% Original plant: step and sine wave. Auxiliary: step.
u_nominal = zeros(m_sys + m_aux, T_sim);

% --- Noise Parameters ---
a_noise = 1e-4;
D_comp = a_noise * 0.5 * eye(n_total); % Process noise coupling
E_comp = a_noise * 0.5 * eye(p_total); % Measurement noise coupling

Q_comp = D_comp * D_comp';
R_comp = E_comp * E_comp';
P_cov = 0.1 * eye(n_total);     % P_0|-1 (Initial covariance matrix)

%%
% --- Parâmetros para o Cálculo Recursivo do K_set (LQR) ---
Q_lqr = eye(n_total);            % Peso nos estados [cite: 109]
R_lqr = eye(m_sys + m_aux);      % Peso no controle [cite: 109]
P_riccati = zeros(n_total, n_total, l); % Matrizes de Riccati para cada modo
for i = 1:l
    P_riccati(:,:,i) = eye(n_total); % Inicialização
end
K_set = zeros(m_sys + m_aux, n_total, l); % Ganhos de controle

%%
theta_2 = 1;

for k = 1:T_sim-1
    %%
    i = theta(k); % Identifies the active mode at the current time instant

    if k >= k_markov_start
        a = rand;
        b = rand;
        P_matrix = [a, 1-a; 
                    1-b, b];
        P_save(:,:,k) = P_matrix; 
        S = cumsum(P_matrix, 2);
        R_k = rand();
        theta_2 = find(R_k <= S(theta_2, :), 1, 'first');
    end
    i_2 = theta_2;

    %%

    % --- Generation of Gaussian Noise (Zero mean) ---
    wk = 1*randn(n_total, 1); % Process disturbance
    vk = 1*randn(p_total, 1); % Measurement noise

    %% Recursive Controller

    % 1. RECURSIVE CALCULATION OF THE K_set GAIN (Coupled Riccati Equations)

    % Calculates Psi for the current mode (Weighted sum by transition probabilities)
    Psi_i = zeros(n_total, n_total);
    for j = 1:l
        % If your P_matrix is 2x2 for auxiliary modes, adjust the indices
        % Here we use i and j to represent the probability p_ij
        if i_2 <= size(P_matrix, 1) && j <= size(P_matrix, 2)
            Psi_i = Psi_i + P_riccati(:,:,j) * P_matrix(i_2, j);
        end
    end

    % Updates the K_set Gain for the current mode i
    % Equation: K_i = -(R + B'*Psi*B)^-1 * B'*Psi*A
    Ai = A_comp(:,:,i_2);
    Bi = B_comp;
    K_set(:,:,i_2) = -(R_lqr + Bi' * Psi_i * Bi) \ (Bi' * Psi_i * Ai);

    % Updates the Riccati Matrix for the next step k+1
    P_riccati(:,:,i_2) = Ai' * (Psi_i - Psi_i * Bi * ((R_lqr + Bi' * Psi_i * Bi) \ (Bi' * Psi_i))) * Ai + Q_lqr;

    % 2. NOMINAL CONTROL (Recursive Closed-Loop)
    % u_nominal = K * corrected x_hat (using the gain calculated above)
    u_nominal(:,k) = K_set(:,:,i_2) * x_hat(:,k);

    %%
    
    % STEP A: Attacker Logic
    % The attacker injects ua(k) and tries to compensate with ya(k)
    % In this example, ua starts at k=1000 and is a ramp limited to 0.5
    u_a = zeros(m_sys + m_aux, 1);
    y_a = zeros(p_sys + p_aux, 1);

    [La_set,~, ~, ~] = dlqe(A_comp(:,:,i_2), eye(n_total), C_att, Q_comp, R_comp);
    
    if k >=1000 % && k <= 7000
        % 1. Calculates the attack input signal (ua) using the function gains
        % u_a(k) = Ka1 * x_estimated_by_attacker + Ka2 * ra [3]
        Ka1 = K_set(:,:,i_2);
        ClosedLoop_A = A_comp(:,:,i_2) + B_att*Ka1;
        M = C_att * inv(eye(n_total) - ClosedLoop_A) * B_att;
        Ka2 = pinv(M); 
        u_a = Ka1 * x_hat_a + Ka2 * ra;
        %u_a = zeros(3,1);
        
        % 2. The attacker simulates the effect of the attack on their internal model to generate ya
        % ya(k) is the signal that will be subtracted from the real output to hide the attack
        y_a = C_att * xa;
        
        % 3. Updates the internal model state of the attacker (xa)
        xa = A_comp(:,:,i_2) * xa + B_att * u_a;
       
        % 4. The attacker updates their estimate of the real state (x_hat_a)
        % (Simulates an internal Kalman Filter of the hacker)
        % Note: y_real_k is obtained from the previous step or from the plant
        y_real_atual = C_comp * x_real(:,k) + E_comp * vk; % Reading intercepted by the hacker
        x_hat_a = A_comp(:,:,i_2) * x_hat_a + B_att * (u_nominal(:,k) + u_a) + ...
                  La_set * (y_real_atual - C_att * x_hat_a);

    else
        y_real_atual = C_comp * x_real(:,k) + E_comp * vk; % Reading intercepted by the hacker
        x_hat_a = A_comp(:,:,i_2) * x_hat_a + B_att * (u_nominal(:,k) + u_a) + ...
                  La_set * (y_real_atual - C_att * x_hat_a);
    end

    % STEP B: Real System Dynamics
    % The physical system receives the nominal control + the attack
    u_total = u_nominal(:,k) + u_a;
    x_real(:, k+1) = A_comp(:,:,i)*x_real(:,k) + B_comp*u_total + D_comp*wk;

    % STEP C: Compromised Measurements
    % What the controller and observer receive is y(k) subtracted by ya(k)
    y_compromised = (C_comp * x_real(:,k) + E_comp * vk) - y_a;

    %% Recursive Kalman Filter

    % --- STEP D: Kalman Filter for MJLS (Algorithm 2.2) ---
    % D.1 Filtering Stage

    S_k = R_comp + C_comp * P_cov * C_comp'; 
    K_kalman = (P_cov * C_comp') / S_k; 

    % D.2 Calculation of Residual d_aux,k BEFORE the k+1 update (Eq. 14 / 4.10)
    % d_aux = y_star_auxiliary - y_hat_auxiliary (Innovation of the 2nd line)
    y_hat = C_comp * x_hat(:,k);

    % D.3 Update and Prediction for k+1
    x_hat_filter = x_hat(:,k) + K_kalman * (y_compromised - C_comp * x_hat(:,k));
    P_filter = P_cov - K_kalman * C_comp * P_cov;

    x_hat(:, k+1) = A_comp(:,:,i_2) * x_hat_filter + B_comp * u_nominal(:,k);
    P_cov = Q_comp + A_comp(:,:,i_2) * P_filter * A_comp(:,:,i_2)';

    %%
    
    % STEP E: Residual Calculation
    % r_aux is the part of the residual related to the auxiliary system
    % residuos(:,k) = abs(y_comprometido - y_hat);
    residuals(:,k) = y_compromised - y_hat;

    y_plot(:,k) = y_a;
    y_c_plot(:,k) = y_compromised;
    y_hat_plot(:,k) = y_hat;
end

% Selects only the auxiliary system outputs for detection
res_aux = residuals(p_sys+1 : end, :);

% Norm calculation
norma_res = sqrt(sum(res_aux.^2, 1));

J_th = 1.1 * max(norma_res(1:999));

% Detection Logic
alarm = norma_res > J_th; % J_th is the threshold defined without attack

%% PLOT

% Figure 1.1

% --- Data Preparation ---
tempo = 0:size(residuals, 2)-1; 
res_planta = residuals(1:p_sys, :);    % r_Sys (Stealthy)
res_aux    = residuals(p_sys+1:end, :); % r_Aux (Detectable)

% --- Figure Creation ---
figure('Color', 'k', 'Name', 'Detection of Covert Attack');

% Plot of the residuals 
subplot(2, 1, 1);
hold on; grid on;
p1 = plot(tempo, res_planta(1,:), 'Color', [0 0.447 0.741], 'LineWidth', 1);
p2 = plot(tempo, res_planta(2,:), 'Color', [1 0.647 0], 'LineWidth', 1);

% --- Use of xline for Time Events (X Axis) ---
% k=1000: Attack start
xline(1000, '--k', 'Attack Start', 'LabelVerticalAlignment', 'top');
xline(3000, ':r', 'Mode Switch', 'LabelVerticalAlignment', 'middle');

% --- Customization Subplot 1 ---
ylabel('Residual r_{Sys}');
xlim([0, T_sim]);
ylim([-0.01 0.01]);
legend([p1, p2], {'r_{Sys}_1', 'r_{Sys}_2'}, 'Location', 'best');
hold off;

% Figure 1.2

subplot(2, 1, 2);
hold on; grid on;
p3 = plot(tempo, res_aux(1,:), 'Color', [1 1 0], 'LineWidth', 1.2); 
p4 = plot(tempo, res_aux(2,:), 'Color', [0.54 0.17 0.89], 'LineWidth', 1.2);

% --- Use of xline for Time Events (X Axis) ---
% k=1000: Attack start
xline(1000, '--k', 'Attack Start', 'LabelVerticalAlignment', 'top');
xline(3000, ':r', 'Mode Switch', 'LabelVerticalAlignment', 'middle');

% --- Use of yline for the Detection Threshold (Y Axis) ---
%J_th: Decision threshold
yline(J_th, '-.g', 'Threshold J_{th}', 'LineWidth', 1.5);
yline(-J_th, '-.g', 'LineWidth', 1.5);

% --- Customization Subplot 2 ---
xlabel('Time in s');
ylabel('Residual r_{Aux}');
xlim([0, T_sim]);
ylim([-0.01 0.01]);

legend([p1,p2,p3,p4], {'r_{Sys}_1', 'r_{Sys}_2', 'r_{Aux}_1', 'r_{Aux}_2'}, 'Location', 'best');

hold off;

% Figure 2.1

% Creating the figure with a black outer background
figure('Name', 'Output Analysis: Real vs Filter', 'Color', 'k');

% --- Subplot 1: Real System Outputs (Plant + Auxiliary) ---
subplot(2, 1, 1);
plot(y_plot(1:4, 1:T_sim-1)', 'LineWidth', 1.5); 
grid on;

% Adjusting axis colors and inner background to black
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]); 

title('Real System Outputs (Plant + Auxiliary)', 'Interpreter', 'tex', 'Color', 'w');
xlabel('k', 'Color', 'w');
ylabel('y_{real}', 'Interpreter', 'tex', 'Color', 'w');

% Adjusting the legend to a black background and white text
legend('y_{sys1}', 'y_{sys2}', 'y_{aux1}', 'y_{aux2}', ...
       'Location', 'bestoutside', 'TextColor', 'w', 'EdgeColor', 'w');
ylim([-0.5 0.5]);

% Figure 2.2
% --- Subplot 2: Kalman Filter Estimates ---
subplot(2, 1, 2);
plot(y_hat_plot(1:4, 1:T_sim-1)', 'LineWidth', 1.5);
grid on;

% Adjusting axis colors and inner background to black
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);

title('Kalman Filter Estimates ($\hat{y}$)', 'Interpreter', 'latex', 'Color', 'w');
xlabel('k', 'Color', 'w');
ylabel('$\hat{y}$', 'Interpreter', 'latex', 'Color', 'w');

% Adjusting the legend to a black background and white text
legend('$\hat{y}_{sys1}$', '$\hat{y}_{sys2}$', '$\hat{y}_{aux1}$', '$\hat{y}_{aux2}$', ...
       'Location', 'bestoutside', 'Interpreter', 'latex', 'TextColor', 'w', 'EdgeColor', 'w');
ylim([-0.5 0.5]);

% Figure 3
% --- Subplot 3: Masked Output ---
figure('Name', 'Output Analysis: Masked', 'Color', 'k');

plot(y_c_plot(:, 1:T_sim-1)', 'LineWidth', 1.5);
grid on;

% Adjusting axis colors and inner background to black
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);

title('Compromised output ($y_{sys2}$)', 'Interpreter', 'latex', 'Color', 'w');
xlabel('k', 'Color', 'w');
ylabel('$\hat{y}$', 'Interpreter', 'latex', 'Color', 'w');

% Adjusting the legend to a black background and white text
legend('y_{sys1}', 'y_{sys2}', 'y_{aux1}', 'y_{aux2}', ...
       'Location', 'bestoutside', 'Interpreter', 'latex', 'TextColor', 'w', 'EdgeColor', 'w');
ylim([-0.5 0.5]);