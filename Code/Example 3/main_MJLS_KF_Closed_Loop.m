clc; clear; %close all;

rng(1952777514)

%% Parameters

% System Matrices:

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

% Transition Probability Matrix for modes 1 and 2 only
P_matrix = [0.77, 0.23; 
            0.36, 0.64];


%% Auxiliary systems:

% number of auxiliary matrices:

n_aux = 4;
% The choice between tau_real_A and tau_im_A must satisfy: 
% n_aux = tau_real_A + 2tau_im_A, based on the number of imaginary
% eigenvalues of A_sys. In this case there are only real eigenvalues
tau_real_A = n_aux;
tau_im_A = 0;
% m_aux >= p_aux;
m_aux = 2;
p_aux = 2;
l = 3; % size(P_matrix) + 1
theta_u = 1;
theta_y = 1;

% --- Parâmetros de Estabilidade ---
is_stable = false;
attempt = 1;
max_attempts = 1000;
tolerance = 1e-6; % Tolerance for strict inequality

% YALMIP settings to use SeDuMi as the solver
ops = sdpsettings('solver', 'sedumi', 'verbose', 0);

% --- Matrix Dimensions ---
% Original plant dimensions
n_sys = size(A_sys, 1); % Number of states
p_sys = size(C_sys, 1); % Number of outputs (sensors)
m_sys = size(B_sys, 2); % Number of inputs (actuators)


while ~is_stable && attempt <= max_attempts
    % 1. Matrix Generation via Truncated Normal Distribution (TND)
    % This function should return the set of l state matrices for the auxiliary system
    for i = 1:l
        [A_coup(:,:,i), B_coup(:,:,i), C_coup(:,:,i), A_aux(:,:,i), B_aux(:,:,i), C_aux(:,:,i), ...
         Q_red_y, Q_red_u, Q_exp1_y, Q_exp2_y, Q_exp_u] = ...
         Truncated_Normal_Distribution(A_sys, B_sys, C_sys, tau_real_A, ...
         tau_im_A, n_aux, m_aux, p_aux, theta_u, theta_y);
    end

    n_total = n_sys + n_aux;
    % Auxiliary system dimensions
    n_aux = size(A_aux(:,:,1), 1); % Number of auxiliary states
    p_aux = size(C_aux(:,:,1), 1);     % Number of auxiliary outputs
    m_aux = size(B_aux(:,:,1), 2);     % Number of auxiliary inputs

    % Montagem temporária de A_comp para o teste
    A_temp = zeros(n_total, n_total, l);

    for i = 1:l
        A_temp(:,:,i) = [A_sys,               zeros(n_sys, n_aux); 
                         A_coup(:,:,i),              A_aux(:,:,i)];

        % --- Constant Extended B and C Matrices ---
        % Note: In this model, B_aux and C_aux are constant across all modes.
            
        % Extended Input Matrix (B_comp)
        % Structure: [ B_sys    0     ]
        %            [ B_coup   B_aux ]
        B_comp(:,:,i) = [B_sys,               zeros(n_sys, m_aux);
                         zeros(n_aux, m_sys), B_aux(:,:,i)];
        % B_comp(:,:,i) = [B_sys,               zeros(n_sys, m_aux);
        %                  B_coup(:,:,i),              B_aux(:,:,i)];
            
        % Extended Output Matrix (C_comp)
        % Structure: [ C_sys    0     ]
        %            [ C_coup   C_aux ]
        C_comp(:,:,i) = [C_sys,               zeros(p_sys, n_aux);
                         zeros(p_aux, n_sys), C_aux(:,:,i)];
        % C_comp = [C_sys,               zeros(p_sys, n_aux);
        %           C_coup(:,:,i),              C_aux(:,:,i)];
    end

    % 2. CONSTRUÇÃO DA MATRIZ AUMENTADA A1 (Fonte: Eq. 3.12d [cite: 386])
    % N = diag(A1 ⊗ A1, A2 ⊗ A2, ..., Al ⊗ Al)
    N_blocks = cell(1, l);
    for i = 2:l
        N_blocks{i} = kron(A_temp(:,:,i), A_temp(:,:,i));
    end
    N_matrix = blkdiag(N_blocks{:});
    
    % C = (P_matrix' ⊗ I_{n_total^2})
    C_matrix = kron(P_matrix', eye(n_total^2));
    
    % Matriz de momentos A1 = C * N
    A1 = C_matrix * N_matrix;

    % 3. TESTE DO RAIO ESPECTRAL
    raio_espectral = max(abs(eig(A1)));
    
    if raio_espectral < 1
        is_stable = true;
        A_comp = A_temp; % Salva o conjunto estável
        fprintf('Estabilidade MSS garantida na tentativa %d (Raio: %.4f)\n', attempt, raio_espectral);
    else
        attempt = attempt + 1;
    end

end

if ~is_stable
    error('Não foi possível encontrar um conjunto MSS estável após %d tentativas.', max_attempts);
end
    
fprintf('Extended System matrices (A_comp, B_comp, C_comp) successfully built.\n');
    
% --- Dimensions ---
n_total = n_sys + n_aux;
p_total = p_sys + p_aux;

%%

% --- Attacker Definitions ---
% The attacker assumes perfect knowledge of the first mode
A_att = A_comp(:,:,1);
B_att = B_comp(:,:,1);
C_att = C_comp(:,:,1);
% Function calls to calculate the attacker's controller and observer gains
[Ka1, Ka2] = calculateAttackerController(A_att, B_att, C_att);
La_set = calculateAttackerObserver(A_att, C_att);

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

% --- Parameters for the Recursive Computation of K_set (LQR) ---
Q_lqr = eye(n_total);            % State weighting matrix
R_lqr = eye(m_sys + m_aux);      % Control weighting matrix
P_riccati = zeros(n_total, n_total, l); % Riccati matrices for each mode
for i = 1:l
    P_riccati(:,:,i) = eye(n_total); % Initialization
end
K_set = zeros(m_sys + m_aux, n_total, l); % Control gains


for k = 1:T_sim-1
    i = theta(k); % Identifies the active mode at the current time instant

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
        if i <= size(P_matrix, 1) && j <= size(P_matrix, 2)
            Psi_i = Psi_i + P_riccati(:,:,j) * P_matrix(i, j);
        end
    end

    % Updates the K_set Gain for the current mode i
    % Equation: K_i = -(R + B'*Psi*B)^-1 * B'*Psi*A
    Ai = A_comp(:,:,i);
    Bi = B_comp(:,:,i);
    K_set(:,:,i) = -(R_lqr + Bi' * Psi_i * Bi) \ (Bi' * Psi_i * Ai);

    % Updates the Riccati Matrix for the next step k+1
    P_riccati(:,:,i) = Ai' * (Psi_i - Psi_i * Bi * ((R_lqr + Bi' * Psi_i * Bi) \ (Bi' * Psi_i))) * Ai + Q_lqr;

    % 2. NOMINAL CONTROL (Recursive Closed-Loop)
    % u_nominal = K * corrected x_hat (using the gain calculated above)
    u_nominal(:,k) = K_set(:,:,i) * x_hat(:,k);

    %%
    
    % STEP A: Attacker Logic
    % The attacker injects ua(k) and tries to compensate with ya(k)
    % In this example, ua starts at k=1000 and is a ramp limited to 0.5
    u_a = zeros(m_sys + m_aux, 1);
    y_a = zeros(p_sys + p_aux, 1);
    
    if k >=1000 % && k <= 7000

        %ra_ramp = ra * min((k - 1000) / 1000, 1);

        % 1. Calculates the attack input signal (ua) using the function gains
        u_a_calc = Ka1 * x_hat_a + Ka2 * ra;
        % u_a_calc = Ka1 * x_hat_a + Ka2 * ra_ramp;
        u_a = u_a_calc; % Ataca ambas as plantas
        %u_a(1:m_sys) = u_a_calc(1:m_sys); % Ataca apenas a planta física
        
        
        % 2. The attacker simulates the effect of the attack on their internal model to generate ya
        % ya(k) is the signal that will be subtracted from the real output to hide the attack
        y_a = C_att * xa;
        
        % 3. Updates the internal model state of the attacker (xa)
        xa = A_att * xa + B_att * u_a;
        
        % 4. The attacker updates their estimate of the real state (x_hat_a)
        % (Simulates an internal Kalman Filter of the hacker)
        % Note: y_real_k is obtained from the previous step or from the plant
        y_real_atual = C_comp(:,:,i) * x_real(:,k) + E_comp * vk; % Reading intercepted by the hacker
        x_hat_a = A_att * x_hat_a + B_att * (u_nominal(:,k) + u_a) + ...
                  La_set * (y_real_atual - C_att * x_hat_a);

    else
        y_real_atual = C_comp(:,:,i) * x_real(:,k) + E_comp * vk; % Reading intercepted by the hacker
        x_hat_a = A_att * x_hat_a + B_att * (u_nominal(:,k) + u_a) + ...
                  La_set * (y_real_atual - C_att * x_hat_a);
    end

    % STEP B: Real System Dynamics
    % The physical system receives the nominal control + the attack
    u_total = u_nominal(:,k) + u_a;
    x_real(:, k+1) = A_comp(:,:,i)*x_real(:,k) + B_comp(:,:,i)*u_total + D_comp*wk;

    % STEP C: Compromised Measurements
    % What the controller and observer receive is y(k) subtracted by ya(k)
    y_compromised = (C_comp(:,:,i) * x_real(:,k) + E_comp * vk) - y_a;

    %% Recursive Kalman Filter

    % --- STEP D: Kalman Filter for MJLS (Algorithm 2.2) ---
    % D.1 Filtering Stage

    S_k = R_comp + C_comp(:,:,i) * P_cov * C_comp(:,:,i)'; 
    K_kalman = (P_cov * C_comp(:,:,i)') / S_k; 

    % D.2 Calculation of Residual d_aux,k BEFORE the k+1 update (Eq. 14 / 4.10)
    % d_aux = y_star_auxiliary - y_hat_auxiliary (Innovation of the 2nd line)
    y_hat = C_comp(:,:,i) * x_hat(:,k);

    % D.3 Update and Prediction for k+1
    x_hat_filter = x_hat(:,k) + K_kalman * (y_compromised - C_comp(:,:,i) * x_hat(:,k));
    P_filter = P_cov - K_kalman * C_comp(:,:,i) * P_cov;

    x_hat(:, k+1) = A_comp(:,:,i) * x_hat_filter + B_comp(:,:,i) * u_nominal(:,k);
    P_cov = Q_comp + A_comp(:,:,i) * P_filter * A_comp(:,:,i)';

    %%
    
    % STEP E: Residual Calculation
    % r_aux is the part of the residual related to the auxiliary system
    % residuos(:,k) = abs(y_comprometido - y_hat);
    residuals(:,k) = y_compromised - y_hat;

    y_real = C_comp(:,:,i)*x_real(:,k) + E_comp*vk;

    y_plot(:,k) = y_compromised;
    y_plot_2(:,k) = y_real;
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
figure('Color','k','Name','System Outputs');

hold on; grid on;

plot(y_plot_2(1,1:T_sim-1),'LineWidth',1.5);
plot(y_plot_2(2,1:T_sim-1),'LineWidth',1.5);
plot(y_plot_2(3,1:T_sim-1),'LineWidth',1.5);
plot(y_plot_2(4,1:T_sim-1),'LineWidth',1.5);

xline(1000,'--k','Attack Start','LabelVerticalAlignment','top');
xline(3000,':r','Mode Switch','LabelVerticalAlignment','middle');

set(gca,...
    'Color','k',...
    'XColor','w',...
    'YColor','w',...
    'GridColor',[0.5 0.5 0.5]);

title('Real System Outputs','Color','w');
ylabel('y_{real}','Color','w');

legend({'y_{sys,1}','y_{sys,2}','y_{aux,1}','y_{aux,2}'},...
       'Location','bestoutside',...
       'TextColor','w',...
       'EdgeColor','w');

xlim([0 T_sim]);

hold off;
