clc; clear; %close all;


rng(1952777514) % seed 'bonita'

%% System Matrices:

A_sys = [0.9843,0.0000,0.0251,0.0000;
         0.0000,0.9892,0.0000,0.0175;
         0.0000,0.0000,0.9747,0.0000;
         0.0000,0.0000,0.0000,0.9823];

B_sys = [0.0478,0.0010;
         0.0005,0.0348;
         0.0000,0.0765;
         0.0554,0.0000];

C_sys = [0.5,0.0,0.5,0.0
         0.0,0.5,0.0,0.5];

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

% --- Matrix Dimensions ---
% Original plant dimensions
n_sys = size(A_sys, 1); % Number of states
p_sys = size(C_sys, 1); % Number of outputs (sensors)
m_sys = size(B_sys, 2); % Number of inputs (actuators)

% --- Dimensions ---
n_total = n_sys + n_aux;
p_total = p_sys + p_aux;

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
        % C_comp(:,:,i) = [C_sys,               zeros(p_sys, n_aux);
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

%% Extended Matrices

    


%%

% --- Attacker Definitions ---
% The attacker assumes perfect knowledge of the first mode
A_att = A_comp(:,:,1);
B_att = B_comp(:,:,i);
C_att = C_comp(:,:,i);
% Function calls to calculate the attacker's controller and observer gains
[Ka1, Ka2] = calculateAttackerController_lqi(A_att, B_att, C_att);
La_set = calculateAttackerObserver(A_att, C_att);

% Malicious reference
% Can be a step input to cause overflow or depletion
ra = 0.8 * ones(p_total, 1); 

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

% --- Aumento do Sistema para Controle Servo (Ogata) ---
n_aug = n_total + p_total; 
A_servo = zeros(n_aug, n_aug, l);
B_servo = zeros(n_aug, m_sys + m_aux, l);

for i = 1:l
    % Matriz A aumentada: [ A  0 ; -C  I ] (Ogata p. 617 / [3, 4])
    A_servo(:,:,i) = [A_comp(:,:,i),            zeros(n_total, p_total);
                      -C_comp(:,:,i),           eye(p_total)];
    % Matriz B aumentada: [ B ; 0 ]
    B_servo(:,:,i) = [B_comp(:,:,i); 
                      zeros(p_total, m_sys + m_aux)];
end

% Matriz de transferência do sistema avaliada em z=1. matriz de Rosenbrock
A_teste = [eye(size(A_comp(:,:,1)))-A_comp(:,:,1),-B_comp(:,:,1);
           C_comp(:,:,1),zeros(size(C_comp(:,:,1),1),size(B_comp(:,:,1),2))];

% Ajuste dos pesos LQR e Riccati para o novo tamanho n_aug [cite: 25, 476]
Q_lqr_aug = eye(n_aug); 
Q_lqr_aug(n_total+1:end, n_total+1:end) = 50 * eye(p_total); % Peso no erro
R_lqr = 0.5*eye(m_sys + m_aux);      % Control weighting matrix
P_riccati = zeros(n_aug, n_aug, l);

for i = 1:l, P_riccati(:,:,i) = eye(n_aug); end

v_int = zeros(p_total, T_sim); % Estado do integrador (erro acumulado)
K_set = zeros(m_sys + m_aux, n_aug, l);

% Inicialização externa para evitar erros de escopo e garantir operação nominal
u_a = zeros(m_sys + m_aux, 1); 
y_a = zeros(p_total, 1);
r_nom = 0.5 * ones(p_total, 1);
r0 = C_comp(:,:,1)*x_real(:,1);    % saída correspondente ao estado inicial
N_ramp = 500;
va_int = zeros(p_total, 1); 

P_matrix_aug = blkdiag(P_matrix,1);


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


% % --- Parameters for the Recursive Computation of K_set (LQR) ---
% Q_lqr = eye(n_total);            % State weighting matrix
%R_lqr = eye(m_sys + m_aux);      % Control weighting matrix
% P_riccati = zeros(n_total, n_total, l); % Riccati matrices for each mode
% for i = 1:l
%     P_riccati(:,:,i) = eye(n_total); % Initialization
% end
% K_set = zeros(m_sys + m_aux, n_total, l); % Control gains

for k = 1:T_sim-1
    i = theta(k); % Identifies the active mode at the current time instant

    % --- Generation of Gaussian Noise (Zero mean) ---
    wk = 1*randn(n_total, 1); % Process disturbance
    vk = 1*randn(p_total, 1); % Measurement noise

    %% Recursive Controller

    % 1. INTEGRATOR UPDATE (Ogata Eq. 8-73)
    % Measurement perceived by the controller (hiding the attack influence ya)
    y_star = (C_comp(:,:,i) * x_real(:,k) + E_comp * randn(p_total,1)) - y_a;
    
    if k <= N_ramp
        alpha = k/N_ramp;
        r = (1-alpha)*r0 + alpha*r_nom;
    else
        r = r_nom;
    end

    if k > 1
        v_int(:,k) = v_int(:,k-1) + (r - y_star); 
    else
        v_int(:,k) = (r - y_star);
    end

    % 2. RECURSIVE GAIN CALCULATION (Coupled Riccati Equations)
    Psi_i = zeros(n_aug, n_aug);
    for j = 1:l
        if i <= size(P_matrix_aug, 1) && j <= size(P_matrix_aug, 2)
            Psi_i = Psi_i + P_riccati(:,:,j) * P_matrix_aug(i, j);
        end
    end

    % Gain calculation for the augmented system
    As = A_servo(:,:,i);
    Bs = B_servo(:,:,i);
    K_set(:,:,i) = -(R_lqr + Bs' * Psi_i * Bs) \ (Bs' * Psi_i * As);
    P_riccati(:,:,i) = As' * (Psi_i - Psi_i * Bs * ((R_lqr + Bs' * Psi_i * Bs) \ (Bs' * Psi_i))) * As + Q_lqr_aug;

    % 3. NOMINAL CONTROL (Augmented state vector [x_hat ; v_int])
    % The control law combines the estimated state and the integrated error
    %u_nominal(:,k) = K_set(:,:,i) * [x_hat(:,k); v_int(:,k)];
    %%

    gamma = 0.02;

    if k == 1
        K_applied = K_set(:,:,i);
    else
        K_applied = (1-gamma)*K_applied + gamma*K_set(:,:,i);
    end
    
    u_nominal(:,k) = K_applied*[x_hat(:,k); v_int(:,k)];
    %%
    
    % STEP A: Attacker Logic
    % The attacker injects ua(k) and tries to compensate with ya(k)
    % In this example, ua starts at k=1000 and is a ramp limited to 0.5
    
    if k >= 1000
        
        ra_ramp = (ra - r_nom) * min((k - 1000) / 1000, 1); 
        % 1. Evolução do Integrador do Atacante
        % erro_a = (ra - y_a_modelo_interno)
        if k > 1
            va_int = va_int + (ra_ramp - (C_att * xa));  
        end

        % 2. Nova Lei de Controle do Atacante (LQI)
        u_a_calc = Ka1 * xa + Ka2 * va_int; 

        % 3. FILTERING: Forces the attack to act ONLY on the real system
        % We keep the first m_sys inputs and zero out the remaining m_aux ones
        u_a = zeros(m_sys + m_aux, 1);
        u_a = u_a_calc; % Ataca ambas as plantas
        %u_a(1:m_sys) = u_a_calc(1:m_sys); % Ataca apenas a planta física
              
        % 4. The attacker generates ya to hide ONLY the effect of this restricted ua
        % Since ua_aux = 0, ya_aux will be zero if the system is decoupled [cite: 66, 68, 191]
        y_a = C_att * xa;
        
        % 4. Internal model state (xa) evolution with the restricted ua
        xa = A_att * xa + B_att * u_a;
        
        % 5. Updating the hacker's estimate (x_hat_a)
        % The hacker intercepts the real measurement and uses their model to estimate the state
        % Note: C_comp(:,:,i) should be used if the output matrix changes with the mode [cite: 61, 240]
        y_real_atual = C_comp(:,:,i) * x_real(:,k) + E_comp * vk; 
        
        % Attacker's internal Kalman Filter [cite: 65, 252, 272]
        x_hat_a = A_att * x_hat_a + B_att * (u_nominal(:,k) + u_a) + ...
                  La_set * (y_real_atual - C_att * x_hat_a);

    else
        % Case without attack: u_a and y_a remain zero (initialized outside)
        y_real_atual = C_comp(:,:,i) * x_real(:,k) + E_comp * vk;
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

title('Masked Outputs (Plant + Auxiliary)', 'Interpreter', 'tex', 'Color', 'w');
xlabel('k', 'Color', 'w');
ylabel('y_{real}', 'Interpreter', 'tex', 'Color', 'w');

% Adjusting the legend to a black background and white text
legend('y_{sys1}', 'y_{sys2}', 'y_{aux1}', 'y_{aux2}', ...
       'Location', 'bestoutside', 'TextColor', 'w', 'EdgeColor', 'w');
%ylim([-0.5 1]);

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
%ylim([-0.5 1]);

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

