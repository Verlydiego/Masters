clear; clc; close all;

% --- Parâmetros de Monte Carlo ---

% --- Definição do Intervalo de Busca ---
seed_inicial = 1;
seed_final = 50; 
N_sim = seed_final - seed_inicial + 1;

T_sim = 10000; % Horizonte de tempo
n_pontos = T_sim - 1;
p_sys = 2; % Número de saídas da planta real [cite: 79, 171]
canal = 1; % Canal de saída para visualização (ex: y1)

% Pré-alocação para armazenar todas as trajetórias (Memória Eficiente)
Y_Comp_all = zeros(N_sim, T_sim-1);
Y_Real_all = zeros(N_sim, T_sim-1);
Y_Hat_all  = zeros(N_sim, T_sim-1);
Norm_Res_Aux_all = zeros(N_sim, T_sim-1);
J_th_vec = zeros(N_sim, 1);

fprintf('Iniciando Monte Carlo com %d rodadas...\n', N_sim);

idx = 1; % Índice para preenchimento das matrizes
for s = seed_inicial:seed_final
    % Monitoramento do Progresso [cite: 12]
    fprintf('Processando Seed %d (%d de %d)...\n', s, idx, N_sim);
    
    % Chamada da sua função [cite: 14, 51]
    [t, y_comp, y_real, y_hat, J_th, res_aux, res_planta] = main_seed(s);
    
    % --- SEGURANÇA: Filtro de Amplitude (Limite 10^5) ---
    % Substitui valores divergentes por NaN para não distorcer o plot [cite: 224]
    % y_comp(abs(y_comp) > 10e6) = NaN;
    % y_real(abs(y_real) > 10e6) = NaN;
    % y_hat(abs(y_hat) > 10e6)   = NaN;
    
    % Armazenamento dos Sinais (9999 colunas)
    Y_Real_Multi(:,:,idx) = y_real(:, 1:n_pontos);
    Y_Comp_all(idx, :) = y_comp(canal, :);
    Y_Hat_all(idx, :)  = y_hat(canal, :);
    
    % Norma do Resíduo Auxiliar (Vetor t completo com T_sim pontos)
    Norm_Res_Aux_all(idx, :) = sqrt(sum(res_aux.^2, 1));
    J_th_vec(idx) = J_th;
    
    idx = idx + 1;
end


% --- Cálculo das Médias (Valor Esperado por Canal) [cite: 461] ---
% Média calculada ao longo da 3ª dimensão (simulações)
mean_y_all = mean(Y_Real_Multi, 3, 'omitnan'); 
mean_norm_res = mean(Norm_Res_Aux_all, 1);
mean_J_th = mean(J_th_vec);

% --- FIGURA 3: SAÍDAS REAIS DO SISTEMA (ESTILO NUVEM + MÉDIA) ---
figure('Color', 'k', 'Name', 'Real System Outputs: Monte Carlo Cloud');
hold on; grid on;

% 1. PLOT DA NUVEM (Trajetórias Individuais em Cinza Claro) [cite: 416, 488]
% Plotamos todas as simulações de uma vez com transparência (0.05 a 0.1)
% O 'Color' [0.8 0.8 0.8, 0.05] define cinza claro com 5% de opacidade.
% squeeze(Y_Real_Multi(canal,:,:))' plota todas as rodadas para um canal específico.

for c = 1:4 % Loop pelos 4 canais para criar a nuvem completa
    plot(t(1:n_pontos), squeeze(Y_Real_Multi(c,:,:))', ...
         'Color', [0.8 0.8 0.8, 0.05], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end

% 2. PLOT DAS MÉDIAS (As "Melhores" Linhas, mais Escuras e Grossas) [cite: 488, 494]
% Plotamos as médias DEPOIS da nuvem para que fiquem por cima de tudo.
p1 = plot(t(1:n_pontos), mean_y_all(1,:), 'g', 'LineWidth', 2.5);   % y_sys,1 (Verde)
p2 = plot(t(1:n_pontos), mean_y_all(2,:), 'c', 'LineWidth', 2.5);   % y_sys,2 (Ciano)
p3 = plot(t(1:n_pontos), mean_y_all(3,:), 'm', 'LineWidth', 2);     % y_aux,1 (Magenta)
p4 = plot(t(1:n_pontos), mean_y_all(4,:), 'y', 'LineWidth', 2);     % y_aux,2 (Amarelo)

% 3. MARCAÇÕES DE EVENTOS (Ataque e Salto) [cite: 58, 152]
xline(1000, '--w', 'Attack Start', 'Color', 'w', 'FontSize', 10);
xline(3000, ':r', 'Mode Switch', 'Color', 'r', 'FontSize', 10);

% 4. ESTÉTICA DO GRÁFICO (GCA)
set(gca,...
    'Color', 'k', ...
    'XColor', 'w', 'YColor', 'w', ...
    'GridColor', [0.5 0.5 0.5], 'GridAlpha', 0.3);

title('Real System Outputs (Monte Carlo Analysis)', 'Color', 'w', 'FontSize', 14);
ylabel('y_{real}', 'Color', 'w');
xlabel('Time (k)', 'Color', 'w');

% 5. LEGENDA (Apenas para as médias)
legend([p1, p2, p3, p4], {'y_{sys,1}', 'y_{sys,2}', 'y_{aux,1}', 'y_{aux,2}'}, ...
       'Location', 'bestoutside', 'TextColor', 'w', 'EdgeColor', 'w', 'Color', 'k');

xlim([0 T_sim]);
%ylim([-5 5]); % Limita o zoom para focar no comportamento estável (Limite 50 ignora explosões) [cite: 224]
hold off;