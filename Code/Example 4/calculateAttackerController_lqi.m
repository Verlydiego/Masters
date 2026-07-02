function [Ka1, Ka2] = calculateAttackerController(A, B, C)
    % calculateAttackerController - Calculates gains for a covert attacker
    % logic based on reference tracking.
    %
    % Inputs:
    %   A, B, C - State-space matrices of the system (original or extended)
    %
    % Outputs:
    %   Ka1 - State feedback gain matrix
    %   Ka2 - Feedforward reference tracking gain matrix

    % 1. Get system dimensions
    n = size(A, 1);
    m = size(B, 2);
    p = size(C, 1);

    % 2. Create the State-Space Object
    % O LQI exige um modelo SS. Usamos Ts=1 conforme Example 1 [cite: 25, 122]
    sys_att = ss(A, B, C, 0, 1);
    
    % 3. Design the LQI Weights
    % A matriz Q deve ter dimensão (n + p) para incluir o estado integral [cite: 370]
    Qa_aug = eye(n + p); 
    % Aumentamos o peso nas p saídas integradas para garantir rastreamento rápido
    Qa_aug(n+1:end, n+1:end) = 100 * eye(p); 

    Ra = eye(m); % Peso no esforço de controle
    
     % 4. Calculate LQI Gain
    % A função lqi retorna um ganho K = [Kx Ki] tal que u = -Kx*x - Ki*vi [cite: 370]
    K_total = lqi(sys_att, Qa_aug, Ra);
    
    % 5. Splitting Gains
    % Ka1 (Proporcional): Ganho aplicado aos estados estimados xa
    Ka1 = -K_total(:, 1:n); 
    
    % Ka2 (Integral): Ganho aplicado ao novo estado integral do atacante (va)
    Ka2 = -K_total(:, n+1:end);

    fprintf('Attacker LQI gains Ka1 and Ka2 successfully calculated.\n');
end