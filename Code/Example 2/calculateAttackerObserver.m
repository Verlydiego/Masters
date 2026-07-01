function La = calculateAttackerObserver(A, C)
    % calculateAttackerObserver - Calculates the Kalman Filter gain of the attacker
    %
    % Inputs:
    %   A, C - System matrices (from the attacker's perspective)
    %
    % Output:
    %   La   - Observer gain matrix (Kalman Filter)

    n = size(A, 1);
    p = size(C, 1);

    % The attacker defines their own noise estimates
    % (Generally eye(n) and eye(p) if they do not have specific details)
    Qa_noise = eye(n); 
    Ra_noise = eye(p);

    % Calculation of the steady-state Kalman Filter gain
    % Note: dlqe expects the model x(k+1) = Ax(k) + Bu(k) + Gw(k)
    % y(k) = Cx(k) + v(k)
    % The returned gain (L) is such that x_hat = A*x_hat + B*u + L(y - C*x_hat)
    [La,~, ~, ~] = dlqe(A, eye(n), C, Qa_noise, Ra_noise);
    fprintf('Attacker observer gain (La) calculated.\n');
end