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
    
    % 2. Design the Feedback Gain (Ka1)
    % Typically designed using LQR to ensure stability of the attacker's 
    % internal tracking error dynamics.
    Qa = eye(n); % State weighting matrix for the attacker
    Ra = eye(m); % Input weighting matrix for the attacker
    
    % The 'dlqr' function provides the optimal gain for discrete-time systems
    Ka1_lqr = dlqr(A, B, Qa, Ra);
    
    % Adjusting sign to follow the law u_a = Ka1*xa + Ka2*ra
    Ka1 = -Ka1_lqr; 

    % 3. Design the Feedforward Gain (Ka2)
    % Objective: Ensure zero steady-state error (y = ra).
    % For discrete systems, the gain is derived from the closed-loop 
    % transfer function: y_ss = C * (I - (A + B*Ka1))^-1 * B * Ka2 * ra
    
    % Closed-loop term (A + B*Ka1)
    ClosedLoop_A = A + B*Ka1;

    % Steady-state gain matrix calculation
    % Note: Use pseudo-inverse (pinv) if the system is not square
    
    M = C * inv(eye(n) - ClosedLoop_A) * B;
    Ka2 = pinv(M); 

    fprintf('Attacker gains Ka1 and Ka2 successfully calculated.\n');
end