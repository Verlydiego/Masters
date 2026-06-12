function [A_aux,B_aux,C_aux] = Truncated_Normal_Distribution(A,B,C)
    %% Auxiliary matrix B:
    
    % TND parameters:
    
    a_B = min(B(:));
    b_B = max(B(:));
    gamma_B = var(B(:));
    mu_B = mean(B(:));
    
    % Truncated Normal distribution for B:
    
    pd = truncate(makedist('Normal','mu',mu_B,'sigma',sqrt(gamma_B)),a_B,b_B);
    
    % Generate the complementary matrix B:
    
    B_aux = random(pd,size(B));
    
    %% Complementary matrix C:
    
    % TND parameters:
    
    a_C = min(C(:));
    b_C = max(C(:));
    gamma_C = var(C(:));
    mu_C = mean(C(:));
    
    % Truncated Normal distribution for B:
    
    pd_C = truncate(makedist('Normal','mu',mu_C,'sigma',sqrt(gamma_C)),a_C,b_C);
    
    % Generate the complementary matrix B:
    
    C_aux = random(pd_C,size(C));
    
    %% Complementary matrix A:
    
    % poles:
    
    lambda_A = eig(A);
    
    % Real poles:
    
    lambda_real_A = lambda_A( abs(imag(lambda_A)) < 1e-10 & real(lambda_A) >= 0 );
    
    % Complex poles (only positive imaginary part):
    
    lambda_im_A = lambda_A( imag(lambda_A) > 0 );
    
    % Radii and angles:
    
    lambda_rad_A = abs(lambda_im_A);
    lambda_ang_A = angle(lambda_im_A);
    
    % Generate real poles:
    
    Delta_real_A = [];
    
    if ~isempty(lambda_real_A)

        a_real_A = min(lambda_real_A);
        b_real_A = max(lambda_real_A);
    
        mu_real_A = mean(lambda_real_A);
        gamma_real_A = var(lambda_real_A);

        if length(lambda_real_A) == 1

            Delta_real_A = lambda_real_A;
        
        else
    
            pd_real_A = truncate(makedist('Normal','mu',mu_real_A,'sigma',sqrt(gamma_real_A)),a_real_A,b_real_A);
    
            tau_real_A = length(lambda_real_A);
    
            Delta_real_A = random(pd_real_A, tau_real_A,1);

        end
    end
    
    % Generate complex poles:
    
    Delta_im_A = [];

    tau_im_A = length(lambda_im_A);
    
    if ~isempty(lambda_im_A)
        
        % For the radii:
        
        if length(lambda_rad_A) == 1 

            r = lambda_rad_A;

        else

            a_rad_A = min(lambda_rad_A);
            b_rad_A = max(lambda_rad_A);
        
            mu_rad_A = mean(lambda_rad_A);
            gamma_rad_A = var(lambda_rad_A);
        
            pd_rad_A = truncate(makedist('Normal','mu',mu_rad_A,'sigma',sqrt(gamma_rad_A)),a_rad_A,b_rad_A);

            r = random(pd_rad_A,tau_im_A,1);

        end
    
        % For the angle:

        if length(lambda_ang_A) == 1

            phi = lambda_ang_A;

        else
    
            a_ang_A = min(lambda_ang_A);
            b_ang_A = max(lambda_ang_A);
        
            mu_ang_A = mean(lambda_ang_A);
            gamma_ang_A = var(lambda_ang_A);
        
            pd_ang_A = truncate(makedist('Normal','mu',mu_ang_A,'sigma',sqrt(gamma_ang_A)),a_ang_A,b_ang_A);

            phi = random(pd_ang_A,tau_im_A,1);

        end
        
        %
    
        for k = 1:tau_im_A
            
            Delta_im_A = [Delta_im_A; r(k)*exp(1j*phi(k)); r(k)*exp(-1j*phi(k))];
    
        end
    end
    
    % Final eigenvalues set:
    
    Delta_A = [Delta_real_A; Delta_im_A];
    
    % Building the auxiliary matrix:
    
    n_aux = length(Delta_A);
    
    M = diag(Delta_A);

    [Q,~] = qr(randn(n_aux));

    D = diag(logspace(0,1.5,n_aux));
    
    %D = eye(4);

    U = Q*D;
    
    A_aux = inv(U)*M*U;
end