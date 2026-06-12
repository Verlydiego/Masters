function K = MJLS_Backwards(A_theta,B_theta,P_markov,Q,R,Nk)

N = size(A_theta,3); 
n = size(A_theta,1);

s = N-1;   % MJLS modes: 2..N

K = cell(N,1);

% =========================
% terminal cost (full space)
% =========================
P_next = cell(N,1);

for i = 1:N
    P_next{i} = Q{i};
end

% =========================
% backward recursion
% =========================
for k = Nk-1:-1:1

    P_curr = cell(N,1);

    % =====================
    % MODE 1 (separado)
    % =====================
    A1 = A_theta(:,:,1);
    B1 = B_theta(:,:,1);

    Psi1 = Q{1};

    S1 = R + B1' * Psi1 * B1;
    K{1} = -S1 \ (B1' * Psi1 * A1);

    P_curr{1} = Q{1} + (A1 + B1*K{1})' * Psi1 * (A1 + B1*K{1});

    % =====================
    % MJLS MODES 2..N
    % =====================
    for i = 2:N

        Ai = A_theta(:,:,i);
        Bi = B_theta(:,:,i);
        Qi = Q{i};

        Psi = zeros(n,n);

        for j = 2:N
            Psi = Psi + P_markov(i-1,j-1)*P_next{j};
        end

        S = R + Bi' * Psi * Bi;

        K{i} = -S \ (Bi' * Psi * Ai);

        Acl = Ai + Bi*K{i};

        P_curr{i} = Qi + Acl' * Psi * Acl;

    end

    P_next = P_curr;

end

end