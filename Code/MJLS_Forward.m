function [x,u] = MJLS_Forward(A_theta,B_theta,K,theta,x0,Nk)

n = size(A_theta,1);
m = size(B_theta,2);

x = zeros(n,Nk);
u = zeros(m,Nk-1);

x(:,1) = x0;

for k = 1:Nk-1

    % =========================
    % MODE INDEX
    % =========================
    i = theta(k);

    % =========================
    % SYSTEM MATRICES
    % =========================
    Ai = A_theta(:,:,i);
    Bi = B_theta(:,:,i);

    % =========================
    % GAIN
    % =========================
    Ki = K{i};

    while iscell(Ki)
        Ki = Ki{1};
    end

    Ki = double(Ki);

    % =========================
    % DIMENSION CHECK
    % =========================
    if size(Ki,2) ~= n
        error("Ki tem dimensão %dx%d mas x é %dx1", ...
              size(Ki,1),size(Ki,2),n);
    end

    % =========================
    % CONTROL INPUT
    % =========================
    u(:,k) = Ki * x(:,k);

    % =========================
    % DYNAMICS
    % =========================
    x(:,k+1) = Ai * x(:,k) + Bi * u(:,k);

end

end