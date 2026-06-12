function theta = Generate_Markov_Chain(N,P,Nk)

    % P rows must sum to 1:

    if any(abs(sum(P,2) - 1) > 1e-10)
        error('P rows must sum to 1.');
    end

    % P must have dimension (N-1) x (N-1):

    if size(P,1) ~= N-1 || size(P,2) ~= N-1
        error('P must be of size (N-1)x(N-1).');
    end
    
    % Number of utilizable modes:

    modes = 2:N;

    % Initialize the chain:

    theta = zeros(Nk,1);

    % Initial phase: mode 1 fixed

    theta(1:100) = 1;

    % Changing phases from mode 1 to {2...N}

    p = P(1,:);

    p(1) = 0;

    p = p / sum(p);
    
    S = cumsum(p);

    r = rand;

    idx = find(r <= S,1);

    if isempty(idx)
        idx = N;
    end

    theta(101) = idx;

    % Markov Evolution in {2...N}

    for k = 101:Nk-1

        idx = theta(k) - 1;
        
        p = P(idx,:);

        %p(1) = 0;

        %p = p / sum(p);

        S = cumsum(p);

        r = rand;

        theta(k+1) = find(r <= S,1) + 1;

    end

end