function [X_norm, mu, sigma] = featureNormalize(X)

mu = zeros(1, size(X, 2));
sigma = zeros(1, size(X, 2));

for p = 1:size(X, 2)
  mu(p) = mean(X(:, p));
end

% sigma
for p = 1:size(X, 2)
  sigma(p) = std(X(:, p));
end

% X_norm
for p = 1:size(X, 2)
  if (sigma(p) ~= 0)
    for i = 1:size(X, 1)
      X_norm(i, p) = (X(i, p)-mu(p))/sigma(p);
    end
  else
    % sigma(p) == 0 <=> forall i, j,  X(i, p) == X(j, p) == mu(p)
    % In this case,  normalized values are all zero.
    % (mean is 0,  standard deviation is sigma(=0))
    X_norm(:, p) = zeros(size(X, 1), 1);
  end
end


% ============================================================

end