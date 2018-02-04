function [J] = jacobian(f, x0)
y0 = f(x0);
eps = 1e-5;
J = zeros(size(y0, 1), size(x0, 1));
for ii = 1:size(J, 2)
  x1 = x0;
  x2 = x0;
  x1(ii) = x1(ii) - eps;
  x2(ii) = x2(ii) + eps;
  J(:, ii) = (f(x2) - f(x1)) / eps;
end
end
