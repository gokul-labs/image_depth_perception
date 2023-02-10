function [K, R, t] = estimate_params(P)
% ESTIMATE_PARAMS computes the intrinsic K, rotation R and translation t from
% given camera matrix P.

[U, sigma, V] = svd(P);
V = V(:, end);
c = V(1:3) ./ V(4);

% https://math.stackexchange.com/questions/1640695/rq-decomposition
reverse_matrix = [
    0, 0, 1;
    0, 1, 0;
    1, 0, 0;
];

% reverse rows
% A~ = reverse matrix * A
A = P(:, 1:3);
P = reverse_matrix * A;
P = P';
% Qr decomposition
[Q, R] = qr(P);

% A~T = Q~ * R~
% similarly
Q = reverse_matrix * Q';
R = reverse_matrix * R' * reverse_matrix;

% Ensure sign of K as needed
% flag = 1;
% if R(1,2) < 0
%     flag = -1;
% end
% K = abs(R);
% K(1,2) = flag * K(1,2);

% from stackexchange posts and reply posts
T = diag(sign(diag(R)));
K = R * T;
R = Q;
R = T * R; 
if round(det(R)) == -1
    R = -R;
end
t = -R * c;