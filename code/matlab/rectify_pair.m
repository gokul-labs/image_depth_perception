function [M1, M2, K1n, K2n, R1n, R2n, t1n, t2n] = ...
                        rectify_pair(K1, K2, R1, R2, t1, t2)
% RECTIFY_PAIR takes left and right camera paramters (K, R, T) and returns left
%   and right rectification matrices (M1, M2) and updated camera parameters. You
%   can test your function using the provided script q4rectify.m

% 1
c1 = -inv(R1) * inv(K1) * (K1 * t1);
c2 = -inv(R2) * inv(K2) * (K2 * t2);

% 2 from slidedeck 20-stereo.pdf

% changng order of c1, c2 as image is inverted or we can take abs
r1 = c2-c1 / norm(c2 - c1); 
r2 = cross(R1(3, :), r1);
r3 = cross(r1, r2);

R1new = [r1';r2/norm(r2);r3/norm(r3)];

r1 = c2-c1 / norm(c2 - c1); 
r2 = cross(R2(3, :),r1);
r3 = cross(r1, r2);

R2new = [r1';r2/norm(r2);r3/norm(r3)];

% 3

K1new = K1;
K2new = K2;

% 4

t1new = -R1new * c1;
t2new = -R2new * c2;

% 5

M1 = -(K1new * R1new) * inv(R1) * inv(K1);
M2 = -(K2new * R2new) * inv(R2) * inv(K2);

K1n = K1new;
R1n = R1new;
R2n = R2new;
K2n = K2new;
t1n = t1new;
t2n = t2new;