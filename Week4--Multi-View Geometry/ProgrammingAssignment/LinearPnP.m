function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

if 1
    N = size(X, 1);
    
    x_calb = K \ [x, ones(N, 1)]';
    x_calb = x_calb';
    
    X = [X ones(N, 1)];
  
    Z = zeros(N, 1);
    O = ones(N, 1);
    
    M = [Z, Z, Z, Z, -X, x_calb(:,2).*X; X, Z, Z, Z, Z, -x_calb(:,1).*X; -x_calb(:,2).*X, x_calb(:,1).*X Z, Z, Z, Z];
    [~, ~, m3] = svd(M);

    mat1 = reshape(m3(:,end), [4, 3])';
    
    mat2 = mat1;
    
    R = mat2(:, 1:3);
    T = mat2(:, end);
    
    [m1, m2, m3] = svd(R);
    
    det_a = det(m1*m3');
    if abs(det_a - 1) < 0.0001
       R = m1 * m3';
       T = T / m2(1, 1);
    elseif abs(det_a + 1) < 0.0001
       R = -m1 * m3';
       T = -T / m2(1, 1);
    end
       
    C = -R' * T;
end
end






