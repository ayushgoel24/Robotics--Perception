function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
matrx = zeros(8,9);
video_x = video_pts(:, 1);
video_y = video_pts(:, 2);
logo_x = logo_pts(:, 1);
logo_y = logo_pts(:, 2);

matrx = [-video_x(1), -video_y(1), -1, 0, 0, 0, video_x(1)*logo_x(1), video_y(1)*logo_x(1), logo_x(1);0, 0, 0, -video_x(1), -video_y(1), -1, video_x(1)*logo_y(1), video_y(1)*logo_y(1), logo_y(1);-video_x(2), -video_y(2), -1, 0, 0, 0, video_x(2)*logo_x(2), video_y(2)*logo_x(2), logo_x(2);0, 0, 0, -video_x(2), -video_y(2), -1, video_x(2)*logo_y(2), video_y(2)*logo_y(2), logo_y(2);-video_x(3), -video_y(3), -1, 0, 0, 0, video_x(3)*logo_x(3), video_y(3)*logo_x(3), logo_x(3);0, 0, 0, -video_x(3), -video_y(3), -1, video_x(3)*logo_y(3), video_y(3)*logo_y(3), logo_y(3);-video_x(4), -video_y(4), -1, 0, 0, 0, video_x(4)*logo_x(4), video_y(4)*logo_x(4), logo_x(4);0, 0, 0, -video_x(4), -video_y(4), -1, video_x(4)*logo_y(4), video_y(4)*logo_y(4), logo_y(4)];

[m1, m2, m3] = svd(matrx);
h = m3(:, end);
H = reshape (h, [3, 3]);
H = H';

end

