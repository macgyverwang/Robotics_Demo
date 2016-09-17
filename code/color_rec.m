%% Recognizing color patches
%% Input: Image in HSV, HSV parameter, structuring element
%% Output: centroid point (2x1)array
function [centroid] = color_rec(pic_hsv, hsv_param, se)

% Tolerance
h_tolerance = 0.0417;     % 0.0417 = (1/6)/4
s_tolerance = 0.15;
v_tolerance = 0.20;

% Thresholding by hue
BW_h_mask = pic_hsv(:,:,1) <= (hsv_param(1) + h_tolerance) & pic_hsv(:,:,1) >= (hsv_param(1) - h_tolerance);
% Thresholding by satuation
BW_s_mask = pic_hsv(:,:,2) <= (hsv_param(2) + s_tolerance) & pic_hsv(:,:,2) >= (hsv_param(2) - s_tolerance);
% Thresholding by value
BW_v_mask = pic_hsv(:,:,3) <= (hsv_param(3) + v_tolerance) & pic_hsv(:,:,3) >= (hsv_param(3) - v_tolerance);

% Get the specific color's binary image
BW = BW_h_mask & BW_s_mask & BW_v_mask;

% Morphology close and open
BW_close    = imclose(BW,se);
BW_open     = imopen(BW_close,se);

% Centroids calculation
C = regionprops(BW_open, 'Centroid');
centroid = cat(1, C.Centroid);

temp = centroid(:,1);
centroid(:,1) = centroid(:,2);
centroid(:,2) = temp;

% figure(1)
% imshow(BW_open)
end