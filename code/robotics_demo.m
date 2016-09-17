clear all
clc

pic = imread('test.jpg');
pic_hsv = rgb2hsv(pic);

se_2 = strel('disk', 2);
se_4 = strel('disk', 4);

%hsv parameters
red_hsv     = [0.97, 0.75, 0.27];
brown_hsv   = [0.05, 0.50, 0.25];
yellow_hsv  = [0.11, 0.80, 0.52];
pink_hsv    = [0.95, 0.60, 0.55];
green_hsv   = [0.32, 0.45, 0.50];

tic

red     = color_rec(pic_hsv, red_hsv, se_2);
brown   = color_rec(pic_hsv, brown_hsv, se_2);
yellow  = color_rec(pic_hsv, yellow_hsv, se_2);
pink    = color_rec(pic_hsv, pink_hsv, se_2);
green   = color_rec(pic_hsv, green_hsv, se_2);

pic_b = obstacle_img(round(yellow/10), round(pink/10));
pathBank = astar(pic_b, round(brown/10), round(green/10), se_4);

toc

for i = 1:size(pathBank,2)
    pic(pathBank(1,i)*10, pathBank(2,i)*10,:) = [255; 0; 0];
end

figure(1)
imshow(pic)