function pic_b = obstacle_img(pic_b, centroid_1, centroid_2)

% bump location
bump = [23, 32];

% bump and car
for x = 1:48
    for y = 1:64
        if sqrt((x-bump(1))^2+(y-bump(2))^2) <= 3.5
            pic_b(x,y) = 1;
        end
        
        if sqrt((x-centroid_1(1))^2+(y-centroid_1(2))^2) <= 3.5
            pic_b(x,y) = 1;
        end
        
        if sqrt((x-centroid_2(1))^2+(y-centroid_2(2))^2) <= 3.5
            pic_b(x,y) = 1;
        end
    end
end

% wall
pic_b(1:19,1) = 1;
pic_b(29:48,1) = 1;
pic_b(1:19,64) = 1;
pic_b(29:48,64) = 1;
pic_b(1,:) = 1;
pic_b(48,:) = 1;



end