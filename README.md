#Robotics_Demo

##Introduction:
The main purpose of this demonstration is to recognize color patches on the cars and ball, and find a path by A* algorithm in further.

##Procedure:
1. Recognizing color patches  
2. Creating map  
3. A* path finding  

Source code comprises one main code, robotics_demo.m, and three M-file functions.

##Input Image:
![Oringinal Input Image](https://github.com/ChangYuHsuan/Robotics_Demo/blob/master/pictures/test.jpg)

##Final Results:
![Final Output Image](https://github.com/ChangYuHsuan/Robotics_Demo/blob/master/pictures/final_output.jpg)

##Functions Description:
###* color_rec:  
Input: Image in HSV, HSV parameter, structuring element  
Output: centroid point (2x1)array   

![Red Patch](https://github.com/ChangYuHsuan/Robotics_Demo/blob/master/pictures/red_patch.jpg)  

###* obstacle_img:  
Input: Color patches' centroid point  
Output: 48x64 Binary Image   

![Obstacle Image](https://github.com/ChangYuHsuan/Robotics_Demo/blob/master/pictures/obstacle_image.jpg)  

###* astar:  
Input: pic_b(48x64 map image), start point, target point, structuring element  
Output: path bank (2xN) matrix  

##Reference:  
More details about A* algorithm: [A* Pathfinding for Beginner](http://www.policyalmanac.org/games/aStarTutorial.htm)

