%% A* algorithm
%% Input: pic_b(48x64 map image), centroid(start) point, target point, structuring element 
%% Output: path bank (2xN) matrix
function pathBank = astar(pic_b, centroid, target, se)

% declare constant
mapWidth = size(pic_b,1);
mapHeight = size(pic_b,2);
tileSize = 1;
walkable = 0;
unwalkable = 1;
onClosedList = 10;
onOpenList = onClosedList - 1;
found = 1;
nonexistent = 2;

% create needed array
% openList = zeros(mapWidth*mapHeight+2);
 whichList = zeros(mapWidth+1, mapHeight+1);
% openX = zeros(mapWidth*mapHeight+2);
% openY = zeros(mapWidth*mapHeight+2);
% parentX = zeros(mapWidth+1, mapHeight+1);
% parentY = zeros(mapWidth+1, mapHeight+1);
% Fcost = zeros(mapWidth*mapHeight+2);
 Gcost = zeros(mapWidth+1, mapHeight+1);
% Hcost = zeros(mapWidth*mapHeight+2);
 pathLength = 0;

% Initial point
startingX = round(centroid(1));
if startingX == 0;
    startingX = 1;
end
startingY = round(centroid(2));
if startingY == 0;
    startingY = 1;
end

% Target point
if (target(1) ~= 0) && (target(2) ~= 0)
   targetX = round(target(1));
   targetY = round(target(2));
else
   targetX = round(centroid(1));
   targetY = round(centroid(2));
end

if targetX == 0;
    targetX = 1;
end
if targetY == 0;
    targetY = 1;
end

% Obstacle
walkability = pic_b;
penalty = imerode(walkability,se);

newOpenListItemID = 0;

%% Run Algorithm
% 1. convert location data to coordinates in the walkability array
startX = round(startingX/tileSize);
startY = round(startingY/tileSize);
pathBank = [startX, startX, startX, startX; startY, startY, startY, startY];

% 2. Quick path check
if (startX == targetX) && (startY == targetY)
    fprintf('found\n');
    pathBank = [startX, startX; startY, startY];
elseif (walkability(targetX,targetY) == unwalkable)
	fprintf('noPath\n');
    pathBank = [startX, startX; startY, startY];
else
    
    % 3. add the starting location to the open list 
    numberOfOpenListItems = 1;
    openList(1) = 1;
    openX(1) = startX;
    openY(1) = startY;
    
    while 1 % 4. until find the path or nonexist
        if numberOfOpenListItems ~= 0
            % 5. the open list is not empty, take the first cell off the list
            parentXval = openX(openList(1));
            parentYval = openY(openList(1));
            if parentXval == 0
                parentXval = 1;
            end
            if parentYval == 0
                parentYval = 1;
            end
            whichList(parentXval,parentYval) = onClosedList; 

            numberOfOpenListItems = numberOfOpenListItems - 1;
            openList(1) = openList(numberOfOpenListItems+1); % move the last item in the heap up to slot #1
            v = 1;
            
            while 1 % repeat the following until the new item in slot #1 sinks to its proper spot
                u = v;
                if (2*u+1 <= numberOfOpenListItems) % if both children exist
                    % Check if the F cost of the parent is greater than each child.
                    % Select the lowest of the two children.
                    if Fcost(openList(u)) >= Fcost(openList(2*u)) 
                        v = 2*u;
                    end
                    if Fcost(openList(v)) >= Fcost(openList(2*u+1)) 
                        v = 2*u+1;
                    end
                else
                    if 2*u <= numberOfOpenListItems % if only child #1 exists
                    % Check if the F cost of the parent is greater than child #1	
                        if Fcost(openList(u)) >= Fcost(openList(2*u)) 
                            v = 2*u;
                        end
                    end
                end

                if u ~= v % if parent's F is > one of its children, swap them
                    temp = openList(u);
                    openList(u) = openList(v);
                    openList(v) = temp;			
                else
                    break; %otherwise, exit loop
                end
            end
            
            % 6. check the adjacent squares
            for b = (parentYval - 1): (parentYval + 1)
                for a = (parentXval - 1): (parentXval + 1)
                    if a > 0 && b > 0 && a <= mapWidth && b <= mapHeight
                        if whichList(a,b) ~= onClosedList
                            if walkability(a,b) ~= unwalkable
                                % don't cut cross corner
                                corner = walkable;
                                if (a == parentXval-1) 
                                    if (b == parentYval-1)
                                        if (walkability(parentXval-1,parentYval) == unwalkable) || (walkability(parentXval,parentYval-1) == unwalkable) 
                                            corner = unwalkable;
                                        end
                                    elseif (b == parentYval+1)
                                        if (walkability(parentXval,parentYval+1) == unwalkable || walkability(parentXval-1,parentYval) == unwalkable) 
                                            corner = unwalkable; 
                                        end
                                    end
                                elseif (a == parentXval+1)
                                    if (b == parentYval-1)
                                        if (walkability(parentXval,parentYval-1) == unwalkable || walkability(parentXval+1,parentYval) == unwalkable) 
                                            corner = unwalkable;
                                        end
                                    elseif (b == parentYval+1)
                                        if (walkability(parentXval+1,parentYval) == unwalkable || walkability(parentXval,parentYval+1) == unwalkable)
                                            corner = unwalkable; 
                                        end
                                    end
                                end
                                if corner == walkable
                                    % If not already on the open list, add it to the open list.			
                                    if (whichList(a,b) ~= onOpenList) 
                                        %Create a new open list item in the binary heap.
                                        newOpenListItemID = newOpenListItemID + 1; % each new item has a unique ID #
                                        m = numberOfOpenListItems+1;
                                        openList(m) = newOpenListItemID; % place the new open list item (actually, its ID#) at the bottom of the heap
                                        openX(newOpenListItemID) = a;
                                        openY(newOpenListItemID) = b; % record the x and y coordinates of the new item

                                        % Figure out its G cost
                                        if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
                                            addedGCost = 14; % cost of going to diagonal squares	
                                        else	
                                            addedGCost = 10; % cost of going to non-diagonal squares
                                        end
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        if penalty(a,b) == 1
                                            addedPenalty = 25;
                                        else
                                            addedPenalty = 0;
                                        end
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        Gcost(a,b) = Gcost(parentXval,parentYval) + addedGCost + addedPenalty;

                                        %Figure out its H and F costs and parent
                                        Hcost(openList(m)) = 10*(abs(a - targetX) + abs(b - targetY));
                                        Fcost(openList(m)) = Gcost(a,b) + Hcost(openList(m));
                                        parentX(a,b) = parentXval ; parentY(a,b) = parentYval;	

                                        % Move the new open list item to the proper place in the binary heap.
                                        % Starting at the bottom, successively compare to parent items,
                                        % swapping as needed until the item finds its place in the heap
                                        % or bubbles all the way to the top (if it has the lowest F cost).
                                        while m ~= 1 % While item hasn't bubbled to the top (m=1)	
                                            %Check if child's F cost is < parent's F cost. If so, swap them.	
                                            if (Fcost(openList(m)) <= Fcost(openList(round(m/2))))            
                                                temp = openList(round(m/2));
                                                openList(round(m/2)) = openList(m);
                                                openList(m) = temp;
                                                m = round(m/2);
                                            else
                                                break;
                                            end
                                        end
                                        
                                        numberOfOpenListItems = numberOfOpenListItems+1; % add one to the number of items in the heap

                                        % Change whichList to show that the new item is on the open list.
                                        whichList(a,b) = onOpenList;
                                    else % If whichList(a,b) = onOpenList
                                        %Figure out the G cost of this possible new path
                                        if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
                                            addedGCost = 14; % cost of going to diagonal tiles	
                                        else	
                                            addedGCost = 10; % cost of going to non-diagonal tiles				
                                        end
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        if penalty(a,b) == 1
                                            addedPenalty = 25;
                                        else
                                            addedPenalty = 0;
                                        end
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                        tempGcost = Gcost(parentXval,parentYval) + addedGCost + addedPenalty;

                                        % If this path is shorter (G cost is lower) then change
                                        % the parent cell, G cost and F cost. 		
                                        if (tempGcost < Gcost(a,b)) % if G cost is less,
                                            parentX(a,b) = parentXval; % change the square's parent
                                            parentY(a,b) = parentYval;
                                            Gcost(a,b) = tempGcost; % change the G cost			

                                            % Because changing the G cost also changes the F cost, if
                                            % the item is on the open list we need to change the item's
                                            % recorded F cost and its position on the open list to make
                                            % sure that we maintain a properly ordered open list.
                                            for x = 1: numberOfOpenListItems % look for the item in the heap
                                                if openX(openList(x)) == a && openY(openList(x)) == b % item found
                                                    Fcost(openList(x)) = Gcost(a,b) + Hcost(openList(x));% change the F cost

                                                    % See if changing the F score bubbles the item up from it's current location in the heap
                                                    m = x;
                                                    while (m ~= 1) % While item hasn't bubbled to the top (m=1)	
                                                        % Check if child is < parent. If so, swap them.	
                                                        if (Fcost(openList(m)) < Fcost(openList(round(m/2))))
                                                            temp = openList(round(m/2));
                                                            openList(round(m/2)) = openList(m);
                                                            openList(m) = temp;
                                                            m = round(m/2);
                                                        else
                                                            break;
                                                        end
                                                    end
                                                    break; % exit for x = loop
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        % 9.If open list is empty then there is no path.	
        else
            path = nonexistent;
            break;
        end
        % If target is added to open list then path has been found.
        if (whichList(targetX,targetY) == onOpenList)
            path = found; 
            break;
        end
    end
    
    % 10.Save the path if it exists.
    if path == found
        pathX = targetX;
        pathY = targetY;
        while 1
            % Look up the parent of the current cell.	
            tempx = parentX(pathX,pathY);		
            pathY = parentY(pathX,pathY);
            pathX = tempx;

            % Figure out the path length
            pathLength = pathLength + 1;
            
            if pathX == startX && pathY == startY
                break;
            end
        end
            
        % Now copy the path information over to the databank. Since we are
        % working backwards from the target to the start location, we copy
        % the information to the data bank in reverse order. The result is
        % a properly ordered set of path data, from the first step to the
        % last.
        pathX = targetX ; pathY = targetY;
        cellPosition = pathLength + 1;	% start at the end	
        pathBank = zeros(2,cellPosition-1);
        while 1
            cellPosition = cellPosition - 1;
            pathBank(1,cellPosition) = pathX;
            pathBank(2,cellPosition) = pathY;

            % d.Look up the parent of the current cell.	
            tempx = parentX(pathX,pathY);		
            pathY = parentY(pathX,pathY);
            pathX = tempx;

            % e.If we have reached the starting square, exit the loop.
            if pathX == startX && pathY == startY
                break;
            end
        end
    end
end

n = size(pathBank,2);
if n < 2;
    pathBank(:,2) = [targetX; targetY];
end