clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
close all;
clear all;
axis equal; %keeps the x and y scale the same
map = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
%map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0];
%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.

disp('You can use the pointInsideMap() function to sample the map vectors');
disp('to make a grid representation of the map for pathfinding');

hold on;
botSim.drawMap();

limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax-limsMin; %dimension of the map
res = 5; %sampling resouloution in cm
iterators = dims / res;
iterators = ceil(iterators)+[ 1, 1 ]; %to counteract 1 based indexing
mapArray = zeros(iterators); %preallocate for speed

%loops through the grid indexes and tests if they are inside the map
figure(1);
hold on
for i = 1:iterators(2)
    for j = 1:iterators(1)
        testPos = limsMin + [ j-1, i-1 ] * res; %to counteract 1 based indexing
        %notice, that i and j have also been swapped here so that the only
        %thing left to do is invert the y axis. 
        mapArray( i, j ) = botSim.pointInsideMap( testPos );
        if mapArray( i, j )
            plot( testPos( 1 ), testPos( 2 ), '*' ); %inside map
        else
            plot( testPos( 1 ), testPos( 2 ), 'o' ); %outside map
        end
    end
end

%prints array of samplePoints. 
disp('This naive implementation does not return the map in the same'); 
disp('orientation as plotted map.  Here the y axis is inverted')
mapArray;
newMapArray = mapArray;

for x = 1:iterators(2)
    if mapArray( x, 1 ) == 1
        newMapArray( x, 1 ) = 0;
    end
    if mapArray( x, iterators(1) ) == 1
        newMapArray( x, iterators(1) ) = 0;
    end
end
for y = 1:iterators(1)
    if mapArray( 1, y ) == 1
        newMapArray( 1, y ) = 0;
    end
    if mapArray( iterators(2), y ) == 1
        newMapArray( iterators(2), y ) = 0;
    end
end

for x = 2:iterators(2) - 1
    for y = 2:iterators(1) - 1
        if mapArray( x - 1, y - 1 ) == 0 || mapArray( x - 1, y ) == 0 || mapArray( x - 1, y + 1 ) == 0 || mapArray( x, y - 1 ) == 0 || mapArray( x, y + 1 ) == 0 || mapArray( x + 1, y - 1 ) == 0 || mapArray( x + 1, y ) == 0 || mapArray( x + 1, y + 1 ) == 0
            newMapArray( x, y ) = 0;
        end
    end
end

newMapArray

figure(2);
hold on;
botSim.drawMap();
axis equal; %keeps the x and y scale the same
for i = 1:iterators(2)
    for j = 1:iterators(1)
        if newMapArray(i,j)
            plot( (j-1)*res, (i-1)*res, '*' );%inside map
        else
            plot( (j-1)*res, (i-1)*res, 'o' );%outside map
        end
    end
end

path = [];
startNode = [ 3, 3 ];
endNode = [ 20, 2 ];
path = myPathPlan( startNode, endNode, newMapArray )
