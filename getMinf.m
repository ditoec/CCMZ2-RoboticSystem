function [ lowestfIndex ] = getMinf( OPEN, numElementsOPEN, endNode )
%% EXPLANATION
% Return index of node with minimum f cost in OPEN list

%% INPUT FORMAT

% [ OPEN list: N-by-8 matrix ]
% [ Occupancy, xSelf, ySelf, xParent, yParent, h, g, f ]

% [ numElementsOPEN: scalar ]

% [ endNode: 1-by-2 matrix ]
% [ x, y ]

%% OUTPUT FORMAT
% [ lowestfIndex: scalar ]

%% FUNCTION
arrayTemp = [];
index = 1;
inOPEN = 0; % 0 -- not in OPEN list; 1 -- in OPEN list
targetIndex = 0;

for i = 1:numElementsOPEN
    if OPEN( i, 1 )
        arrayTemp( index, : ) = [ OPEN( i, : ), i];
        if OPEN( i, 2:3 ) == endNode( 1:2 )
            inOPEN = 1;
            targetIndex = i; % Update the target index
        end
        index = index + 1;
    end
end

if inOPEN == 1
     lowestfIndex = targetIndex;
end

% Send index of node with lowest f cost
if size( arrayTemp ~= 0 )
    [ min_fn, minTemp ] = min( arrayTemp( : ,8 ) );% Index of node with lowest f cost
    lowestfIndex = arrayTemp( minTemp, 9 );% Index of node with lowest f cost in OPEN list
else
     lowestfIndex = -1; % No path available
end