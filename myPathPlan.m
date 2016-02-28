function [ path ] = myPathPlan( startNode, endNode, mapArray )
%% INPUT FORMAT
% [ startNode: 1-by-2 matrix ]
% [ x, y ]

% [ endNode: 1-by-2 matrix ]
% [ x, y ]

% [ mapArray: N-by-M matrix ]
% 0 -- impassable nodes
% 1 -- passable nodes

%% OUTPUT FORMAT
% [ path: N-by-2 matrix ]
% [ x1, y1; x2, y2; ...; xN, yN ]

%% 'A STAR' ALGORITHM
% Number of rows and columns of input mapArray
numRow = size( mapArray, 1 );
numCol = size( mapArray, 2 );

% [ OPEN list: N-by-8 matrix ]
% [ Occupancy, xSelf, ySelf, xParent, yParent, h, g, f ]
OPEN = []; 

% [ CLOSED list: N-by-2 matrix ]
% [ xSelf, ySelf ]
CLOSED = [];

%% Include impassable nodes into CLOSED list
numElementsCLOSED = 0;
for x = 1:numRow
    for y = 1:numCol
        if ~mapArray( x, y )
            numElementsCLOSED = numElementsCLOSED + 1;
            CLOSED( numElementsCLOSED, 1 ) = x; 
            CLOSED( numElementsCLOSED, 2 ) = y; 
        end
    end
end

%% Start node manipulation
% Set start node as current node
numElementsOPEN = 1;
node = startNode;

% Add current node to OPEN list
costFromStart = 0;
costToEnd = getDistance( node, endNode );
OPEN( numElementsOPEN, : ) = addToOPEN( node, node, costToEnd, costFromStart );

% Remove current node from OPEN list
OPEN( numElementsOPEN, 1 ) = 0;

% Add current node to CLOSED list
numElementsCLOSED = numElementsCLOSED + 1;
CLOSED( numElementsCLOSED, : ) = node;

% Indicator of whether path is found
% 1 -- path not found; 2 -- path found
pathNotFound = 1;

%% Repeat loop until end node is found
% If new path to neighbour is shorter or heighbour is not in OPEN list:
% 1. Set f cost of neighbour
% 2. Set parent of neighbour to current
% 3. Add neighbour to OPEN if not in it
while ~isequal( node, endNode ) && pathNotFound
    neighbourNodes = regionGrow( node, endNode, costFromStart, numRow, numCol, CLOSED );
    numTraversableNodes = size( neighbourNodes, 1 );
    for i = 1:numTraversableNodes
        inOPEN = 0; % 0 -- not in OPEN list; 1 -- in OPEN list
        for j = 1:numElementsOPEN
            if neighbourNodes( i, 1:2 ) == OPEN( j, 2:3 ) % Neighbour node in OPEN list
                OPEN( j, 8 ) = min( OPEN( j, 8 ), neighbourNodes( i, 5 ) ); % Update if f cost is lower
                if OPEN( j, 8 ) == neighbourNodes( i, 5 )
                    OPEN( j, 4:5 ) = node; % Update x and y coordinates
                    OPEN( j, 6 ) = neighbourNodes( i, 3 ); % Update h cost
                    OPEN( j, 7 ) = neighbourNodes( i, 4 ); % Update g cost
                end
                inOPEN = 1;
            end
        end
        if inOPEN == 0
            numElementsOPEN = numElementsOPEN + 1;
            OPEN( numElementsOPEN, : ) = addToOPEN( neighbourNodes( i, 1:2 ), node, neighbourNodes( i, 3 ), neighbourNodes( i, 4 ) );
        end
    end
    
    % Find node with lowest f cost
    lowestfIndex = getMinf( OPEN, numElementsOPEN, endNode );
    if lowestfIndex ~= -1 % Set xNode and yNode to the node with minimum fn
        node = OPEN( lowestfIndex, 2:3 );
        costFromStart = OPEN( lowestfIndex, 6 ); % Update cost of reaching parent node
        % Add current node to CLOSED list
        numElementsCLOSED = numElementsCLOSED + 1;
        CLOSED( numElementsCLOSED, 1:2 ) = node;
        OPEN( lowestfIndex, 1 ) = 0; % Delete current node from OPEN
    else
      % No path exists
      pathNotFound = 0;
    end
end

% Trace parent nodes from the end to get optimal path
i = size( CLOSED, 1 );
path = [];
nodeTemp = CLOSED( i, : );

i=1;
path( i, : ) = nodeTemp;
i = i + 1;

if nodeTemp == endNode
    nodeIndex = 0;
    % Traverse OPEN and find parent nodes
    parentNode(1) = OPEN( getNodeIndex( OPEN, nodeTemp ), 4 ); % getNodeIndex returns index of node
    parentNode(2) = OPEN( getNodeIndex( OPEN, nodeTemp ), 5 );
    
    while ~isequal( parentNode, startNode )
        path( i, : ) = parentNode;
        % Get parent of parent
        nodeIndex = getNodeIndex( OPEN, parentNode );
        parentNode = OPEN( nodeIndex, 4:5 );
        i = i + 1;
    end
end

end
