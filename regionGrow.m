function nodeList = regionGrow( currentNode, endNode, g, numRow, numCol, CLOSED )
%% EXPLANATION
% Add neighbour nodes of current nodes to nodeList is they are not in
% CLOSED

%% INPUT FORMAT
% [ currentNode: 1-by-2 matrix ]
% [ x, y ]

% [ endNode: 1-by-2 matrix ]
% [ x, y ]

% [ g: scalar ]

% [ numRow: scalar ]

% [ numCol: scalar ]

% [ CLOSED list: N-by-2 matrix ]
% [ xSelf, ySelf ]

%% OUTPUT FORMAT
% [ nodeList: N-by-5 matrix ]
% [ ( x, y ), h, g, f ]

%% FUNCTION
nodeList = [];
listIndex = 0;
closedSize = size( CLOSED, 1 );

% o -- currrent node
% x -- successor nodes of current node
% | x | x | x |
%  - - - - - -
% | x | o | x |
%  - - - - - -
% | x | x | x |
for x = -1:1
    for y = -1:1
        if x ~= 0 || x ~= y
            successorNode(1) = currentNode(1) + x; % xSuccessorNode
            successorNode(2) = currentNode(2) + y; % ySuccessorNode
            if successorNode(1) > 0 && successorNode(2) > 0 && successorNode(1) <= numRow && successorNode(2) <= numCol
                inCLOSED = 1; % 1 -- not in CLOSED list; 0 -- in CLOSED list
                for i = 1:closedSize
                    if successorNode == CLOSED( i, 1:2 )
                        inCLOSED = 0;
                    end
                end
                if inCLOSED
                    listIndex = listIndex + 1;
                    nodeList( listIndex, 1:2 ) = successorNode; % x and y coordinates of successor node
                    nodeList( listIndex, 3 ) = getDistance( endNode, successorNode ); % h cost
                    nodeList( listIndex, 4 ) = g + getDistance( currentNode, successorNode ); % g cost
                    nodeList( listIndex, 5 ) = nodeList( listIndex, 3 ) + nodeList( listIndex, 4 ); % f cost
                end
            end
        end
    end
end