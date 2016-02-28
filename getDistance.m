function [ distance ] = getDistance( currentNode, endNode )
%% EXPLANATION
% Calculate distance between current node and end node by Pythagorean theorem
% c^2 = a^2 + b^2

%% INPUT FORMAT
% [ currentNode: 1-by-2 matrix ]
% [ x, y ]

% [ endNode: 1-by-2 matrix ]
% [ x, y ]

%% OUTPUT FORMAT
% [ distance: scalar ]

%% FUNCTION
deltaX = currentNode(1) - endNode(1);
deltaY = currentNode(2) - endNode(2);
distance = sqrt( deltaX * deltaX + deltaY * deltaY );
end

