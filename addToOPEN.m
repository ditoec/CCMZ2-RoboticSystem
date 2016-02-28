function [ newElementOPEN ] = addToOPEN( currentNode, parentNode, h, g )
%% EXPLANATION
% Add information of new node ( new row ) to OPEN list

%% INPUT FORMAT
% [ currentNode: 1-by-2 matrix ]
% [ x, y ]

% [ endNode: 1-by-2 matrix ]
% [ x, y ]

% [ h: scalar ]

% [ g: scalar ]

%% OUTPUT FORMAT
% [ newElementOPEN: 1-by-8 matrix ]
% [ Occupancy, xSelf, ySelf, xParent, yParent, h, g, f ]

%% FUNCTION
newElementOPEN = [ 1, 8 ]; % Create a 1-by-8 matrix
newElementOPEN( 1, 1 ) = 1; % The node exists
newElementOPEN( 1, 2 ) = currentNode(1); % xCurrent
newElementOPEN( 1, 3 ) = currentNode(2); % yCurrent
newElementOPEN( 1, 4 ) = parentNode(1); % xParent
newElementOPEN( 1, 5 ) = parentNode(2); % yParent
newElementOPEN( 1, 6 ) = h; % h cost
newElementOPEN( 1, 7 ) = g; % g cost
newElementOPEN( 1, 8 ) = h + g; % f cost
end