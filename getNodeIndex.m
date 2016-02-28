function [ index ] = getNodeIndex( OPEN, node )
%% EXPLANATION
% Return index of node

%% INPUT FORMAT

% [ OPEN list: N-by-8 matrix ]
% [ Occupancy, xSelf, ySelf, xParent, yParent, h, g, f ]

% [ node: 1-by-2 matrix ]
% [ x, y ]

%% OUTPUT FORMAT
% [ index: scalar ]

%% FUNCTION
index=1;
while OPEN(index,2) ~= node(1) || OPEN(index,3) ~= node(2)
    index = index + 1;
end
end