function [cellNames] = nameExpand(name,order)
% Creates a cell array with the name spread from 1 to n. name(number)
% Example nameExpand('y',4) = {'y(1)','y(2)','y(3)','y(4)'};

if(ischar(name) || ischar(name) ) % check input type
    cellNames = {};

    for i = 1:order
        cellNames = {cellNames{1:end} , [name '(' num2str(i,1) ')']};
    end
else
    error('The inputs must be as it shows: nameExpand( char vector ''like this'' , integer or real(rounded )). I tought it was obvious')
end

end

