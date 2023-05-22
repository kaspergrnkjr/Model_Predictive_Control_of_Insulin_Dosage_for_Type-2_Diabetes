function output = shiftAndAppend(obj, array, newColumn)
%shiftAndAppend     It takes the element and put it first in the array
%
%output = shiftAndAppend(array, newColumn) where array is your old array
%and new column is the column you want to insert. 
    output = array;
    output(:,(2:end)) = output(:,(1:end-1));
    output(:,1) = newColumn;
end