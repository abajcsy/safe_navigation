% load data
load("./DubinsCar_RS_RS.mat");
load("./DubinsCar_Beacls_Interface.mat");
d1 = convert_4D_time_dimension(data); 
d2 = convert_4D_time_dimension(data_beacls);
disp(d1(1:10));
disp(d2(1:10));
% make sure data0 are all equal
t1 = d1(:, :, :, :);
t2 = d2(:, :, :, :); 
[equal, unequal_indexes] = array_almost_equal(t1(:), t2(:), 0.1); 
fprintf("Data equal? %d\n", equal); 
fprintf("%d ", unequal_indexes);
fprintf("\n"); 
fprintf("%d percentage unequal\n", (100 * numel(unequal_indexes)) / prod(size(d1)));


function [equal, unequal_indexes] = array_almost_equal(a1, a2, eps) 
    assert(isequal(size(a1), size(a2)), "sizes of arrays not equal");
    unequal_indexes = find(abs(a1 - a2) > eps * abs(a1)); 
    equal = numel(unequal_indexes) == 0;
end 

function [array] = convert_4D_time_dimension(data)
    shape = [size(data{1}), length(data)];
    array = zeros(shape);
    fprintf("dimension %d\n", shape); 
    for i = 1:length(data)  
        array(:, :, :, i) = data{i};
    end 
end

function [value] = get_index(array, indexes)
    sizes = size(array);
    assert(isequal(size(sizes), size(indexes)), "Array size not equal to index size");
    value = 1; % 1 indexed matlab 8)
    prod = 1;
    for i = 1:length(sizes) 
        value = value + prod * (indexes(i) - 1); 
        prod = prod * sizes(i); 
    end 
end 

