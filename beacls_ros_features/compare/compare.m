EPS = 0.1;
path1 = "./dubins_c.mat";
path2 = "./dubins.mat";
verbose = true; 
equal, unequal_index = compareFunction(path1, path2, EPS, verbose); 

function [equal, unequal_indexes] = compareFunction(path1, path2, EPS, verbose)
    % load data and parametes
    load(path1);
    load(path2);

    % convert data
    data_c = convert_4D_time_dimension(data, true); 
    %data_c(2, 1, 1, 1) = 1;
    %data_c(41, 1, 1, 1) = 1;
    data_m = d2; 

    % compare for unequal indexes
    error_msg = "c and matlab value function shapes are not equivalent";
    assert(numel(data_c) == numel(data_m), error_msg);
    [equal, unequal_indexes] = array_almost_equal(data_c(:), data_m(:), EPS);
    if verbose
        fprintf("Data equal? %d\nUnequal Indexes:", equal); 
        fprintf("%d ", unequal_indexes);
        fprintf("\n"); 
        fprintf("Percentage of unequal indexes: %.2f%%\n", numel(unequal_indexes) / numel(data_c) * 100);  
    end 
end 

function [equal, unequal_indexes] = array_almost_equal(a1, a2, eps) 
    assert(isequal(size(a1), size(a2)), "sizes of arrays not equal");
    unequal_indexes = find(abs(a1 - a2) > eps * abs(a1)); 
    equal = numel(unequal_indexes) == 0;
end 

function [array] = convert_4D_time_dimension(data, skip_index)
    shape = [size(data{1}), length(data)];
    array = zeros(shape);
    fprintf("Original dimensions: %d\n", shape); 
    for i = 1:length(data)  
        array(:, :, :, i) = data{i};
    end 
    if skip_index
        shape = [size(data{1}), round(length(data) / 2)];
        array = zeros(shape);
        fprintf("Updated dimensions: %d\n", shape); 
        for i = 1:length(data)  
            if mod(i, 2) == 1
               array(:, :, :, round((i + 1) / 2)) = data{i}; 
            end
        end 
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
