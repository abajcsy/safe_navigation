load("./c++_base_grid/t5.mat");
load("./base_grid/t5.mat");
EPS = 1e-2;
diff = {};
for i = 1:length(data_beacls)
    arr = data_beacls{i};
    arr2 = valueFun(:, :, :, i);
    arr3 = arr(:);
    arr4 = arr2(:);
    [equal, unequal_indexes] = compare_array(arr3, arr4, EPS);
    if ~equal
        fprintf("%d unequal indexes in time %d\n", length(unequal_indexes), i);
    end 
    diff{i} = max(abs(arr3 - arr4));
end 


function [equal, unequal_indexes] = compare_array(a1, a2, eps) 
    assert(isequal(size(a1), size(a2)), "sizes of arrays not equal");
%     unequal_indexes = find(abs(a1 - a2) > eps * abs(a1)); 
    unequal_indexes = find(abs(a1 - a2) > eps); 
    equal = numel(unequal_indexes) == 0;
end 
