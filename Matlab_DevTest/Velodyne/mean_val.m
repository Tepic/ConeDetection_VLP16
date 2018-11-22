function output = mean_val(input)
    [r c] = size(input);
    if(r~=1)
        output = zeros(1,c);
        for column = 1:c
            output(column) = sum(input(:,column));
        end
        output = output./r;
    else
        output = sum(input)/r;
    end
end