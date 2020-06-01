function matrixn_rand_bool(numRows, numCols)
    local output=matrixn()
    output:setSize(numRows, numCols)
    for i=0,numRows-1 do
        for j=0, numCols-1 do
            output:set(i,j, math.random(0,1))
        end
    end 
    return output
end

-- matrixn table, vectorn selection
function matrixn_zero_columns(table, selection)
    local output=matrixn()
    output:assign(table)
    for i=0, table:cols()-1 do
        if selection:get(i)==1.0 then
            output:column(i):setAllValue(0)
        end
    end
    return output
end

