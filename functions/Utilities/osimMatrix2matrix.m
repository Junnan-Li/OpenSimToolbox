function matrix = osimMatrix2matrix(OsimMatrix)

nrow = OsimMatrix.nrow;
ncol = OsimMatrix.ncol;
matrix = zeros(nrow,ncol);
for i = 1:nrow
    if ncol == 1
        matrix(i) = OsimMatrix.get(i-1);
    else
        for j = 1:ncol
            matrix(i,j) = OsimMatrix.get(i-1,j-1);
        end
    end
end
end