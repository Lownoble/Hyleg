function output = current_process(data,current_col)

[N,M] = size(data);
j = current_col;
for i=2:N
    if(data(i-1,j)-data(i,j)<-20)
            data(i,j) = data(i,j)-36;
    end
    if(data(i-1,j)-data(i,j)>20)
            data(i,j) = data(i,j)+36;
    end
end
output = data;
for i=3:N-2
    output(i,j) = (data(i-2,j)+data(i-1,j)+data(i,j)+data(i+1,j)+data(i+2,j))/5;
end

end