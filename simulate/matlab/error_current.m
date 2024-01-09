clc
close all
clear

%% 导入数据
data1 = importdata('./DATA/walk_0.0MPa_0kg.txt');
% 获取行数和列数
numRows = numel(data1);
numCols = 26;

% 创建矩阵来存储分开的数据
matrixData = zeros(numRows, numCols);

% 将每一行的数据分开存储到矩阵中
for i = 1:numRows
    rowData = data1{i};
    matrixData(i, :) = str2double(strsplit(rowData));
end


%% 
i = 13;
error = matrixData(:, 5+i)-matrixData(:, 6+i);
current = matrixData(:, 8+i);

plot(error*345,"r");
hold on
plot(current,"g");
legend("error*345","current")