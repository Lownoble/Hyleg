function data = walk_dataInput(inputArg)

data = [];

% 打开文件
fid = fopen(inputArg, 'r');

% 逐行读取文件
line = fgetl(fid);
while ischar(line)
    % 用空格或制表符分隔每行的内容
    parts = strsplit(line, {' ', '\t'});
    
    % 将字符串转换为数字，忽略字符部分
    numericValues = str2double(parts(~isnan(str2double(parts))));
    
    % 将数值添加到数据矩阵
    data = [data; numericValues];
    
    % 读取下一行
    line = fgetl(fid);
end

% 关闭文件
fclose(fid);

end