function [exectime, data] = dl_miss_code(seg, data)

% switch seg
%  case 1
    task = sscanf(ttGetInvoker,'%s');
    disp(task +' missed deadline');
    exectime  = 0.00001;
% case 2
%     exectime = -1;
end