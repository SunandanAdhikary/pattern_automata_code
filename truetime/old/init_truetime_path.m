%%% After install truetime package you need to be set a path of you truetime folder.So run this script with proper "folder path" 
setenv('TTKERNAL','D:\network\truetime-2.0\kernel')
% getenv('ttkernal')
addpath(getenv('TTKERNAL'))
addpath(strcat(getenv('TTKERNAL'),'/matlab/help'))
addpath(strcat(getenv('TTKERNAL'),'/matlab'))