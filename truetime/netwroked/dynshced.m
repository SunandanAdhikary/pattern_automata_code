function [exectime,data] = dynshced(seg,data)

switch seg
	case 1
		exectime = data.E;
	case 2
		exectime = -1;
end

		