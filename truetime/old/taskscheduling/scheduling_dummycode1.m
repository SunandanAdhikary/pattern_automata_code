function [exectime,data] = scheduling_dummycode1(seg,data)

switch seg
	case 1
		exectime = data.E;
	case 2
		exectime = -1;
end

		