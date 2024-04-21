function [exectime,data] = other_task(seg,data)

switch seg
	case 1
		exectime = data.E;
        
        disp("time: "+ttCurrentTime()+", Executing "+ttCurrentTask()+" that started at "+ttGetRelease()+" with abs deadline= "+ttGetAbsDeadline()+" and remaining execution time "+ ttGetBudget());
	case 2
		exectime = -1;
end

		