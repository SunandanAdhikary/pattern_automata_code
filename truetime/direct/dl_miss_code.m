function [exectime, data] = dl_miss_code(seg, data)

% ttCurrentTime = ttCurrentTime()
thistask = ttCurrentTask()

task = sscanf(ttGetInvoker,'DLtimer:%s')
ttGetAbsDeadlineOfDLMissTask=ttGetAbsDeadline(task)
ttGetReleaseTimeOfDLMissTask=ttGetRelease(task)

if ~isempty(task) && (ttGetReleaseTimeOfDLMissTask ~= ttGetAbsDeadlineOfDLMissTask)
  ttKillJob(task)
end

exectime = -1;
