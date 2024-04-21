%%% Example 4 - Control Task Period Selection %%%

% This example requires the MatlabTrueTime and Jitterbug to be run. 

Tvec = 0.3:0.10:1.2; % Vector of periods to try
% Tvec = 0.6; %[0.4 0.6];
Jvec = [];

Tsim = 1000;
for Tsamp = Tvec          % Period of task 3
  Tsamp
	sim('scheduling1',Tsim)  % Simulate the system for Tsim seconds
	J = JP.signals(1).values(end) / Tsim  % Calculate average cost
	Jvec = [Jvec J];
end

% Plot the results
plot(Tvec,Jvec)
xlabel('Task period')
ylabel('Controller cost')
