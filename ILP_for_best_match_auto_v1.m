%{ 
Maximize
obj: 3 x1 + 3 x2 + 3 x3 -3 x4 + 3 y1 - 3 y2 + 3 y3 - 3 y4 + 1 slack

Subject To

x1 + x2 + x3 + x4 = 3
y1 + y2 + y3 + y4 = 2

1 x1 <= 2
1 x2 <= 2
1 x3 <= 2
1 x4 <= 2

3 y1 <= 4
3 y2 <= 4
3 y3 <= 4
3 y4 <= 4

x1 + x2 + 3 y1 <= 4
x3 + x4 + 3 y2 <= 4
x1 + x2 + 3 y3 <= 4
x3 + x4 + 3 y4 <= 4

slack=1

%}

%{
A=[1 0 0 0 0 0 0 0 0
    0 1 0 0 0 0 0 0 0
    0 0 1 0 0 0 0 0 0
    0 0 0 1 0 0 0 0 0
    0 0 0 0 3 0 0 0 0
    0 0 0 0 0 3 0 0 0
    0 0 0 0 0 0 3 0 0
    0 0 0 0 0 0 0 3 0
    1 1 0 0 3 0 0 0 0
    0 0 1 1 0 3 0 0 0
    1 1 0 0 0 0 3 0 0
    0 0 1 1 0 0 0 3 0];

b=[2;2;2;2;4;4;4;4;4;4;4;4];

Aeq=[1 1 1 1 0 0 0 0 0
    0 0 0 0 1 1 1 1 0
    0 0 0 0 0 0 0 0 1];

beq=[3;2;1];

intcon=[1,2,3,4,5,6,7,8,9];
lb = zeros(9,1);
ub=ones(9,1);

f=[-3;-3;-3;3;-3;3;-3;3;-1];
% x=intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);
[x,fval,exitflag,output]=intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);
disp('x is: ')
X=[x(1) x(2) x(3) x(4)]

disp('y is: ')
Y=[x(5) x(6) x(7) x(8)]

disp('slack is: ')
x(9)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Output Produced by intlinprog %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
LP:                Optimal objective value is -12.000000.                                           

Optimal solution found.

Intlinprog stopped at the root node because the objective value is within a gap tolerance of the optimal value,
options.TolGapAbs = 0 (the default value). The intcon variables are integer within tolerance,
options.TolInteger = 1e-05 (the default value).

x =

     0
     1
     1
     1
     1
     0
     1
     0
     1


fval =

   -10


exitflag =

     1

output = 

        relativegap: 0
        absolutegap: 0
      numfeaspoints: 1
           numnodes: 0
    constrviolation: 0
            message: 'Optimal solution found.…'

%}
%}

% function X=ILP_for_best_match(s,pattern_length,no_of_plants)

no_of_plants=2;
pattern_length(1)=4;
pattern_length(2)=4;
s{1,1}='1110'; s{2,1}='1010';  % contains uniform pattern
s{1,2}=1; s{2,2}=2;  % contains period (ms)
s{1,3}=0.5; s{2,3}=1.25;   % contains wect (ms)

% #variables = len(pattern_1) + len(pattern_2) + 1(slack variable) 

no_of_variables=1;
for i=1:no_of_plants
    no_of_variables=no_of_variables+pattern_length(i);
end

% Make Equality Constraints
% -------------------------
% Constraints are : no. of 1 for each pattern remains same
% and slack varible is always equal to 1

A_eq=zeros(no_of_plants+1,no_of_variables);
b_eq=zeros(no_of_plants+1,1);
shift=0;

for i=1:no_of_plants 
    
    % Fill A_eq
    for j=1:pattern_length(i)
        A_eq(i,shift+j)=1;
    end
    shift=pattern_length(i);
    
    % Fill b_eq
    [~, no_of_that_bit]=size(strfind(s{i,1},'1'));
    b_eq(i)=no_of_that_bit;
end
% last row for slack variable

A_eq(i+1,no_of_variables)=1;  
b_eq(i+1)=1;                  

% Make Inequality Constraints
%------------------------------
% types of constraints= nC1 + nC2 + ... + nCn
% Each type from nC1 and nCn gives only one constraint
% Each type from nC2 to nC(n-1) gives t_B/lcm(those choices) no. of
% constraints

%no_of_sets_of_cons=2^(no_of_plants-1); % avioding nCo case 

t_B=1;
for i=1:no_of_plants
      t_B=lcm(t_B,pattern_length(i)*s{i,2});
end

% Constraints over relative deadline of each task. Each occurance of a task
% makes one constraints. No. of occurances of a task=pattern_length of that task.

total_deadline_cons=sum(pattern_length);  

total_bandwidth_cons=zeros(1,no_of_plants-1);
for i=2:no_of_plants      % from nC2 to nC(n-1)- total n-1 types
    all_comb_of_nCi=nchoosek(1:no_of_plants,i);
    [all_comb,~]=size(all_comb_of_nCi);
    for j=1:all_comb
        LCM=1;
        for k=1:i
            LCM=lcm(LCM,s{all_comb_of_nCi(j,k),2});
        end
        total_bandwidth_cons(i-1)=total_bandwidth_cons(i-1)+(t_B/LCM);
    end
end 
total_constraints=total_deadline_cons+sum(total_bandwidth_cons);

A_ineq=zeros(total_constraints,no_of_variables);
b_ineq=zeros(total_constraints,1);

% Constraints for Deadline Requirements i.e, for nC1 choices
count=1;
shift=0;
while(count<total_deadline_cons)
    for i=1:no_of_plants
        for j=1:pattern_length(i) 
            A_ineq(count,shift+j)=s{i,3};
            b_ineq(count)=s{i,2};
            count=count+1;
        end
        shift=shift+pattern_length(i);
    end
end

% Constraints for Bandwidth Requirements

%count=total_deadline_cons+1;  % since all deadline cons are already formulated

count=1;
while(count<=sum(total_bandwidth_cons))
	for k=2:no_of_plants      % from nC2 to nC(n-1)- total n-1 types
		
		arrival_count_within_T=zeros(1,k);  % keeps total arrival count of k no. of loops within T
		pattern_looping_within_T=zeros(1,k); % keeps total patterns repeatation count of k no. of loops within T
		%arrival_indices_within_T=cell(no_of_plants,1);
		arrival_indices_within_T={};   % tracks the arrival indices of a loop within T
		cnt=1;
		shift=0;
		arrival_at_previous_T=cell(k,1);
		arrival_at_previous_T{1}=[];
		arrival_at_previous_T{2}=[];
		
		all_comb_of_nCk=nchoosek(1:no_of_plants,k);
		[total_comb,~]=size(all_comb_of_nCk);
		for j=1:total_comb
			LCM=1;
			for p=1:k
				LCM=lcm(LCM,s{all_comb_of_nCi(j,p),2});
			end		
		end
		T_end=LCM

	% Constructing one constraint
	   for i=1:no_of_plants
			 arrival_indices=[];
			 arrival_count_within_T(i)= T_end/s{i,2}  % s{i,2} is period
			 pattern_looping_within_T(i)=floor(arrival_count_within_T(i)/pattern_length(i))
			 rest_arrivals=mod(arrival_count_within_T(i),pattern_length(i));
			 if pattern_looping_within_T(i)~=0    % If pattern loops within T
				 for j=1:pattern_looping_within_T(i)  % For each looping find its respective arrival indices
					for k=1:pattern_length(i)
						arrival_indices(cnt)=k;
						cnt=cnt+1;
					end
				 end
			 end
			 if rest_arrivals~=0   % If some arrival indices exist, which do not complete a pattern within T
				 for k=1:rest_arrivals
				   arrival_indices(cnt)=k;
				   cnt=cnt+1;
				 end     
			 end
			 cnt=1; 
			 arrival_indices_within_T{i,count}=arrival_indices;  % Stores the entire arrival pattern of i-th task upto T
			 current_arrivals=arrival_indices(length(arrival_at_previous_T{i})+1 : length(arrival_indices)); % extracting current arrivals
			 
			 for k=1:length(current_arrivals)
				A_ineq(total_deadline_cons+count,shift+current_arrivals(k))=s{i,3}; %put respective wcet against each arrival
			 end
			 arrival_at_previous_T{i}=arrival_indices_within_T{i,count}; % save the currrent occurances as previous
			 shift=shift+pattern_length(i);  % shift for pattern length times of current loop to reach at starting position of next loop
	   end
	   b_ineq(total_deadline_cons+count)=LCM;
	   T_end=T_end+LCM;  
	   count=count+1;  
	   shift=0;
	end %for
end %while

% Specifying integer varibales

intcon=zeros(no_of_variables,1);
for j=1:no_of_variables
    intcon(j)=j;    
end

% Specifying Upper nad Lower Bounds of each variables

lb = zeros(no_of_variables,1);
ub=ones(no_of_variables,1);

% Make Objective Function
%----------------------------
% For the Match Cost = -2 and misMatch Cost = 1

f=zeros(no_of_variables,1);

no_of_1=zeros(no_of_plants,1);
no_of_0=zeros(no_of_plants,1);
cost_1=1;   
cost_0=-2;  
cnt=1;
slack_value=0;

for i=1:no_of_plants
    if s{i,1}(1)=='1'  % For 1st bit position..To avoid mismatch we set higher cost (4/-4) for it.
        f(cnt)=-4;
        slack_value=slack_value+1; % Since cost of mismatching of first bit is 2
    elseif s{i,1}(1)=='0'
        f(cnt)=4;
    end
    cnt=cnt+1;
    for j=2:pattern_length(i)
        if s{i,1}(j)=='1'
            f(cnt)=-3;
        elseif s{i,1}(j)=='0'
            f(cnt)=3;
        end
        cnt=cnt+1;
    end
    [~,no_of_that_bit]=size(strfind(s{i,1},'1'));
    no_of_1(i)=no_of_that_bit;
    [~,no_of_that_bit]=size(strfind(s{i,1},'0'));
    no_of_0(i)=no_of_that_bit;
    
    slack_value=slack_value+no_of_1(i)*cost_1+no_of_0(i)*cost_0;
    
end
% cnt
% slack_value
f(cnt)= slack_value;

[x,fval]=intlinprog(f,intcon,A_ineq,b_ineq,A_eq,b_eq,lb,ub);
X=cell(1,no_of_plants);
if (~isempty(x))
    j=1;
%     disp('The Minimally Deviated Permutation is: ')
    for i=1:no_of_plants
        x(j:j+pattern_length(i)-1)'          % displaying the pattern
        X{1,i}=x(j:j+pattern_length(i)-1);
        j=j+pattern_length(i);
    end
%     disp('Optimal Cost of the above solution is:')
    fval    
end

% A_ineq
% A_eq
% b_ineq
% b_eq
