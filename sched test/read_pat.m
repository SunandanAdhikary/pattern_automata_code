system ="trajectory";
lines = 1344;
minatklen = zeros(1,lines);
n = 30;
subseqs={};
fileID = fopen('pat_out.out','r');
for i=1:lines
    subseq = fscanf(fileID,'%1d\t',n)'
    [finalpat,minatklen(i)]=find_respat(system,subseq);
    subseqs{i}=subseq;
end
fclose(fileID);
max_minatklen=0;
j=0;
[max_minatklen,j] = max(minatklen);
subseqs{j}