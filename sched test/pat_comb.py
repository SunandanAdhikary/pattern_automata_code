# A Python program to print all 
# permutations using library function 
import itertools
from itertools import permutations 
from itertools import combinations, chain

from matplotlib.cbook import index_of 

def findsubsets(s, n): 
    return list(map(set, itertools.combinations(s, n))) 

def get_uniques(links):    
    my_selection = links
    for l in links:
        if len(l)>1:
            for i in range(1,len(l)):
                sub_list = l[i:] + l[:i]
                if sub_list in links:
                    my_selection.remove(sub_list)
    return my_selection

def removeRateWise(links, rate):
    my_selection = []
    for l in links:
        ones = l.count('1')        
        if float(ones)/len(l)>=rate and ones!=len(l):
            my_selection.append(l)
    return my_selection

####################################################################################################

arr = [101111, 101110, 101011, 101010, 100111, 100110, 100011, 100010]

patternList = []
count = 5
# for j in range(len(arr)):
#     arr1 = findsubsets(arr, j+1)
arr1 = findsubsets(arr, count)
for k in range(len(arr1)):
    perm = list(permutations(arr1[k]))         
    for i in list(perm):             
        patternList.append(list(i))


unique = get_uniques(patternList)
pattern = []
for i in range(len(unique)):
    s = ""
    for element in unique[i]:
        s = s + str(element)
    pattern.append(s)
    
pattern_asmats=[]
for p in pattern:
    p_list = []
    for i in range(len(p)):
        p_list.append(int(p[i]))
    pattern_asmats.append(p_list)
    
   
f = open("pat_out.out", "w")
i = 0
for p in pattern_asmats:
    i = i+1
    f.write(' '.join(map(str,p)))
    if i != len(pattern_asmats):
        f.write("\n")
f.close()
