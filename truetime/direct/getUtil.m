function util = getUtil(sys)
    disp('---util calc---');
    util = (sum(sys.acess,2)*sys.wcet)/(sys.h*sys.ct);
end