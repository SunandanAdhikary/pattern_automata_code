function util = getUtil(sys)
    disp('---util calc---');
    sys.totalutil= sys.totalutil - sys.util;
    util = (sum(sys.acess,2)*sys.wcet)/(sys.h*sys.ct);
    sys.totalutil = sys.totalutil + sys.util;
end