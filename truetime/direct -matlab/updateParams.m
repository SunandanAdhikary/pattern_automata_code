function [System] = updateParams(System, Jnew)

      name = System.name;
      %------------------------%
      if Jnew > System.Jth
        disp("-------"+name+" more-------");
        if System.hidx > 1 && System.totalutil < 1
            System.hidx = System.hidx-1;
            System.h = System.hset(System.hidx);
            System.util = getUtil(System);
            System.acess = getAcess(System, System.util, 0);
            ttSetPeriod(System.h, System.task);
            ttSetData(System.task, System);
        else
            disp(name+" no way 2 incr");
        end
      elseif Jnew < System.Jth*0.3
          disp("--------"+name+" less-------");
          red_ratio = 1/System.totalutil;
          if System.hidx < length(System.hset)
            System.hidx = System.hidx+1;
            System.h = System.hset(System.hidx);
            System.util = getUtil(System);
            System.acess = getAcess(System, System.util*red_ratio, 1);
            ttSetPeriod(System.h, System.task);
            ttSetData(System.task, System);
         else
            disp(name+" no way 2 red");
          end
      else
          disp (name +" ok");
      end
      disp (name +" period");
      ttCreateJob(System.task, ttCurrentTime+System.h);
    %------------------------%
end