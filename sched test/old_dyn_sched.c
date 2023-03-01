int* scheduler(task taskset[], int task_ct){
    // int task_ct = (int)(sizeof(taskset)/sizeof(taskset[0]));
    int job_ct = 0;
    double util = 0;
    int hyperperiod = find_hyperperiod(taskset, task_ct);
    for (int i =0 ; i<task_ct; i++){
        taskset[i].rem_job = hyperperiod/taskset[i].h;
        job_ct += taskset[i].rem_job;
        util += (double)taskset[i].c/taskset[i].h;
        printf("--%d jobs to do for task %d \n", taskset[i].rem_job, taskset[i].id);
    }
    if (util>1){
        printf("not scheduleable\n");
        return NULL, NULL;
    }else{
        printf("scheduleable with utilization: %f\n",util);
    }
    int time = hyperperiod;
    // qsort(taskset,  task_ct, sizeof(task), compare);
    task* ready_q = NULL;//&taskset[0];
    int* schedule_task = (int*)malloc(sizeof(int)*(job_ct));
    int* schedule_start = (int*)malloc(sizeof(int)*(job_ct));
    int min_dl=1e9;
    int job_i = 0;
    // for (int i=0; i+=step ; i<hyperperiod){
    while (job_i < job_ct && time > 0){
        for (int i =0 ; i<task_ct; i++){
            if (taskset[i].rem_time==-1 && time % taskset[i].h == 0){
                taskset[i].rem_time = taskset[i].h;
                printf("task %d to %d arrived\n", taskset[i].id, taskset[i].rem_time);
            }else if (taskset[i].rem_time < taskset[i].c){
                printf("at %d time task %d has missed deadline\n", (hyperperiod-time), taskset[i].id);
            }else if (taskset[i].rem_time!=-1 && time!=hyperperiod){
                taskset[i].rem_time = (hyperperiod-time) % taskset[i].h;
                printf("at %d time task %d has remaining time %d\n", (hyperperiod-time), taskset[i].id, taskset[i].rem_time);
            }
            if (taskset[i].rem_time >= taskset[i].c && taskset[i].rem_job > 0 && min_dl > time % taskset[i].h ){
                min_dl =time % taskset[i].h;
                ready_q = &taskset[i];
                printf("task %d is ready with minimum dl\n", ready_q->id);
            }
        }
        if (ready_q->rem_job > 0){
            schedule_task[job_i] = ready_q->id;
            schedule_start[job_i] = time;
            ready_q->rem_job -= 1;
            ready_q->rem_time = -1;
            time -= (*ready_q).c;
            job_i += 1;
            // ready_q = NULL;
            printf("task %d, start %d, end %d\n", ready_q->id, hyperperiod-time, hyperperiod-time+(*ready_q).c);
        }else{
            time -= 1;
            printf("---idle, time %d\n", hyperperiod-time);
        }
    }
    return schedule_task, schedule_start;
}