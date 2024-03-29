# include <stdio.h>
# include <stdlib.h>
# include <time.h>
# include <stdbool.h>
// declare structs
typedef struct task task;
typedef struct system sys;
typedef struct automaton automaton;

// define structs
//task params 
typedef struct task{
    int id;
    int criticality;
    int h;
    int c;
    int o;
    int rem_time;
    int rem_job;
    // int* h_list;
    // automaton* automata;
    task* (*new_task)(int id, int h, int c, int o, int rem_job, int* h_list, automaton* automata);
    } task;
typedef struct system{
    int max_miss;
    // int** A;
    // int** B;
    // int** C;
    // int** D;
    // int** K;
    // int** L;
    int h;
    int state_ct;
    int input_ct;
    int output_ct;
    int*** Am;
    int* x0;
    int* safex;
    double gamma;
    sys* (*new_sys)(int max_miss, int h, int states, int inputs, int outputs, int*** Am, int* x0);
    double* (*pat_simulate)(sys* self, int len, int* pat);
    } sys;
// pattern generator automaton for a control new_task
// for certain periodicity, length of pattern, and maximum number of misses
typedef struct automaton{
    int task_id;
    int h;
    int s0;
    sys* plant;
    int clf_states;
    int* mlf_dwelltimes;
    int pat_len;
    int** transitions;
    int** (*create_transitions)(automaton* self, int pat_len, int h, int max_miss);
    int** (*gen_pats)(automaton* self, int pat_len, int** transitions);
    automaton* (*new_automaton)(int task_id, int h, int max_miss, int** transitions);
    } automaton;
// declare functions
int gcd(int a, int b);
int find_hyperperiod(task* taskset, int task_ct);
int compare(const void* a, const void* b);
int* scheduler(task* taskset, int task_ct);
// int* edf_scheduler(task* taskset, int task_ct, int* job_ct);
int* edf_scheduler(task taskset[], int task_ct, int* job_ct, int** chosen_pats);
//main
int main(){
    // int id;
    // int criticality;
    // int h;
    // int c;
    // int o;
    // int rem_time;
    // int rem_job;
    // int* h_list;
    // automaton* automata;
    int task_ct = 5;
    int job_ct =0;
    task taskset[5]={
        {1, 1, 20, 5, 0, 20, 0},// NULL, NULL},
        {2, 1, 30, 5, 0, 30, 0},// NULL, NULL},
        {3, 1, 60, 5, 0, 60, 0},// NULL, NULL},
        {4, 1, 40, 10, 0, 40, 0},// NULL, NULL},
        {5, 1, 60, 15, 0, 60, 0}//, NULL, NULL},
    };
    int chosen_pats[5][6] = {
            {1,1,1,1,1,1},
            {1,0,1,0},
            {1,1},
            {1,0,1},
            {1,1}
    };
    // int hyperperiod = find_hyperperiod(taskset, task_ct);
    clock_t start, end;
    double cpu_time_used;
    start = clock();
    // int *schedule_task, *schedule_start = edf_scheduler(taskset, task_ct, &job_ct);
    int *schedule_task, *schedule_start = edf_scheduler(taskset, task_ct, &job_ct, &chosen_pats);
    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("time: %f\n", cpu_time_used);
    // int job_ct = (int)sizeof(schedule_task)/sizeof(schedule_task[0]);
    // printf("job_ct: %d\n", job_ct);
    // while(*job_ct > 0){
    //     printf("task %d at start time: %d \n", &schedule_task[*job_ct], &schedule_start[*job_ct]);
    //     (*job_ct)--;
    // }
    printf("job_ct: %d\n", job_ct);
}

//define functions

sys* new_sys(int max_miss, int h, int states, int inputs, int outputs, int*** Am, int* x0){
        sys* self = (sys*)malloc(sizeof(sys));
        self->max_miss = max_miss;
        self->h = h;
        self->state_ct = states;
        self->input_ct = inputs;
        self->output_ct = outputs;
        self->Am = Am;
        self->x0 = x0;
        return self;
    }

int** create_transitions(automaton* self, int pat_len, int h, int max_miss){
    int** transitions = (int**)malloc(sizeof(int*)*pat_len);
    // ruless
    return transitions;
}

automaton* new_automaton(int task_id, int h, sys* plant, int** transitions){
    automaton* self = (automaton*)malloc(sizeof(automaton));
    self->task_id = task_id;
    self->h = h;
    self->plant = plant;
    self->transitions = self->create_transitions(self, self->pat_len, self->h, self->plant->max_miss);
    return self;
}

// task* new_task(int id, int criticality, int h, int c, int o, int* h_list, automaton* automata){
//     task* t = (task*)malloc(sizeof(task));
//     t->id = id;
//     t-> criticality = criticality;
//     t->h = h;
//     t->c = c;
//     t->o = o;
//     t->rem_time = h;
//     t->rem_job = 0;
//     t->h_list = h_list;
//     t->automata = automata;
//     return t;
// }

double pat_simulate(sys* self, int len, int* pat){
    double how_safe = self->safex-self->x0;
    double del_e = 0;
    int* x = (int*)malloc(sizeof(int)*self->state_ct);
    for(int i=0; i++; i< len){

    }
    return how_safe, del_e;
}

int** gen_pats(automaton* self, int pat_len, int** transitions){
    int i = 0;
    int** pat_list = NULL;
    int* pat = (int*)malloc(sizeof(int)*pat_len);
    int len = self->pat_len;
    int min_clf_miss_idx = self->plant->max_miss-1;
    for(int i=0; i++; i < self->plant->max_miss){
        if ((min_clf_miss_idx+1)*self->mlf_dwelltimes[min_clf_miss_idx] > (i+1)*self->mlf_dwelltimes[i]){
            min_clf_miss_idx = i;
        }
    }
    int s = self->s0;
    while (len){
        if(self->pat_len < (min_clf_miss_idx+1)*self->mlf_dwelltimes[min_clf_miss_idx]){
            pat[len] = i;
            len--;
        }else{
            int s_next = transitions[s][pat[i+1]];
        }
    }
    pat_list[i] = pat;
    return pat_list;
}

// int compare_pats_safety(const void* a, const void* b) {
//     int howsafe, del_e = pat_simulate(a);
//     return (int)((task*)a)->rem_time - ((task*)b)->rem_time;
// }
// int compare_pats_0(const void* a, const void* b) {
//     int howsafe_a = pat_simulate(((task*)a)->automata->plant, ((task*)a)->automata->pat_len, ((task*)a)->automata->pat);
//     return (int)((task*)a)->rem_time - ((task*)b)->rem_time;
// }

int gcd (int a, int b){
    if (b==0){
        return a;
    }
    else{
        if (a > b) return gcd(b, a%b);
        else if (b > a) return gcd(a, b%a);
        else return a;
    }
    // return;
}

int find_hyperperiod(task* taskset, int task_ct){
    int hyperperiod = 1;
    // int step = 1;
    // int job_ct = *job_ct;
    // double utilization = *util;
    for(int i = 0; i < task_ct; i++){
        hyperperiod = hyperperiod * taskset[i].h / gcd(hyperperiod, taskset[i].h);
        // step = gcd(taskset[i].h, step);
    }
    printf("hyperperiod: %d\n", hyperperiod);
    return hyperperiod;
}

int compare_tasks(const void* a, const void* b) {
    return (int)((task*)a)->rem_time - ((task*)b)->rem_time;
}

// double get_util_jobct(task* taskset, int task_ct, double* util, int* job_ct){
//     double u = *util;
//     int ct = *job_ct;
//     for (int i =0 ; i<task_ct; i++){
//         taskset[i].rem_job = hyperperiod/taskset[i].h;
//         *job_ct += taskset[i].rem_job;
//         util += (double)taskset[i].c/taskset[i].h;
//         printf("--%d jobs to do for task %d \n", taskset[i].rem_job, taskset[i].id);
//     }
//     return util;
// }

// edf scheduler for a set of tasks following automaton of each task
// int* edf_scheduler(task taskset[], int task_ct, int* job_ct){
int* edf_scheduler(task taskset[], int task_ct, int* job_ct, int** chosen_pats){
    // int task_ct = (int)(sizeof(taskset)/sizeof(taskset[0]));
    // int* job_ct = 0;
    double util = 0;
    int hyperperiod = find_hyperperiod(taskset, task_ct);
    // measuring utility and total number of jobs for all taks
    // yet to incorporate the automata
    for (int i =0 ; i<task_ct; i++){
        /*
        // taskset[i].automata->pat_len = hyperperiod/taskset[i].h;
        // int** pats = taskset[i].automata->gen_pats(taskset[i].automata->pat_len, taskset[i].automata->transitions);
        // int pat_ct = (int)(sizeof(pats)/sizeof(pats[0]));
        // double max_safe = 0;
        // double max_del_e = 0;
        // int best_pat_idx = 0;
        // for (int j = 0; j<pat_ct; j++){
        //     int how_safe, del_e = taskset[i].automata->plant->pat_simulate(taskset[i].automata->pat_len, pats[j]);
        //     if (max_safe <= how_safe && max_del_e <= del_e){
        //         max_del_e = del_e;
        //         max_safe = how_safe;
        //         best_pat_idx = j;
        //     }
        // }
        // int* chosen_pat = pats[best_pat_idx];
        */
        /*
        int* chosen_pat = chosen_pats[i];
        int rem_job = 0;
        for (int j = 0; j<taskset[i].automata->pat_len; j++){
            if(chosen_pat[j] != 0){
                rem_job++;
            }
        }
        */
        int rem_job = hyperperiod/taskset[i].h;
        taskset[i].rem_job = rem_job;
        // counting and updating the no. of jobs from the chosen pattern for each task
        for (int k=0; k++; k< rem_job){
            if(chosen_pats[i][k] == 0){
                taskset[i].rem_job--;
            }
        }
        // taskset[i].rem_job = hyperperiod/taskset[i].h; //rem_job ;
        *job_ct += taskset[i].rem_job;
        util += taskset[i].rem_job*taskset[i].c;
        // util += (double)taskset[i].c/taskset[i].h;
        printf("--%d jobs to do for task %d \n", taskset[i].rem_job, taskset[i].id);
    }
    util = util/hyperperiod;
    // judging schedulability of the taskset from utility
    if (util>1){
        printf("not scheduleable\n");
        return NULL, NULL;
    }else{
        printf("scheduleable with utilization: %f\n",util);
    }
    // sorting the taskset in ascending order of remaining time
    qsort(taskset,  task_ct, sizeof(task), compare_tasks);
    int time = 0;
    int min_dl_idx=0;
    int job_i = 0;
    task ready_q[task_ct];
    int* schedule_task = (int*)malloc(sizeof(int)*(*job_ct));
    int* schedule_start = (int*)malloc(sizeof(int)*(*job_ct));
    // for (int i=0; i+=step ; i<hyperperiod){
    // scheduling all tasks along the hyperperiod
    while (job_i < *job_ct && time < hyperperiod){
        // taskwise remaining time update
        for (int i =0 ; i<task_ct; i++){
            // update remaining time for tasks that has not yet arrived and is not scheduled
            if (taskset[i].rem_time != 1e9){
                taskset[i].rem_time = taskset[i].h-(time % taskset[i].h);
            }
            // when task arrives update remaining time to its deadline
            if (time == 0 || time % taskset[i].h == 0){
                taskset[i].rem_time = taskset[i].h-(time % taskset[i].h);
                // when task is scheduled before the deadline/arrival of new job
                if (time == 0 || taskset[i].rem_time == 1e9){
                    ready_q[taskset[i].id] = taskset[i];
                    printf("task %d has arrived at %d\n", taskset[i].id, time);
                }else{// when task is not yet scheduled before the deadline/arrival of new job
                    // taskset[i].rem_time = taskset[i].h-(time % taskset[i].h);
                    printf("last deadline missed since time remaining: %d, but new task %d has arrived at %d\n", taskset[i].rem_time, taskset[i].id, time);
                }
            } 
            printf("--at %d time task %d has remaining time %d\n", time, taskset[i].id, taskset[i].rem_time);
        }
        // sorting the ready queue in ascending order of remaining time
        qsort(taskset, task_ct, sizeof(task), compare_tasks);
        min_dl_idx = 0;
        bool can_schedule = taskset[min_dl_idx].rem_job > 0 && chosen_pats[taskset[min_dl_idx].id][(hyperperiod/taskset[min_dl_idx].h)-taskset[min_dl_idx].rem_job] != 0;
        // checking the minimum remaining time task that can be scheduled following the pattern
        while (!can_schedule && min_dl_idx < task_ct){
                min_dl_idx += 1;
                can_schedule = taskset[min_dl_idx].rem_job > 0 && chosen_pats[taskset[min_dl_idx].id][(hyperperiod/taskset[min_dl_idx].h)-taskset[min_dl_idx].rem_job] != 0;
        }
        // scheduling the task with minimum remaining time if permitted by pattern and increase
        if (can_schedule){
            if (taskset[min_dl_idx].rem_time >= taskset[min_dl_idx].c && taskset[min_dl_idx].rem_time != 1e9){
                schedule_task[job_i] = taskset[min_dl_idx].id;
                schedule_start[job_i] = time;
                taskset[min_dl_idx].rem_job -= 1;
                taskset[min_dl_idx].rem_time = 1e9;
                job_i += 1;
                printf("task %d, start %d, ", taskset[min_dl_idx].id, time);
                time += taskset[min_dl_idx].c;
                printf("end %d\n", time);
            }else{// if task misses deadline then do not schedule just increase time
                printf("at %d time task %d has missed deadline\n", time, taskset[min_dl_idx].id);
                time += taskset[min_dl_idx].rem_time;
            }
        }else{// if no task can be scheduled then idle and increate time
            printf("idle at %d\n",time++);
        }
    }
    return schedule_task, schedule_start;
}
