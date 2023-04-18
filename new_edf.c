# include <stdio.h>
# include <stdlib.h>
# include <time.h>
// declare structs
typedef struct task task;
typedef struct minheap minheap;
typedef struct system sys;
typedef struct automaton automaton;

// define structs
// task params 
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
// task ready queue : min heap
typedef struct minheap{
    int size;
    int capacity;
    task** q;
    minheap* (*new_minheap)(int capacity);
    void (*insert)(minheap* self, task* t);
    task* (*extract_min)(minheap* self);
    void (*heapify)(minheap* self, int i);
    void (*heap_sort)(minheap* self);
    } minheap;
// declare functions
void swap (const void* a, const void* b);
int gcd(int a, int b);
int find_hyperperiod(task* taskset, int task_ct);
int compare(const void* a, const void* b);
int is_schedulable(task* taskset, int task_ct, int hyperperiod, int** chosen_pats, double* util);
void edf_scheduler(task* taskset, int* task_count, int** chosen_pats, int* job_ct, int hyperperiod, int* schedule_task, int* schedule_start);
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
    int job_ct = 0;
    double util = 0;
    task ts[5]={
        {1, 1, 20, 5, 0, 20, 0},// NULL, NULL},
        {2, 1, 30, 5, 0, 30, 0},// NULL, NULL},
        {3, 1, 60, 5, 0, 60, 0},// NULL, NULL},
        {4, 1, 40, 10, 0, 40, 0},// NULL, NULL},
        {5, 1, 60, 15, 0, 60, 0}//, NULL, NULL},
    };
    task* taskset = (task*)malloc(sizeof(int)*30);
    for (int i = 0; i < task_ct; i++)
    {
        taskset[i] = ts[i];
    }
    
    // int* pats=(int*)malloc(sizeof(int)*30);
    int pats[5][6] = {{1,1,1,1,1,1},
            {1,1,1,1},
            {1,1},
            {1,1,1},
            {1,1}};
    int** chosen_pats = (int**)malloc(sizeof(int*)*5);
    for(int i=0;i<5;i++){
        chosen_pats[i] = (int*)malloc(sizeof(int)*6);
    }
    for(int i=0; i<5; i++){
        for(int j=0; j<6; j++){
            chosen_pats[i][j] = pats[i][j];
        }
    }
    int hyperperiod = find_hyperperiod(taskset, task_ct);
    // clock_t start, end;
    double cpu_time_used;
    // start = clock();
    job_ct = is_schedulable(taskset, task_ct, hyperperiod, chosen_pats, &util);
    printf("job_ct: %d\n", job_ct);
    if (job_ct == 0){
        printf("not schedulable \n");
        return 0;
    }
    int* schedule_task = (int*)malloc(sizeof(int)*(job_ct));
    int* schedule_start = (int*)malloc(sizeof(int)*(job_ct));
    edf_scheduler(taskset, &task_ct, chosen_pats, &job_ct, hyperperiod, schedule_task, schedule_start);
    // end = clock();
    // cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("time: %f\n", cpu_time_used);
    // int *job_ct = (int)sizeof(schedule_task)/sizeof(schedule_task[0]);
    // printf("*job_ct: %d\n", *job_ct);
    // while(**job_ct > 0){
    //     printf("task %d at start time: %d \n", &schedule_task[**job_ct], &schedule_start[**job_ct]);
    //     (**job_ct)--;
    // }
}

//define functions
minheap* new_minheap(int capacity){
    // printf("new minheap with capacity %d \n", capacity);
    minheap* self = (minheap*)malloc(sizeof(minheap));
    self->size = 0;
    self->capacity = capacity;
    self->q = (task**)malloc(sizeof(task*)*capacity);
    printf("new minheap done with capacity %d and size %d\n", self->capacity, self->size);
    return self;
}

// minheap* init_heap(minheap* self, task* tasks, int* task_count){
//     printf("init heap \n");
//     for(int i = 0, i < *task_count; i++){
//         self->insert(self, &tasks[i]);
//     }
// }

// for swapping elements
void swap (const void* a, const void* b){
    printf("swap \n");
    const void* temp = a;
    a = b;
    b = temp;
}

// for heapifying
void heapify(minheap *self, int i){
    printf("heapifying index %d in heap of size %d\n",i, self->size);
    int left = 2*i+1;
    int right = 2*i+2;
    int min = i;
    if(left < self->size){
        // printf("<left>: %d, right: %d, min: %d \n", left, right, min);
        if(self->q[left]->rem_time < self->q[i]->rem_time){
            min = left;
        }
    }
    if (right < self->size){
        // printf("left: %d, <right>: %d, min: %d \n", left, right, min);
        if(self->q[right]->rem_time < self->q[min]->rem_time){
            min = right;
        }
    }
    if(right < self->size && left < self->size){
        // printf("<min>: %d, i: %d \n", min, i);
        if (min != i){
            swap(&self->q[i], &self->q[min]);
        }
        heapify(self,min);
        i++;
        printf("heapifying index %d done\n",i);
    }else{
        return;
    }
}

void heap_sort(minheap* self){
    printf("heap sorting \n");
    for(int i = self->size/2-1; i >= 0; i--){
        heapify(self, i);
    }
    for(int i = self->size-1; i >= 0; i--){
        swap(&self->q[0], &self->q[i]);
        heapify(self, 0);
    }
}
//     int left = 2*i+1;
//     int right = 2*i+2;
//     int min = i;
//     while (i < self->size)
//     {
//         printf("heapifying index %d in heap of size %d\n",i, self->size);
//         if(self->q[left]->rem_time < self->q[i]->rem_time && left < self->size){
//             printf("<left>: %d, right: %d, min: %d \n", left, right, min);
//             min = left;
//         }
//         if(self->q[right]->rem_time < self->q[min]->rem_time && right < self->size){
//             printf("left: %d, <right>: %d, min: %d \n", left, right, min);
//             min = right;
//         }
//         if (min != i){
//             printf("<min>: %d, i: %d \n", min, i);
//             swap(&self->q[i], &self->q[min]);
//         }
//         i++;
//         printf("heapifying index %d done\n",i);
//     }
// }

// for inserting
void insert(minheap* self, task* t){
    // printf("inserting to size %d and capacity %d \n",self->size, self->capacity);
    if(self->size==self->capacity){
        printf("full heap \n");
        return;
    }else{
        // printf("inserting task id %d  at %d \n", t->id, self->size);
        // self->q[self->size] = *t;
        int i = self->size;
        self->size++;
        // while(i >= 0 && t->rem_time < self->q[i/2-1].rem_time){
        //     self->q[i] = self->q[i/2-1];
        //     i = i/2 - 1;
        // }
        // self->q[i] = (task*)malloc(sizeof(task**));
        // printf("value, size of q: %p and %d \n", self->q+1*(int)sizeof(task*), (int)sizeof(self->q[i]));
        // printf("self->q[i] is %p, task id is %d \n", self->q[i], t->id);
        self->q[i] = t;
        printf("inserting task id %d at %d\n", self->q[i]->id, self->size-1);
        swap(&self->q[i], &self->q[0]);
        heapify(self, 0);
    }
}

task* extract_min(minheap* self){
    task* to_pop = NULL;
    task* to_del = NULL;
    if(self->size == 0){
        printf("empty heap \n");
    }else{
        printf("extracting task id %d with minimum remaining deadline\n", self->q[0]->id);
        to_pop = self->q[0];
        to_del = self->q[self->size-1];
        self->size --;
        self->q[0] = self->q[self->size];
        if (to_del != NULL){
            free(to_del);
        }
        heapify(self, 0);
    }
    return to_pop;
}

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
    for(int i = 0; i < task_ct; i++){
        hyperperiod = hyperperiod * taskset[i].h / gcd(hyperperiod, taskset[i].h);
        // step = gcd(taskset[i].h, step);
    }
    printf("hyperperiod: %d\n", hyperperiod);
    return hyperperiod;
}

int compare_tasks(const void* a, const void* b) {
    printf("comparing tasks \n");
    return (int)((task*)a)->rem_time - ((task*)b)->rem_time;
}

int is_schedulable(task* taskset, int task_ct, int hyperperiod, int** chosen_pats, double* util){
    int job_ct = 0;
    for (int i = 0 ; i<task_ct; i++ ){
        taskset[i].rem_job = hyperperiod/taskset[i].h;
        int rem_job = taskset[i].rem_job;
        // printf("task %d, rem_job: %d \n", taskset[i].id, rem_job);
        for (int k=0; k< taskset[i].rem_job; k++){
            if(chosen_pats[i][k] == 0){
                taskset[i].rem_job--;
            }
            // printf("task %d, job %d, pattern %d \n", i, k, chosen_pats[i][k]);
        }
        job_ct += taskset[i].rem_job;
        *util += taskset[i].rem_job*taskset[i].c;
        printf("--%d jobs to do for task %d \n", taskset[i].rem_job, taskset[i].id);
    }
    *util = *util / (double) hyperperiod;
    // printf("util: %f, job_ct: %d\n", *util, job_ct);
    if (*util > 1.00){
        printf("not scheduleable\n");
        return 0;
    }else{
        printf("scheduleable with utilization: %f\n", *util);
        return job_ct;
    }
}

// edf scheduler for a set of tasks following automaton of each task
// int* edf_scheduler(task taskset[], int task_ct, int* *job_ct){
void edf_scheduler(task* taskset, int* task_ct, int** chosen_pats, int* job_ct, int hyperperiod, int* schedule_task, int* schedule_start){
    // int chosen_pats= **chosen_pats;
    // int hyperperiod = find_hyperperiod(taskset, task_ct);
    printf("hyperperiod: %d, job count: %d\n", hyperperiod, *job_ct);
    // if (is_schedulable(taskset, task_ct, hyperperiod, chosen_pats) == 0){
    //     return 0;
    // }else{
    //     *job_ct = is_schedulable(taskset, task_ct, hyperperiod, chosen_pats);
    // }
    int time = 0;
    int i = 0;
    int job_i = 0;
    minheap *ready_q = new_minheap(*job_ct);
    // int* schedule_task = (int*)malloc(sizeof(int)*(*job_ct));
    // int* schedule_start = (int*)malloc(sizeof(int)*(*job_ct));
    while (job_i < *job_ct && time < hyperperiod){
        printf("time: %d \n", time);
        for (int i = 0 ; i< *task_ct; i++){
                taskset[i].rem_time = taskset[i].h-(time % taskset[i].h);
                // printf("new job of task %d has arrived at %d 1\n",taskset[i].id, time);
            if (time == 0 || time % taskset[i].h == 0){
                // printf("new job of task %d has arrived at %d 2\n",taskset[i].id, time);
                if (time == 0 || taskset[i].rem_time == 1e9){
                    // printf("new job of task %d will execute if chosen_pats = %d \n",taskset[i].id, chosen_pats[taskset[i].id][(time/taskset[i].h)]);
                    if (chosen_pats[taskset[i].id][(time/taskset[i].h)]){
                        task* t = &taskset[i];
                        printf("new job of task %d has arrived ", t->id);
                        insert(ready_q, t);
                        printf("and put in ready q at %d\n", time);
                    }else{
                        printf("new job of task %d has arrived but not put in ready q at %d due to drop in pattern \n", taskset[i].id, time);
                    }
                }else{
                    // skip next

                    printf("not putting new job of task %d arrived at %d in ready queue since the last deadline is missed as %d time is remaining \n", taskset[i].id, time,taskset[i].rem_time);
                }
            }
            printf("--at %d time task %d has remaining time %d\n", time, taskset[i].id, taskset[i].rem_time);
        }
        heap_sort(ready_q);
        if(ready_q->size == 0){
            printf("no task in ready queue at %d\n",time++);
            continue;
        }else{
            task *t = extract_min(ready_q);
            if (t->rem_job > 0){
                if (t->rem_time >= t->c && t->rem_job != 1e9){
                    schedule_task[job_i] = t->id;
                    schedule_start[job_i] = time;
                    t->rem_job -= 1;
                    t->rem_time = 1e9;
                    job_i += 1;
                    printf("task %d, start %d, ", t->id, time);
                    time += t->c;
                    printf("end %d\n", time);
                }else{
                    // kill job
                    printf("at %d time task %d has missed deadline\n", time, t->id);
                    time += t->rem_time;
                }
            }else{
                printf("no task/idle at %d\n",time++);
            }
        }
    }
    return;
}