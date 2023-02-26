#include <stdio.h>

#define MAX_TASKS 10

struct task {
    int id;
    int period;
    int exec_time;
    int deadline;
    // int remaining job;
    int remaining_time;
};

void edf(struct task tasks[], int n);

int main() {
    struct task tasks[MAX_TASKS] = {
        {1, 20, 5, 20, 20},
        {2, 30, 5, 30, 30},
        {3, 60, 5, 60, 60},
        {4, 40, 10, 40, 40},
        {5, 60, 15, 60, 60}
    };
    int n = 5;

    edf(tasks, n);

    return 0;
}

void edf(struct task tasks[], int n) {
    int time = 0;
    struct task *min_deadline_task = NULL;

    while (1) {
        // Find the task with the earliest deadline among the remaining tasks
        min_deadline_task = NULL;
        for (int i = 0; i < n; i++) {
            if (tasks[i].remaining_time > 0) {
                if (min_deadline_task == NULL || tasks[i].deadline < min_deadline_task->deadline) {
                    min_deadline_task = &tasks[i];
                }
            }
        }

        // If all tasks have completed, exit the loop
        if (min_deadline_task == NULL) {
            break;
        }

        // Run the task with the earliest deadline
        int run_time = 1;
        min_deadline_task->remaining_time -= run_time;
        printf("Scheduled Task %d at time %d\n", min_deadline_task->id, time);

        // Check if the task missed its deadline
        if (min_deadline_task->remaining_time < 0) {
            printf("Task %d missed deadline at time %d\n", min_deadline_task->id, time);
            min_deadline_task->remaining_time = min_deadline_task->period;
        }

        time += run_time;
    }
}
