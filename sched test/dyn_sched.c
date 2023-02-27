# include <stdio.h>
# include <stdlib.h>

typedef struct automaton{
    int task_id;
    int h;
    int max_miss;
    int* clf_states;
    int* mlf_states;
    int pat_len;
    int** create_transitions(){
        int 
    }
    } automaton;

automaton* new_automaton(int task_id, int h, int max_miss, int** transitions){



typedef struct task{
    int id;
    double h;
    double c;
    double o;
    double rel_dl;
    double* h_list;
    automaton* automata;
    } task;

task* new_task(int id, double h, double c, double o, double rel_dl, double* h_list, automaton* automata){
 task* t = (task*)malloc(sizeof(task));
 t->id = id;
 t->h = h;
 t->c = c;
 t->o = o;
 t->rel_dl = rel_dl;
 t->h_list = h_list;
 t->automata = automata;
 return t;
}


int main(){


}