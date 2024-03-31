#include "stdio.h"
#include "stdlib.h"


int main(int argc, char *argv[]){

    long p_a, width = 200, p_b;
    p_a = 1;

    FILE *gnuplot = popen("gnuplot", "w");
    for (long simcounter = 1; simcounter < 2000;++simcounter){

        if(simcounter < width){
            p_b = simcounter;
        }
        else{
            p_a++;
            p_b = p_a + width;
        }

        fprintf(gnuplot, "plot for [col=1:12] '/home/randy/HDSRL/data.dat' using col every ::%ld::%ld with lines notitle\n", p_a, p_b);
        fflush(gnuplot);
    }

    fprintf(gnuplot, "exit \n");
    pclose(gnuplot);

    return 0;
}
