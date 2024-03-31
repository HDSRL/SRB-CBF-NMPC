#include "stdio.h"
#include "fcntl.h"
#include "unistd.h"

#include "linux/input.h"
#include "sys/stat.h"

#define LOGFILE "/tmp/data"

int main(int argc, char **argv){

    struct input_event ev;

    int k = 1;

    int fd = open("/dev/input/event2", O_RDONLY);

    printf("fd = %d\n", fd);

    while(1){

        size_t mmm = read(fd, &ev, sizeof(ev));


        if( (ev.type == EV_KEY) && (ev.value == 0) ){
            if (ev.code == 103) { 
                printf("k = %d\n", k);
                printf("You pressed the up-arrow\n");
            }
        }
    };

    return 0;
}


