#include "debug_time.h"

unsigned long millis_of_clock(clock_t t)
{
    return t * 1000 / CLOCKS_PER_SEC;
}

void chr_start_clock(Chrono chr)
{
    gettimeofday(&chr->tm, NULL);
    chr->start = chr->tm.tv_sec * 1000 + chr->tm.tv_usec / 1000;
    chr->duration = 0;
    chr->isPaused = 0;

    if ( chr->fd != -1 ){
        char * str = "last_start, time_since_last_start, duration_since first start\n";
        write(chr->fd, str, strlen(str) * sizeof(char));

        char buf[64];
        // last_start, time since last start, time unpaused since call to start_clock
        //sprintf(buf, "%9ld, 0, 0\n", millis_of_clock(chr->start));
       // write(chr->fd, buf, strlen(buf) * sizeof(char));
    }
}

void chr_pause(Chrono chr)
{
    if ( ! chr->isPaused ){
        gettimeofday(&chr->tm, NULL);
        unsigned long time_start = chr->tm.tv_sec * 1000 + chr->tm.tv_usec / 1000 - chr->start;
        
        chr->duration += time_start;
        chr->isPaused = 1;

        if ( chr->fd != -1 ){
            char buf[128];
            // last_start, time since last start, time unpaused since call to start_clock
            sprintf(buf, "%9ld, %9ld, %9ld\n", chr->start, time_start, chr->duration);
            //printf("%s", buf);
            //printf("%ld\n", write(chr->fd, buf, strlen(buf) * sizeof(char)));
            write(chr->fd, buf, strlen(buf) * sizeof(char));
        }
    }
}

void chr_restart(Chrono chr)
{
    if ( chr->isPaused ){
        if ( chr->fd != -1 ){
            char buf[128];
            //printf("mark\n");
            //sprintf(buf, "%9ld, 0, %9ld\n", chr->start, chr->duration);
            //write(chr->fd, buf, strlen(buf) * sizeof(char));
        }

        gettimeofday(&chr->tm, NULL);
        chr->start = chr->tm.tv_sec * 1000 + chr->tm.tv_usec / 1000;
        chr->isPaused = 0;
    }
}

void chr_stop(Chrono chr)
{
    chr_pause(chr);

    chr->duration = 0;
}

void chr_assign_log(Chrono chr, char* filepath)
{
    chr_close_log(chr);

    chr->fd = open(filepath, O_WRONLY | O_CREAT | O_TRUNC, 0666);
}

void chr_close_log(Chrono chr)
{
    if ( chr->fd != -1 ){
        fsync(chr->fd);
        close(chr->fd);
    }
}