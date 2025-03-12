#include "debug_time.h"

unsigned long millis_of_clock(clock_t t)
{
    return t * 1000 / CLOCKS_PER_SEC;
}

void chr_start_clock(Chrono chr)
{
    chr->start = clock();
    chr->duration = 0;
    chr->isPaused = 0;

    if ( chr->fd != -1 ){
        char * str = "last_start, time_since_last_start, duration_since first start\n";
        write(chr->fd, str, strlen(str) * sizeof(char));

        char buf[64];
        // last_start, time since last start, time unpaused since call to start_clock
        sprintf(buf, "%9ld, 0, 0\n", millis_of_clock(chr->start));
        write(chr->fd, buf, strlen(buf) * sizeof(char));
    }
}

void chr_pause(Chrono chr)
{
    if ( ! chr->isPaused ){
        unsigned long time_start = millis_of_clock(clock() - chr->start);
        
        chr->duration += time_start;
        chr->isPaused = 1;

        if ( chr->fd != -1 ){
            char buf[128];
            // last_start, time since last start, time unpaused since call to start_clock
            sprintf(buf, "%9ld, %9ld, %9ld\n", millis_of_clock(chr->start), time_start, chr->duration);
            write(chr->fd, buf, strlen(buf) * sizeof(char));
        }
    }
}

void chr_restart(Chrono chr)
{
    if ( chr->isPaused ){
        if ( chr->fd != -1 ){
            char buf[128];
            sprintf(buf, "%9ld, 0, %9ld\n", millis_of_clock(chr->start), chr->duration);
            write(chr->fd, buf, strlen(buf) * sizeof(char));
        }

        chr->start = clock();
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

    chr->fd = open(filepath, O_WRONLY | O_CREAT, 0666);
}

void chr_close_log(Chrono chr)
{
    if ( chr->fd != -1 ){
        close(chr->fd);
    }
}