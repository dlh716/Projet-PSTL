#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

struct chrono{
    struct timeval tm;
    int fd;
    clock_t start, duration;
    short isPaused;
};

typedef struct chrono* Chrono;

void chr_start_clock(Chrono chr);
void chr_pause(Chrono chr);
void chr_restart(Chrono chr);
void chr_stop(Chrono chr);
void chr_assign_log(Chrono chr, char* filepath);
void chr_close_log(Chrono chr);
