#include "Tools.h"

int incr_or_max(_Atomic int* n, int max)
{
    int res = *n;
    while ( ! atomic_compare_exchange_weak(n, &res, res+1) )
    {
        if ( res >= max ){
            *n = max;
            return max;
        }
    }

    return res;
}