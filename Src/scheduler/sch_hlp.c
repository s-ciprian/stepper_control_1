
#include "sch_hlp.h"

inline uint32_t timeDiff(uint32_t ct, uint32_t pt)
{
    if (ct > pt)
    {
        return (ct - pt);
    }
    else
    {
        return (pt - ct);
    }
}
