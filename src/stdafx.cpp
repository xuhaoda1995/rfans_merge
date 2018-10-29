#include "stdafx.h"

using namespace global_param;

const int R_FANS_LINE32=32;
const int R_FANS_LINE16=16;

const int OFFSET_IDX32[4]={0,55,14,70};
const int OFFSET_IDX16[2]={0,1};

const float laser_vangle32[32]={
   -20.5, -19.5, -18.5, -17.5,
   -16.5, -15.5, -14.5, -13.5,
   -12.5, -11.5, -10.5,  -9.5,
   -8.5,  -7.5,  -6.5,  -5.5,
   -4.5,  -3.5,  -2.5,  -1.5,
   -0.5,   0.5,   1.5,   2.5,
    3.5,   4.5,   5.5,   6.5,
    7.5,   8.5,   9.5,   10.5};

int global_param::get_idx_with_offset(int idx_beam, int idx_sweep, int num_sweep)
{
    int idx = idx_sweep + OFFSET_IDX32[idx_beam % 4];
    if (idx >= num_sweep)
        idx -= num_sweep;
    return get_idx(idx_beam, idx);
}