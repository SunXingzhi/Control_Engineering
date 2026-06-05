#include "PID.h"

typedef struct {
    float SumError;
    float DError;
} PID_PosState_t;

// 位置式公式
// Output = Kp*Error + Ki*SumError + Kd*DError

// const PID_AlgoInterface_t PID_POSITIONAL_OPS = {
//     .init    = pos_init,
//     .calc    = pos_calc,
//     .reset   = pos_reset,
//     .destroy = free,
// };



