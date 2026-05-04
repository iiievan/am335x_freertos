#ifndef INIT_H
#define INIT_H

#include "stdint.h"
#ifdef __cplusplus
extern "C" {
#endif

bool init_board(void);
void delay_ms(const uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* INIT_H */