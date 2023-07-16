/**
 * @file main.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Show importance of volatile qualifier when using code optimization
 * @version 0.1
 * @date 2022-06-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define SRAM_ADDRESS1       0x20000004U

int main(void) {
    uint32_t value = 0 ;
    uint32_t volatile *p ;
    p = (uint32_t*) SRAM_ADDRESS1 ;

    while (1) {
        value = *p ;
        if (value) {
            break ;
        }
    }

    /* Loop forever */
    while (1) ;
}
