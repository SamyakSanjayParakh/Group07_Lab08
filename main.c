

/**
 * main.c
 */
#include  <stdint.h>

#include "tm4c123gh6pm.h"
int main(void)
{
    SYSCTL_RCGCI2C_R =1;
    SYSCTL_RCGCGPIO_R=2;
    GPIO_PORTB_AFSEL_R=12;//reviwe
    GPIO_PORTB_ODR_R=8;//
    GPIO_PORTB_ODR_R=0x00003300;
    I2C0_MCR_R=0x00000010;
    I2C0_MTPR_R=0x00000009;
    I2C0_MSA_R =0x076;//for 3B
    I2C0_MDR_R=0X000000EE;
    I2C0_MCS_R =7;





    return 0;
}
