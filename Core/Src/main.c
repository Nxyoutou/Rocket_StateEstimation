#include "init.h"
#include "DefFuncs.h"
#include "stdio.h"
#include "string.h"
#include "StateMachine.h"
#include "Time.h"



int main(void)
{
       init();
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
       statemachine_init();
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);

		      
		      
       while(1) 
       {
	       statemachine_run();
	       delay_us(20);
       }


}
