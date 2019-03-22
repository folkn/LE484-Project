#include "bsp.h"
#include "timer.h"

// exported global variables
volatile int Button_Status = 0;

/** @brief  Initialize I/O for lamps
    @retval None
*/
void BSP_Init(void)
{
}

/** @brief  Change lamps to GREEN
    @retval None
*/
void Signal_Pass(void) //Car Pass
{
    RED_OFF();
		GREEN_ON();
}

/** @brief  Change lamps to RED
    @retval None
*/
void Signal_Block(void)
{
    RED_ON();
		GREEN_OFF();
}

/** @brief  Flash YELLOW lamp
    @retval None
*/
void Signal_Flash(void)
{
}
