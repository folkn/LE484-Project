#include "ctrler.h"
#include "bsp.h"
#include <stdio.h>
#include "main.h"

Ctrler_State_t Ctrler_State = RESTART;

void Ctrler_Init()
{
}

uint32_t Ctrler_Exec(Event_t evt)
{
    uint32_t timeout_value = 0; //0.1s
    
    switch(Ctrler_State) {
				case RESTART:
					Ctrler_State = THROUGH;
					timeout_value=20;
					Signal_Pass();
					break; 
        case THROUGH:
					if(evt == BUTTON ){ //&& min green time
						Ctrler_State = BLOCKED;
						timeout_value=20;
						Signal_Block();
					}
					//Go to WAIT
            break;
				case BLOCKED:
					if (evt == TIMEOUT){
						Ctrler_State = THROUGH;
						Signal_Pass();
					}
					break;
				case WAIT:
					if(evt == TIMEOUT){
						Ctrler_State = BLOCKED;
					}
					break; 
        default:
            Ctrler_State = OUT_OF_SERVICE;
    }
    return timeout_value;
}
