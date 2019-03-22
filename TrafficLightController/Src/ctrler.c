#include "ctrler.h"
#include "bsp.h"

Ctrler_State_t Ctrler_State = RESTART;

void Ctrler_Init()
{
}

uint32_t Ctrler_Exec(Event_t evt)
{
    uint32_t timeout_value = 0; //0.1s
    
    switch(Ctrler_State) {
        case RESTART:
					if(evt == BUTTON){
						Ctrler_State = BLOCKED;
						timeout_value=20;
						Signal_Block();
					}
            break;
				case BLOCKED:
					if (evt == TIMEOUT){
						Ctrler_State = RESTART;
						Signal_Pass();
					}
					break;
        default:
            Ctrler_State = OUT_OF_SERVICE;
    }
    return timeout_value;
}
