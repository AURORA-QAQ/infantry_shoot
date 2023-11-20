#include "bsp_fric.h"
#include "main.h"
#include "CAN_receive.h"
extern TIM_HandleTypeDef htim1;
//void fric_off(void)
//{
//			
//    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, FRIC_OFF);
//    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, FRIC_OFF);
//}

//void fric1_on(uint16_t cmd)
//{
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,cmd );
//}

//void fric2_on(uint16_t cmd)
//{
//    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,cmd );
//}

void fric_off(void)
{
			
		CAN_cmd_fric(0,0,0,0);

}

void fric1_on(uint16_t cmd)
{
//		  CAN_cmd_fric(1000,1000,0,0);
    CAN_cmd_fric1(cmd);
	
}

void fric2_on(uint16_t cmd)
{
//			CAN_cmd_fric(1000,1000,0,0);
    CAN_cmd_fric2(cmd);

}

