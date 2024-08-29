#include "main.h"

float calculate_heading(float Mx, float My, float Mz, float pitch, float roll) {
    // 倾斜补偿
    float Mx_prime = Mx * cos(pitch) + Mz * sin(pitch);
    float My_prime = Mx * sin(roll) * sin(pitch) + My * cos(roll) - Mz * sin(roll) * cos(pitch);

    // 计算航向角
    float heading = atan2(My_prime, Mx_prime) * 180.0 / M_PI;

    // 确保航向角在 0 到 360 度之间
    if (heading < 0) {
        heading += 360.0;
    }

    return heading;
}




void my_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map){
    //uint16_t * buf16 = (uint16_t *)px_map; /*Let's say it's a 16 bit (RGB565) display*/

    //设置显示窗口
    sDRV_ST7789V_SetWindow(area->x1 + 40, area->y1 + 53, area->x2 + 40, area->y2 + 53);
    // 向ST7789V发送显示数据
    sDRV_ST7789V_SendColorArea(px_map, lv_area_get_width(area) * lv_area_get_height(area));
    // 通知LVGL刷新完成
    lv_display_flush_ready(disp);
}

__attribute__((section(".ramfunc"))) void my_log(lv_log_level_t level, const char * buf){
    sHMI_Debug_Printf("LEVEL:%d :%s",level, buf);
}

#include "lvgl/examples/lv_examples.h"




int main(){
    //初始化系统
    sBSP_SYS_Init();
    //初始化DWT计时单元
    sBSP_DWT_Init(HAL_RCC_GetSysClockFreq());
    //初始化调试串口
    sHMI_Debug_Init(115200);

    uint32_t freq = HAL_RCC_GetSysClockFreq();
    sHMI_Debug_Printf("System Clock: %.0f MHz\n",freq / 1e6);
    sHMI_Debug_Printf("UART Debug text encoding by UTF-8\n\n");


    __GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Pin   = GPIO_PIN_1 | GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB,&gpio);

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);

    HAL_Delay(500);

    sBSP_SPI2M_Init(SPI_BAUDRATEPRESCALER_64);   //11.25MBit/S

    sDRV_ICM_Init();
    sDRV_LPS22_Init();
    sDRV_LIS3_Init();

    sLIB_6AXIS_INPUT_t input = {0};
    sLIB_ATTITUDE_RESULT_t result = {0};

    sLIB_6AXIS_INPUT_t bias = {0};
    

    #define POINT_COUNT 10000
    float acc_x_accu = 0;
	float acc_y_accu = 0;
	float acc_z_accu = 0;
	float gyro_x_accu = 0;
	float gyro_y_accu = 0;
	float gyro_z_accu = 0;
	for(uint16_t i = 0; i < POINT_COUNT; i++){
        sDRV_ICM_GetData();
		acc_x_accu  += g_icm.acc_x;
		acc_y_accu  += g_icm.acc_y;
		acc_z_accu  += g_icm.acc_z;
		gyro_x_accu += g_icm.gyro_x;
		gyro_y_accu += g_icm.gyro_y;
		gyro_z_accu += g_icm.gyro_z;
		//HAL_Delay(1);
	}
	bias.acc_x  = acc_x_accu  / POINT_COUNT;
	bias.acc_y  = acc_y_accu  / POINT_COUNT;
	bias.acc_z  = acc_z_accu  / POINT_COUNT - 9.81398f;	//重力加速度
	bias.gyro_x = gyro_x_accu / POINT_COUNT;
	bias.gyro_y = gyro_y_accu / POINT_COUNT;
	bias.gyro_z = gyro_z_accu / POINT_COUNT;



    sBSP_SPI1M_Init(SPI_BAUDRATEPRESCALER_2);
    sDRV_ST7789V_Init();

    #define DRAW_BUF_SIZE (SDRV_ST7789V_LCDH * SDRV_ST7789V_LCDW / 10 * (LV_COLOR_DEPTH / 8))
    static uint32_t draw_buf[DRAW_BUF_SIZE / 2];

    lv_init();
    lv_tick_set_cb(HAL_GetTick);
    lv_log_register_print_cb(my_log);
    lv_display_t * disp;
    disp = lv_display_create(SDRV_ST7789V_LCDH, SDRV_ST7789V_LCDW);
    lv_display_set_rotation(disp,LV_DISPLAY_ROTATION_0);
    lv_display_set_flush_cb(disp, my_flush_cb);
    lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);


    // lv_obj_t * scale = lv_scale_create(lv_screen_active());
    // lv_obj_set_size(scale, lv_pct(80), 100);
    // lv_scale_set_mode(scale, LV_SCALE_MODE_HORIZONTAL_BOTTOM);
    // lv_obj_center(scale);

    // lv_scale_set_label_show(scale, true);

    // lv_scale_set_total_tick_count(scale, 31);
    // lv_scale_set_major_tick_every(scale, 5);

    // lv_obj_set_style_length(scale, 5, LV_PART_ITEMS);
    // lv_obj_set_style_length(scale, 10, LV_PART_INDICATOR);
    

    lv_obj_t *label = lv_label_create( lv_scr_act() );
    lv_obj_set_style_text_font(label,&lv_font_montserrat_18,0);
    lv_label_set_text( label, "Jie LOVE HHR" );
    lv_obj_align( label, LV_ALIGN_TOP_LEFT, 10, 10 );

    


// #include "lvgl/demos/widgets/lv_demo_widgets.h"
//     lv_demo_widgets();
//     lv_demo_widgets_start_slideshow();
    
    //while(1);
int i = 0;

    while(1){
        sDRV_LIS3_ReadData();
        sDRV_ICM_GetData();
        sDRV_LPS22_GetData();
        input.acc_x = g_icm.acc_x - bias.acc_x;
        input.acc_y = g_icm.acc_y - bias.acc_y;
        input.acc_z = g_icm.acc_z - bias.acc_z;
        input.gyro_x = g_icm.gyro_x - bias.gyro_x;
        input.gyro_y = g_icm.gyro_y - bias.gyro_y;
        input.gyro_z = g_icm.gyro_z - bias.gyro_z;

        sLib_6AxisCompFilter(&input,&result);
        float heading = atan2(g_lis3.mag_y, g_lis3.mag_x) * RAD2DEG;
        heading = 180 + heading;

        if(i > 2){
            i = 0;
            lv_label_set_text_fmt(label,"Pitch: %6.2f\nRoll:%6.2f\nYaw:%6.2f\nMagYaw:%6.2f"\
            ,result.pitch,result.roll,result.yaw,heading);
            //lv_scale_set_range(scale, heading - 60, heading + 60);
            
        
        }else{
            i++;
        }
        
        lv_task_handler(); /* let the GUI do its work */
        HAL_Delay(5);





        

        // //float heading = calculate_heading(g_lis3.mag_z,g_lis3.mag_x,g_lis3.mag_y\
        //         ,result.pitch,result.roll);
        


        // sHMI_Debug_Printf("%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f\n",\
        // input.acc_x,input.acc_y,input.acc_z,\
        // input.gyro_x,input.gyro_y,input.gyro_z, \
        // g_lps22_pa);

        //sHMI_Debug_Printf("%.2f,%.2f,%.2f\n",result.pitch,result.roll,heading);
        //sHMI_Debug_Printf("%.2f,%.2f,%.2f,%.2f\n",g_lis3.mag_x,g_lis3.mag_y,g_lis3.mag_z,heading);



        // 
        // float temp = sDRV_LPS22_GetTemp();
        // float pressure = sDRV_LPS22_GetPress();
        // float alt_m = sLib_Press2Alt(pressure * 100);
        // sHMI_Debug_Printf("Temperature: %.2f C, Pressure: %.4f hPa,alt_m:%.3f m\n",temp,pressure,alt_m);

        //HAL_Delay(20);

        


        

        //sHMI_Debug_Printf("%8.4f,%8.4f,%8.4f,%8.2f,%8.2f,%8.2f,%.2f\n",\
                        input.acc_x,input.acc_y,input.acc_z, \
                        input.gyro_x,input.gyro_y,input.gyro_z \
                        ,g_icm.temp);

        //sHMI_Debug_Printf("%.3f,%.3f,%.3f\n",result.pitch,result.roll,result.yaw);

        HAL_Delay(10);

    }
}



void Error_Handler(){
    __disable_irq();
    while (1){
        sHMI_Debug_Printf("WARNING警告:错误! 禁用IRQ,死循环...\n");
        HAL_Delay(500);
    }
}

void assert_failed(uint8_t* file, uint32_t line){
    __disable_irq();
    while (1){
        sHMI_Debug_Printf("WARNING警告:断言! 文件名:%s,行:%u 禁用IRQ,死循环...\n",file,line);
        HAL_Delay(500);
    }
}

void vApplicationMallocFailedHook(){
    sHMI_Debug_Printf("WARNING警告:内存申请失败! 禁用IRQ\n");
}

void vApplicationIdleHook(){
    
}   

void vApplicationTickHook(){
}


void vApplicationStackOverflowHook(TaskHandle_t xTask,char* pcTaskName){
    sHMI_Debug_Printf("WARNING警告:触发栈金丝雀机制! 任务句柄:0x%X,任务名:%s\n",xTask,pcTaskName);
}

