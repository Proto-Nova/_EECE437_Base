/******************************************************************************
 * @course      EECE 437 Spring 2014
 * @proj        Lab #1 Template File
 * @file        EECE 437 S14 Lab #1 Template.c
 * 
 * @authors     Jon Hourany, James Anderson
 * @date        2/13/2014
 * @breif       Graphically draw gyroscope readings
 ******************************************************************************/
/*******************************************************************************
 * Includes                                                                    
 ******************************************************************************/
#include "main.h"
#include "stm32f429i_discovery_l3gd20.h"

/*******************************************************************************
 * Private define                                                              
 ******************************************************************************/
/* Button 1 (Usually Red) pixel boundries       */
#define         B1_XMIN         2
#define         B1_XMAX         64
#define         B1_YMIN         228
#define         B1_YMAX         290

/* Button 2 (Usually Blue) pixel boundries      */
#define         B2_XMIN         89
#define         B2_XMAX         153
#define         B2_YMIN         228
#define         B2_YMAX         290

/* Button 3 (Usually Green) pixel boundries     */
#define         B3_XMIN         176
#define         B3_XMAX         238
#define         B3_YMIN         228
#define         B3_YMAX         290

/* Accelerometor Draw Boundries. A1 = Xval, A2 = Yval */
#define         A1_YMIN         54
#define         A1_YMAX         130
#define         A1_ZONE         A1_YMIN-1, A1_YMAX+2

#define         A2_YMIN         140
#define         A2_YMAX         210
#define         A2_ZONE         A2_YMIN-1, A2_YMAX+2

extern uint32_t CurrentFrameBuffer;

/* Private macro -------------------------------------------------------------*/

#define ABS(x)                     (x < 0) ? (-x) : x
#define L3G_Sensitivity_250dps     (float)114.285f        /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps]  */
#define L3G_Sensitivity_500dps     (float)57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps]  */
#define L3G_Sensitivity_2000dps    (float)14.285f         /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
  
/* Private variables ---------------------------------------------------------*/
float Buffer[6];
float Gyro[3];
float X_BiasError, Y_BiasError, Z_BiasError = 0.0;
uint8_t Xval, Yval = 0x00;
static __IO uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private variables ---------------------------------------------------------*/
int a1_ground      = 0;         /*!< Zero point for X read out               >*/
int a1_y_signal    = 0;         /*!< The numerica value captured from gyro   >*/
int a1_y_position  = 0;         /*!< Draw position for X read out            >*/

int a2_ground      = 0;         /*!< Zero point for Y read out               >*/
int a2_y_signal    = 0;         /*!< The numerica value captured from gyro   >*/
int a2_y_position  = 0;         /*!< Draw position for Y read out            >*/

int amplification  = 1;         /*!< Aplification on to signal (sensitivity) >*/
int sample_rate    = 1;         /*!< Sample Rate effects delay in loop       >*/
int time           = 0;         /*!< Moves draw point per systick            >*/
char gyro_val_buffer[15];       /*!< For i to a conversion of X/YVals        >*/

/* Private function prototypes -----------------------------------------------*/
static void TP_Config(void);

void LCD_ClearSection(uint16_t, uint16_t);

static void Demo_GyroConfig(void);
static void Demo_GyroReadAngRate (float* pfData);
static void Gyro_SimpleCalibration(float* GyroData);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  static TP_STATE* TP_State; 
    
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32f429_439xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f4xx.c file
  */      

    /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  
  /* LCD initialization */
  LCD_Init();
  
  /* LCD Layer initialization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
  
   /* Gyroscope configuration */
  Demo_GyroConfig();

  /* Gyroscope calibration */
  Gyro_SimpleCalibration(Gyro); 
  
  /* Touch Panel configuration */
  TP_Config();
  
  a1_ground = A1_YMIN + ((A1_YMAX - A1_YMIN)/2);
  a2_ground = A2_YMIN + ((A2_YMAX - A2_YMIN)/2);
// Your code changes go here...........................................
  while (1)
  {
      TP_State = IOE_TP_GetState();
      
      if((TP_State->TouchDetected) && ((TP_State->Y < B1_YMAX) && (TP_State->Y >= B1_YMIN))) {
        /* Red Button - Forces screen clear and resents draw pos */  
        if((TP_State->X >= B1_XMIN) && (TP_State->X < B1_XMAX)) {   
                 LCD_SetFont(&Font16x24);
                 LCD_SetTextColor(LCD_COLOR_RED); 
                 time = 2;          
                 LCD_ClearSection(A1_ZONE);
                 LCD_ClearSection(A2_ZONE);

          /* Blue Button - Changes sensitivity by doubling values in signal */       
          } else if ((TP_State->TouchDetected) && (TP_State->X >= B2_XMIN) && (TP_State->X < B2_XMAX)) {            
                 LCD_SetFont(&Font16x24);
                 LCD_SetTextColor(LCD_COLOR_BLUE); 
                 amplification = (amplification == 1 ? 2 : 1);  /*!< Toggles Amplification >*/

          /* Green Button - Changes sampling rate (delay at end of loop) */       
          } else if ((TP_State->TouchDetected) && (TP_State->X >= B3_XMIN) && (TP_State->X < B3_XMAX)) {
                 LCD_SetFont(&Font16x24);
                 LCD_SetTextColor(LCD_COLOR_GREEN); 
                 sample_rate = (sample_rate == 1 ? .5 : 1);     /*< Toggles Sample rate >*/
          }
      }

      time++;                               /*!< Move draw point right one pixel >*/
      if (time >= 238) {                    /*!< LCD is 238 pixels across.       >*/
          time = 2;                         /*!< Reset draw point to pixel 2     >*/
          LCD_ClearSection(A1_ZONE);        /*!< Clear both sections             >*/
          LCD_ClearSection(A2_ZONE);
      }

      /* Read Gyro Angular data */
      Demo_GyroReadAngRate(Buffer);

      Buffer[0] = (int8_t)Buffer[0] - (int8_t)Gyro[0];
      Buffer[1] = (int8_t)Buffer[1] - (int8_t)Gyro[1];
      
      /* X value position */
      /* @float(120)    Emperical testing showed that highest avg val produced from wrist is 120 */
      /* We determined that there were 37 pixels of draw space above and below the ground level  */
      /* so the signal should be a percentage of that                                            */
      a1_y_signal   = -37*(Buffer[0]/(float)(120));             /*!< NOTE: float MUST be used in math operation */ 
      a1_y_position = amplification*a1_y_signal + a1_ground;    /*   here or result of multiplication will      */
                                                                /*   almost always either be 1 or 0             *>/
      /* Y value position */
      a2_y_signal   = -37*(Buffer[1]/(float)(120));
      a2_y_position = amplification*a2_y_signal + a2_ground;    /*!< If needed, amplify sig before drawing *>/
      
      if (a1_y_position < A1_YMIN) { a1_y_position = A1_YMIN; } /*!< Celing value for X Pos >*/
      if (a1_y_position > A1_YMAX) { a1_y_position = A1_YMAX; } /*!< Floor value for X Pos  >*/
      if (a2_y_position < A2_YMIN) { a2_y_position = A2_YMIN; } /*!< Celing value for y Pos >*/
      if (a2_y_position > A2_YMAX) { a2_y_position = A2_YMAX; } /*!< Floor value for Y Pos  >*/
      LCD_SetTextColor(LCD_COLOR_BLACK);
      LCD_DrawFullCircle(time, a1_y_position, 1);
      LCD_DrawFullCircle(time, a2_y_position, 1);
      
      Delay(sample_rate);
    }
}

// Supplied Functions - Do not change ......

void LCD_ClearSection(uint16_t YStart, uint16_t YEnd)
{
    int x = 0;
    
    LCD_SetTextColor(LCD_COLOR_WHITE);
    for (x=0; x<=240; x++) 
      LCD_DrawLine(x, YStart, (YEnd - YStart), LCD_DIR_VERTICAL);
    
}

/**
* @brief  Configure the IO Expander and the Touch Panel.
* @param  None
* @retval None
*/
static void TP_Config(void)
{
  /* Clear the LCD */ 
  LCD_Clear(LCD_COLOR_WHITE);
  
  /* Configure the IO Expander */
  if (IOE_Config() == IOE_OK)
  { 
      LCD_SetFont(&Font16x24);
      LCD_DisplayStringLine(LINE(1), (uint8_t*)" EECE 437 S14 ");
      LCD_DrawLine(0, 15, 240, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(0, 50, 240, LCD_DIR_HORIZONTAL);
   
      
      LCD_DrawLine(0, 133, 240, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(0, 217, 240, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(0, 300, 240, LCD_DIR_HORIZONTAL);
      
      LCD_SetTextColor(LCD_COLOR_BLACK); 
      LCD_DrawFullRect(2, 228, 62, 62);
      LCD_SetTextColor(LCD_COLOR_RED); 
      LCD_DrawFullRect(6, 232, 54, 54);    
    
      LCD_SetTextColor(LCD_COLOR_BLACK); 
      LCD_DrawFullRect(89, 228, 62, 62);
      LCD_SetTextColor(LCD_COLOR_BLUE); 
      LCD_DrawFullRect(93, 232, 54, 54);
      
      LCD_SetTextColor(LCD_COLOR_BLACK); 
      LCD_DrawFullRect(176, 228, 62, 62);
      LCD_SetTextColor(LCD_COLOR_GREEN); 
      LCD_DrawFullRect(180, 232, 54, 54);    
 
   
    //LCD_SetFont(&Font16x24);
    //LCD_DisplayChar(LCD_LINE_12, 195, 0x43);
    //LCD_DrawLine(0, 248, 240, LCD_DIR_HORIZONTAL);
    
  }  
  else
  {
    LCD_Clear(LCD_COLOR_RED);
    LCD_SetTextColor(LCD_COLOR_BLACK); 
    LCD_DisplayStringLine(LCD_LINE_6,(uint8_t*)"   IOE NOT OK      ");
    LCD_DisplayStringLine(LCD_LINE_7,(uint8_t*)"Reset the board   ");
    LCD_DisplayStringLine(LCD_LINE_8,(uint8_t*)"and try again     ");
  }
}



/**
* @brief  Configure the Mems to gyroscope application.
* @param  None
* @retval None
*/
static void Demo_GyroConfig(void)
{
  L3GD20_InitTypeDef L3GD20_InitStructure;
  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; 
  L3GD20_Init(&L3GD20_InitStructure);
  
  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
  
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
* @brief  Calculate the angular Data rate Gyroscope.
* @param  pfData : Data out pointer
* @retval None
*/
static void Demo_GyroReadAngRate (float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;
  
  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;
    
  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;
    
  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
  pfData[i]=(float)RawData[i]/sensitivity;
  }
}

/**
* @brief  Calculate offset of the angular Data rate Gyroscope.
* @param  GyroData : Data out pointer
* @retval None
*/
static void Gyro_SimpleCalibration(float* GyroData)
{
  uint32_t BiasErrorSplNbr = 500;
  int i = 0;
  
  for (i = 0; i < BiasErrorSplNbr; i++)
  {
    Demo_GyroReadAngRate(GyroData);
    X_BiasError += GyroData[0];
    Y_BiasError += GyroData[1];
    Z_BiasError += GyroData[2];
  }
  /* Set bias errors */
  X_BiasError /= BiasErrorSplNbr;
  Y_BiasError /= BiasErrorSplNbr;
  Z_BiasError /= BiasErrorSplNbr;
  
  /* Get offset value on X, Y and Z */
  GyroData[0] = X_BiasError;
  GyroData[1] = Y_BiasError;
  GyroData[2] = Z_BiasError;
}


/**
* @brief  Basic management of the timeout situation.
* @param  None.
* @retval None.
*/
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
