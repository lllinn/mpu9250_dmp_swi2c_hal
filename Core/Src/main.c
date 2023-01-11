/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "delay.h"
#include "myiic.h"
#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{
    while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET){}; 
    USART1->DR=c;  
} 

//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0X01~0X1C
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+4]=0;	//У��������
	send_buf[0]=0XAA;	//֡ͷ
	send_buf[1]=0XAA;	//֡ͷ
	send_buf[2]=fun;	//������
	send_buf[3]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//��������
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//����У���	
	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}
//���ͼ��ٶȴ���������+����������(������֡)
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ 
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[18]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	tbuf[12]=0;//��Ϊ����MPL��,�޷�ֱ�Ӷ�ȡ����������,��������ֱ�����ε�.��0���.
	tbuf[13]=0;
	tbuf[14]=0;
	tbuf[15]=0;
	tbuf[16]=0;
	tbuf[17]=0;
	usart1_niming_report(0X02,tbuf,18);//������֡,0X02
}	
//ͨ������1�ϱ���������̬���ݸ�����(״̬֡)
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
//prs:��ѹ�Ƹ߶�,��λ:cm
//fly_mode:����ģʽ
//armed:����״̬
void usart1_report_imu(short roll,short pitch,short yaw,int prs, u8 fly_mode, u8 armed)
{
	u8 tbuf[12];   	
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF;
	tbuf[6]=(prs>>24)&0XFF;
	tbuf[7]=(prs>>16)&0XFF;
	tbuf[8]=(prs>>8)&0XFF;
	tbuf[9]=prs&0XFF;
	tbuf[10]=fly_mode;
	tbuf[11]=armed;
	usart1_niming_report(0X01,tbuf,12);//״̬֡,0X01
}  

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	delay_init(96);  // ��ʼ��ʱ��
	IIC_Init();
	u8 device_id;
	//(MPU9250_ADDR, MPU_DEVICE_ID_REG)
	u8 res;
	//printf("��Ƭ����������\r\n");
	res = MPU_Read_Len(MPU9250_ADDR, MPU_DEVICE_ID_REG, 1, &device_id);
	//printf("the read is %d\r\n", res);
	if(res) {
		//printf("err\r\n");
		while(1);
	} else {
		//printf("OK\r\n");
	}
	//printf("the id is %#x\r\n", device_id);
	if(device_id == MPU6500_ID) {
		//printf("�豸id�����ȷ\r\n");
	} else {
		//printf("��鿴��ַ�Ƿ�������ȷ���߽����Ƿ���ȷ\r\n");
		while(1);
	}
	u8 err = mpu_dmp_init();
	if(err) {
		//printf("the err is %d\r\n", err);
		res = MPU_Read_Len(MPU9250_ADDR, MPU_DEVICE_ID_REG, 1, &device_id);
		//printf("the read is %d\r\n", res);
		//printf("the id is %#x\r\n", device_id);
		while(1);
	} else {
		//printf("mpu9250 init ok\r\n");
	}
	   float pitch,roll,yaw; 	        //ŷ����
    short aacx,aacy,aacz;	        //���ٶȴ�����ԭʼ����
    short gyrox,gyroy,gyroz;        //������ԭʼ����
    short temp;                     //�¶�
		u8 report = 1;
	  while(1)
    {
        err = mpu_mpl_get_data(&pitch,&roll,&yaw);
        if(err == 0)
        {
            temp = MPU_Get_Temperature();	                                     //�õ��¶�ֵ��������100����
            MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	                         //�õ����ٶȴ���������
            MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	                         //�õ�����������
            //mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);               //���ͼ��ٶ�+������ԭʼ����
//            printf("ok\r\n");
//            printf("roll = %f   pitch = %f   yaw = %f (T = %d)\r\n",roll,pitch,yaw,temp);
  			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���ͼ��ٶ�+������ԭʼ����
			if(report)usart1_report_imu((int)(roll*100),(int)(pitch*100),(int)(yaw*100),0,0,0);

        }
    }

	
  /* USER CODE END 2 */

  /* Infinite loop */
	
	 

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		printf("hello world\r\n");
		//HAL_Delay(1000);
		delay_ms(1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
