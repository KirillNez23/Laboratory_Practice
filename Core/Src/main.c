// #include "../Inc/init.h"
// #include "main.h"

// volatile uint8_t btncnt = 0, flag = 0;

// int main(void)
// {
//     RCC_Init();
//     ITR_init();

//     SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN);

//     SET_BIT(GPIOB->MODER, GPIO_MODER_MODER7_0);
//     SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED7_0);
//     SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR7);

//     SET_BIT(GPIOC->MODER, GPIO_MODER_MODER9_1);
//     SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED9);
//     MODIFY_REG(GPIOC->AFR[1], GPIO_AFRH_AFSEL9_Msk, 0x00UL);

//     SET_BIT(GPIOA->MODER, GPIO_MODER_MODE8_1);
//     SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8);
//     CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL8);
// }



//1 кнопка 3 частоты
//2 кнопка включает последовательно 4 светодиода(циклично, короткое нажатие 1 12 123 1234 1 )
//выбор светодиода, для которого выбирается частота(длинное нажатие имзенеие порядкового номера, если не настраивать, то базовя, можно настроить свтодтод. даже если он сейчас не горит) без таймеров и прерываний


/*
volatile uint32_t GlobalTickCount;
uint8_t mode = 1;
uint8_t freq_mode;
uint8_t LedState = 0;
volatile uint32_t BtnStartTick;
uint8_t i;
void blink(uint32_t bitmask, uint8_t freq) //Freq in ms
{
    SET_BIT(GPIOB->BSRR, bitmask);
    milis(freq/2);
    SET_BIT(GPIOB->BSRR, bitmask<<16U);
    milis(freq/2);
}

void blink2(uint32_t bitmask1, uint32_t bitmask2, uint8_t freq) //Freq in ms
{
    SET_BIT(GPIOB->BSRR, bitmask1);
    SET_BIT(GPIOB->BSRR, bitmask2);
    milis(freq/2);
    SET_BIT(GPIOB->BSRR, bitmask1<<16U);
    SET_BIT(GPIOB->BSRR, bitmask2<<16U);
    milis(freq/2);
}

int main(void)
{
    uint32_t led_bitmasks[6] = {GPIO_BSRR_BS0, GPIO_BSRR_BS7, GPIO_BSRR_BS14, GPIO_BSRR_BS8, GPIO_BSRR_BS10, GPIO_BSRR_BS11};
    uint16_t freq1[3] = {2500, 625, 385};
    uint16_t freq2[3] = {3300, 625, 435};
    GPIO_Init();
    CLK_CLEAR();
    RCC_Init();
    ITR_init();
    SysTick_Init();

    while(1)
    {
        switch( mode)
        {
            case 1:
                for( i = 0; i<6; i++)
                {
                    blink(led_bitmasks[i],freq1[freq_mode]);
                }
            break;

            case 2:
                
            for(i = 0; i<3; i++)
            {
                blink2(led_bitmasks[i], led_bitmasks[i+3],freq2[freq_mode]);
            }
            break;
        }
    }


}*/

#include "../Inc/init.h"

static void delay_cycles(volatile uint32_t t) //time in ms
{
    while (t--) __NOP();
}

static uint8_t debounce(uint32_t bitmask)
{
    for (int i = 0; i < 5; i++)
    {
        if (BIT_READ(GPIOD->IDR, bitmask))
            return 0;
        delay_cycles(50000);
    }
    return 1;
}


uint8_t btn1 = 0;
uint8_t btn2 = 0;
uint8_t btn2_state = 0;
uint8_t led_flag = 0;
int main(void)
{
    Led_init();
    Buttons_Init();
    SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR0);    

    while (1)
    {
        
        if (debounce(GPIO_IDR_ID3))
        {
            btn2 = 1;
                
                btn2_state = 1;
                delay_cycles(50000);
        }
        btn2 = 0;

        if (btn2_state == 0)
        {
 
            PD0_inp();

            if (debounce(GPIO_IDR_ID0))
            {
                btn1++;
                    switch(btn1)
                    {
                    case 1:
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_1);
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_RESET_2);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_0);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_7);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_14);
                        break;
                    case 2:
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_1);
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_2);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_0);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_7);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_14);
                        break;
                    case 3:
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_1);
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_2);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_SET_0);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_7);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_14);
                        break;
                    case 4:
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_1);
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_2);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_SET_0);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_SET_7);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_14);
                        break;
                    case 5:
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_1);
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_2);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_SET_0);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_SET_7);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_SET_14);
                        break;
                    case 6:
                        btn1 = 0;
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_RESET_1);
                        SET_BIT(GPIOD->BSRR, GPIO_PIN_RESET_2);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_0);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_7);
                        SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_14);
                        break;
                    }
                    delay_cycles(50000);
                
            }
            
        }
        else
        {
            PD0_out();
  
            while (1)
            {
                SET_BIT(GPIOD->BSRR, GPIO_PIN_RESET_1);
                SET_BIT(GPIOD->BSRR, GPIO_PIN_RESET_2);
                SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_0);
                SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_7);
                SET_BIT(GPIOB->BSRR, GPIO_PIN_RESET_14);
                SET_BIT(GPIOD->BSRR, GPIO_PIN_SET_0);

                if (debounce(GPIO_IDR_ID3))
                {
                        BIT_SET(GPIOD_BSRR, GPIO_BSRR_BS0);
                        btn2 = 1;
                        BIT_SET(GPIOD_BSRR, GPIO_BSRR_BR0);
                        PD0_inp();
                        btn2_state = 0;
                        delay_cycles(500000);
                }
                break;
            }
        }
    }
}