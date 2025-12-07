#include "main.h"
#include "stdint.h"
//1 кнопка 3 частоты
//2 кнопка включает последовательно 4 светодиода(циклично, короткое нажатие 1 12 123 1234 1 )
//выбор светодиода, для которого выбирается частота(длинное нажатие имзенеие порядкового номера, если не настраивать, то базовя, можно настроить свтодтод. даже если он сейчас не горит) без таймеров и прерываний


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


}