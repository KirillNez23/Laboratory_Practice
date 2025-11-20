#include "../Inc/init.h"


void GPIO_Init (void)
{

    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN);
    (void)RCC->AHB1ENR;
    //PB0 - out
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE0, (0x1u << GPIO_MODER_MODE0_Pos));
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_0);
    MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED0, (0x1u << GPIO_OSPEEDR_OSPEED0_Pos));
    CLEAR_BIT(GPIOB->BSRR, GPIO_BSRR_BR0);
    MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD0, (0x0u << GPIO_PUPDR_PUPD0_Pos));

    //PB7 - out
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE7, (0x1u << GPIO_MODER_MODE7_Pos));
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_7);
    MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED7, (0x1u << GPIO_OSPEEDR_OSPEED7_Pos));
    CLEAR_BIT(GPIOB->BSRR, GPIO_BSRR_BR7);
    MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD7, (0x0u << GPIO_PUPDR_PUPD7_Pos));
    //PB14 - out
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE14, (0x1u << GPIO_MODER_MODE14_Pos));
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_14);
    MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED14, (0x1u << GPIO_OSPEEDR_OSPEED14_Pos));
    CLEAR_BIT(GPIOB->BSRR, GPIO_BSRR_BR14);
    MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD14, (0x0u << GPIO_PUPDR_PUPD14_Pos));

    //PB8 - out
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE8, (0x1u << GPIO_MODER_MODE8_Pos));
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_8);
    MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED8, (0x1u << GPIO_OSPEEDR_OSPEED8_Pos));
    CLEAR_BIT(GPIOB->BSRR, GPIO_BSRR_BR8);
    MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD8, (0x0u << GPIO_PUPDR_PUPD8_Pos));

    //PB11 - out
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE11, (0x1u << GPIO_MODER_MODE11_Pos));
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_11);
    MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED11, (0x1u << GPIO_OSPEEDR_OSPEED11_Pos));
    CLEAR_BIT(GPIOB->BSRR, GPIO_BSRR_BR11);
    MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD11, (0x0u << GPIO_PUPDR_PUPD11_Pos));

    //PB10 - out
    MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE10, (0x1u << GPIO_MODER_MODE10_Pos));
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_10);
    MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED10, (0x1u << GPIO_OSPEEDR_OSPEED10_Pos));
    CLEAR_BIT(GPIOB->BSRR, GPIO_BSRR_BR10);
    MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD10, (0x0u << GPIO_PUPDR_PUPD10_Pos));
}

   

void CLK_CLEAR(void)
{
 MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U); 
 CLEAR_REG(RCC->CFGR); 
 while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET); 
 CLEAR_BIT(RCC->CR, RCC_CR_PLLON); 
 while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET); 
 CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON); 
 while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET); 
 CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);   
}

// void RCC_Init(void)
// {
//     //RCC_CR_init
//     SET_BIT(RCC->CR, RCC_CR_HSEON); //Запускаем внешний кварцевый резонатор 
//     while(READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET); //Ждём пока он запустится 
//     CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP); //Сбросим бит байпаса в 0, если вдруг там что-то лежит 
//     SET_BIT(RCC->CR, RCC_CR_CSSON); //Запустим Clock detector

//     //RCC_PLLCFGR
//     CLEAR_REG(RCC->PLLCFGR); 
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE); 
//     MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, RCC_PLLCFGR_PLLM_2); //Выставляем предделитель входной частоты PLL на 4 
//     MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, RCC_PLLCFGR_PLLN_2 | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_7); //Настраиваем умножение частоты, полученной после деления (частоты VCO) на х180 
//     CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk); //Настраиваем предделитель получившейся частоты после умножения. Иными словами, получаем итоговую частоту PLL 
//     SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL 
//     while(READ_BIT(RCC->CR, RCC_CR_PLLRDY)); //Ждём запуска PLL

//     //RCC_CFGR
//     MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); //Выбираем PLL в качестве System Clock

//     MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1); //Предделитель AHB, без делителя 
//     MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV4); //Предделитель APВ1, делим на 4 
//     MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV2); //Предделитель APВ2, делим на 2 
//     MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO2PRE, RCC_CFGR_MCO2PRE_Msk); //Предделитель на выходе MCO2 (PC9) = 5 
//     CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCO2); //Настраиваем на выход MCO2 - System clock
//     MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);

// }

void ITR_init(void)
{ 
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN); //Включение тактирования периферии SYSCFG 
    MODIFY_REG(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI13_Msk, SYSCFG_EXTICR4_EXTI13_PC); //Настройка мультиплексора на вывод линии прерывания EXTI13 на PC13 
    SET_BIT(EXTI->IMR, EXTI_IMR_MR13); //Настройка маскирования 13 линии 
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR13); //Настройка детектирования нарастающего фронта 13 линии 
    SET_BIT(EXTI->FTSR, EXTI_FTSR_TR13); //Настройка детектирования спадающего фронта 13 линии 
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); //Установка 0 приоритета прерывания для вектора EXTI15_10
    NVIC_EnableIRQ(EXTI15_10_IRQn); //Включение прерывания по вектору EXTI15_10 
}

void SysTick_Init(void)
{
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //На всякий случай, предварительно, выключим счётчик 
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk); //Разрешаем прерывание по системному таймеру 
    SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk); //Источник тактирования будет идти из AHB без деления 
    MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, (180000000 / 1000) - 1 << SysTick_LOAD_RELOAD_Pos); //Значение с которого начинается счёт, эквивалентное 1 кГц 
    MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, 0 << SysTick_VAL_CURRENT_Pos); //Очистка поля 
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Включим счётчик
}

void RCC_Init(void){
    MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
    CLEAR_REG(RCC->CFGR);
    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET);
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET);
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET);
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP); //Сбросим бит байпаса в 0, если вдруг там что-то лежит

    SET_BIT(RCC->CR, RCC_CR_HSEON); //Запускаем внешний кварцевый резонатор
    while(READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET); //Ждём пока он запустится
    SET_BIT(RCC->CR, RCC_CR_CSSON); //Запустим Clock detector
    

    CLEAR_REG(RCC->PLLCFGR);
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE);
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_2);
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_6);
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_0);
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_3);

    SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);
    SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV4);
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV2);
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO1);
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1);
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCO2);
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO2PRE_2 | RCC_CFGR_MCO2PRE_1);

    SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_5WS);
    SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
    while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET); //Ждём запуска PLL
}

// void ITR_Init(void){
//     SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

//     SET_BIT(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI13_PC);
//     SET_BIT(EXTI->IMR, EXTI_IMR_IM13);
//     SET_BIT(EXTI->RTSR, EXTI_RTSR_TR13);
//     CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR13);

//     NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
//     NVIC_EnableIRQ(EXTI15_10_IRQn);
// }






void Led_init(void)
{
    RCC_AHB1ENR |= RCC_GPIOD_EN;
    RCC_AHB1ENR |= RCC_GPIOB_EN;
    //PB0 - yelow
    BIT_SET(GPIOB_MODER, GPIO_PIN_OUT_0);
    BIT_SET(GPIOB_OTYPER, GPIO_OFF);
    BIT_SET(GPIOB_OSPEEDR, GPIO_PIN_MED_0);
    BIT_SET(GPIOB_BSRR, GPIO_PIN_RESET_0);
    //PB7 - blue
    BIT_SET(GPIOB_MODER, GPIO_PIN_OUT_7);
    BIT_SET(GPIOB_OTYPER, GPIO_OFF);
    BIT_SET(GPIOB_OSPEEDR, GPIO_PIN_MED_7);
    BIT_SET(GPIOB_BSRR, GPIO_PIN_RESET_7);
    //PB14 - red
    BIT_SET(GPIOB_MODER, GPIO_PIN_OUT_14);
    BIT_SET(GPIOB_OTYPER, GPIO_OFF);
    BIT_SET(GPIOB_OSPEEDR, GPIO_PIN_MED_14);
    BIT_SET(GPIOB_BSRR, GPIO_PIN_RESET_14);
    //PD1 with my macros
    BIT_SET(GPIOD_MODER, GPIO_PIN_OUT_1);
    BIT_SET(GPIOD_OTYPER, GPIO_OFF);
    BIT_SET(GPIOD_OSPEEDR, GPIO_PIN_MED_1);
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_1);
    //PD3 with my macros
    BIT_SET(GPIOD_MODER, GPIO_PIN_OUT_3);
    BIT_SET(GPIOD_OTYPER, GPIO_OFF);
    BIT_SET(GPIOD_OSPEEDR, GPIO_PIN_MED_3);
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_3);
    //PD2 with CMSIS
    MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODE2, (0x1u << GPIO_MODER_MODE2_Pos));
    CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT_2);
    MODIFY_REG(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED2, (0x1u << GPIO_OSPEEDR_OSPEED2_Pos));
    CLEAR_BIT(GPIOD->BSRR, GPIO_BSRR_BR2);
    MODIFY_REG(GPIOD->PUPDR, GPIO_PUPDR_PUPD2, (0x0u << GPIO_PUPDR_PUPD2_Pos));
    //PD0 bytes operations
    *(uint32_t *)(0x40020C00UL + 0x00UL) |= 0x40000000UL; //MODER
    *(uint32_t *)(0x40020C00UL + 0x04UL) |= 0x00; //OTYPER
    *(uint32_t *)(0x40020C00UL + 0x08UL) |= 0x40000000UL; //OSPEEDR
    *(uint32_t *)(0x40020C00UL + 0x0CUL) |= 0x80000000UL; //BSRR

}

void Buttons_Init(void)
{

    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
    (void)RCC->AHB1ENR;

    MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODE0_Msk, (0x0u << GPIO_MODER_MODE0_Pos)); // 00 = Input
    MODIFY_REG(GPIOD->PUPDR, GPIO_PUPDR_PUPD0_Msk, (0x2u << GPIO_PUPDR_PUPD0_Pos)); // 10 = Pull-Down


    MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODE3_Msk, (0x0u << GPIO_MODER_MODE3_Pos)); // 00 = Input
    MODIFY_REG(GPIOD->PUPDR, GPIO_PUPDR_PUPD3_Msk, (0x2u << GPIO_PUPDR_PUPD3_Pos)); // 10 = Pull-Down
}

void PD0_out(void)
{
    MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODE0_Msk, (0x1u << GPIO_MODER_MODE0_Pos)); // Output
    CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT_0); // Push-pull
    MODIFY_REG(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED0_Msk, (0x1u << GPIO_OSPEEDR_OSPEED0_Pos)); // Medium
    MODIFY_REG(GPIOD->PUPDR, GPIO_PUPDR_PUPD0_Msk, (0x00u << GPIO_PUPDR_PUPD0_Pos)); // Off
}
void PD0_inp(void)
{
    MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODE0_Msk, (0x0u << GPIO_MODER_MODE0_Pos)); // 00 = Input
    MODIFY_REG(GPIOD->PUPDR, GPIO_PUPDR_PUPD0_Msk, (0x2u << GPIO_PUPDR_PUPD0_Pos)); // 10 = Pull-Down
}


void GPIO_Init_With_Myself_Macros(void){    
    RCC_AHB1ENR |= RCC_GPIOD_EN; // Включение тактирования на периферии GPIOD, регистр AHB
    
        // Configure PD4 as input with pull-up
    BIT_CLEAR(GPIOD_MODER, 0x03U << 8); // Clear bits 9:8
    BIT_SET(GPIOD_PUPDR, GPIO_PUPDR_PU_4);
    
    // Set output type to push-pull (already 0 by default)
    BIT_CLEAR(GPIOD_OTYPER, 0x0FU); // Clear bits 0-3
    
    // Set medium speed
    BIT_SET(GPIOD_OSPEEDR, GPIO_OSPEEDR_MED_0);
    BIT_SET(GPIOD_OSPEEDR, GPIO_OSPEEDR_MED_1);
    BIT_SET(GPIOD_OSPEEDR, GPIO_OSPEEDR_MED_2);
    BIT_SET(GPIOD_OSPEEDR, GPIO_OSPEEDR_MED_3);
    
    // Configure PD0 as input with pull-up (Button 2)
    BIT_CLEAR(GPIOD_MODER, GPIO_PIN_OUT_0);
    BIT_SET(GPIOD_PUPDR, GPIO_PIN_PU_0);
    
    // Configure PD4 as input with pull-up (Button 1)
    BIT_CLEAR(GPIOD_MODER, GPIO_PIN_OUT_4);
    BIT_SET(GPIOD_PUPDR, GPIO_PIN_PU_4);

    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_0); // Предварительное переключение вход, регистр BSRR, бит BR0
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_1); // Предварительное переключение вход, регистр BSRR, бит BR1
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_2); // Предварительное переключение вход, регистр BSRR, бит BR2
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_3); // Предварительное переключение вход, регистр BSRR, бит BR3
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_4); // Предварительное переключение вход, регистр BSRR, бит BR4

    BIT_SET(GPIOD_MODER, GPIO_PIN_OUT_0); //Настройка пина PD0 на вывод, регистр MODER
    BIT_SET(GPIOD_MODER, GPIO_PIN_OUT_1); //Настройка пина PD1 на вывод, регистр MODER
    BIT_SET(GPIOD_MODER, GPIO_PIN_OUT_2); //Настройка пина PD2 на вывод, регистр MODER
    BIT_SET(GPIOD_MODER, GPIO_PIN_OUT_3); //Настройка пина PD3 на вывод, регистр MODER

    BIT_SET(GPIOD_OTYPER, GPIO_OFF); //Настройка режима работы вывода на push-pull, регистр OTYPER

    BIT_SET(GPIOD_OSPEEDR, GPIO_PIN_MED_1); // Настройка скороcти работы вывода PD0, регистр OSPEEDR
    BIT_SET(GPIOD_OSPEEDR, GPIO_PIN_MED_2); // Настройка скороcти работы вывода PD0, регистр OSPEEDR
    BIT_SET(GPIOD_OSPEEDR, GPIO_PIN_MED_3); // Настройка скороcти работы вывода PD0, регистр OSPEEDR
}

void GPIO_Init_CMSIS(void){
    SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_0); // Настройка скороcти работы вывода PB14, регистр MODER
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT14);
    SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR14_0);
    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR14);
}

void GPIO_Reset(void){
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_0); // Предварительное переключение вход, регистр BSRR, бит BR0
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_1); // Предварительное переключение вход, регистр BSRR, бит BR1
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_2); // Предварительное переключение вход, регистр BSRR, бит BR2
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_3); // Предварительное переключение вход, регистр BSRR, бит BR3
    BIT_SET(GPIOD_BSRR, GPIO_PIN_RESET_4); // Предварительное переключение вход, регистр BSRR, бит BR4
}

void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms * 1000; i++);
}