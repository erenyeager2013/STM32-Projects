//********************************************************************
//*                    EEE2046F C template                           *
//*==================================================================*
//* WRITTEN BY: Jesse Arendse   	                 		         *
//* DATE CREATED: 07/04/2023                                         *
//* MODIFIED: 21/11/2025                                             *
//*==================================================================*
//* PROGRAMMED IN: Visual Studio Code                                *
//* TARGET:        STM32F0                                           *
//*==================================================================*
//* DESCRIPTION: Timer                                               *
//*                                                                  *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

#define sw1 ((GPIOA -> IDR & GPIO_IDR_1) == 0) // Switch 1 connected to PA1
#define sw2 ((GPIOA -> IDR & GPIO_IDR_2) == 0) // Switch 2 connected to PA2
#define sw3 ((GPIOA -> IDR & GPIO_IDR_3) == 0) // Switch 3 connected to PA3
#define sw0 ((GPIOA -> IDR & GPIO_IDR_0) == 0) // Switch 0 connected to PA0

//====================================================================
// GLOBAL VARIABLES
//====================================================================

typedef struct {
    uint32_t pattern;
    const char *message;
} DisplayState;

DisplayState states[] = {
    {GPIO_ODR_0, "State 0"},
    {GPIO_ODR_1, "State 1"},
    {GPIO_ODR_2, "State 2"},
    {GPIO_ODR_3, "State 3"},
    {GPIO_ODR_4, "State 4"},
    {GPIO_ODR_5, "State 5"},
    {GPIO_ODR_6, "State 6"},
    {GPIO_ODR_7, "State 7"}
};

/* Mask for the 8 LED bits on PB0..PB7 (adjust if pins differ) */
const uint32_t LED_MASK = (GPIO_ODR_0 | GPIO_ODR_1 | GPIO_ODR_2 | GPIO_ODR_3 |
                           GPIO_ODR_4 | GPIO_ODR_5 | GPIO_ODR_6 | GPIO_ODR_7);


uint8_t current_state = 0;
uint8_t prev_state = 0;
volatile uint8_t current_state1 = 0;
volatile uint8_t timer_triggered = 0; // Renamed from uflag for clarity
volatile uint8_t timer2_triggered = 0; // New flag for Timer 2


//====================================================================
// FUNCTION DECLARATIONS
//====================================================================

void init_GPIO();
void TIM14_init();
void TIM2_init();

//====================================================================
// MAIN FUNCTION
//====================================================================


int main ()
{
    init_GPIO();
    init_LCD();
    
    // Initial Display
    lcd_command(CLEAR);
    lcd_putstring("System Ready");
    delay(100000);

    TIM2_init();
   
    lcd_command(CLEAR);
    lcd_putstring("TIMER DEMO");
    delay(100000);

    TIM14_init(); // Initialize timer but don't start it immediately if you want SW0 to do it
    TIM14->CR1 &= ~TIM_CR1_CEN; // Ensure stopped at start

    uint8_t last_state = 0xFF; // Track change to prevent flickering

    while (1){
        
        // CHECK 1: Timer Event (Automatic State Change)
        if (timer_triggered == 1){
            current_state = (current_state + 1) % 8;
            timer_triggered = 0; // Reset flag
        }

        // CHECK 2: Button Inputs
        if (sw0) {
            TIM14->CR1 |= TIM_CR1_CEN; // Start Timer
            lcd_command(CLEAR);
            lcd_putstring("Timer Started");
            delay(200000); // Debounce/Wait to read
            last_state = 0xFF; // Force display update
        }
        
        if (sw1) {
            TIM14->CR1 &= ~TIM_CR1_CEN; // Stop Timer
            lcd_command(CLEAR);
            lcd_putstring("Timer Stopped");
            delay(200000);
            last_state = 0xFF;
        }

        if (sw2) {
            current_state = 0; // Reset
            lcd_command(CLEAR);
            lcd_putstring("Resetting...");
            delay(200000);
            last_state = 0xFF;
        }
        
        // CHECK 3: Update Outputs (Only if state changed)
        if ((current_state != last_state) || timer2_triggered) {
            // 1. Update LEDs
            // Clear existing LED bits, then OR in the new pattern
            GPIOB->ODR = (GPIOB->ODR & ~states[last_state].pattern) | states[current_state].pattern | states[7 - current_state1].pattern ;
            GPIOB -> ODR = ( GPIOB -> ODR & ~states[prev_state].pattern ) | states[7 - current_state1].pattern | states[current_state].pattern; // Update LEDs 
            timer2_triggered = 0; // Reset Timer 2 flag  

            // 2. Update LCD
            lcd_command(CLEAR);
            lcd_command(0x80);
            lcd_putstring("TIM14:");
            lcd_putstring((char*)states[current_state].message); // Cast to char* to avoid warnings
            lcd_command(0xC0); // Move to second line
            lcd_putstring("TIM2:");
            lcd_putstring((char*)states[current_state1].message);

            last_state = current_state;
        }

        // Removed the unconditional delay(100000) to allow faster polling of buttons
    } 
}

//********************************************************************
// FUNCTION DEFINITIONS 
//********************************************************************
void init_GPIO(){
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN; 
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN; 
    
    // Configure PA0..PA3 as input (Switches)
    GPIOA -> MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | 
                        GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    
    // Configure PB0..PB7 as output (LEDs)
    GPIOB -> MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | 
                        GPIO_MODER_MODER2 | GPIO_MODER_MODER3 |
                        GPIO_MODER_MODER4 | GPIO_MODER_MODER5 |
                        GPIO_MODER_MODER6 | GPIO_MODER_MODER7);

    GPIOB -> MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | 
                       GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 |
                       GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
                       GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);

    // Enable pull-up resistors on PA0..PA3 (Switches)
    GPIOA -> PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | 
                         GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3); // Clear bits first
    GPIOA -> PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | 
                        GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0); // pull-up
}



void TIM14_init(){
    RCC -> APB1ENR |= RCC_APB1ENR_TIM14EN; 
    TIM14 -> PSC = 47999;    // 1ms tick (at 48MHz)
    TIM14 -> ARR = 1000;     // CHANGED: 2000 ticks = 2 Seconds
    TIM14 -> SR &= ~TIM_SR_UIF;               
    TIM14 -> DIER |= TIM_DIER_UIE;            
    NVIC_SetPriority(TIM14_IRQn, 2);          
    NVIC_EnableIRQ(TIM14_IRQn);               
    // TIM14 -> CR1 |= TIM_CR1_CEN; // Suggest removing this here and letting SW0 start it
}

void TIM14_IRQHandler(){
    if (TIM14 -> SR & TIM_SR_UIF){ 
        TIM14 -> SR &= ~TIM_SR_UIF; 
        timer_triggered = 1; // Set flag for main loop to handle      
    }
}

void TIM2_init(){
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN; 
    TIM2 -> PSC = 46999;    // 1ms tick (at 48MHz)
    TIM2 -> ARR = 200;     // 500 ticks = 0.2 Seconds
    TIM2 -> SR &= ~TIM_SR_UIF;               
    TIM2 -> DIER |= TIM_DIER_UIE;            
    NVIC_SetPriority(TIM2_IRQn, 3);          
    NVIC_EnableIRQ(TIM2_IRQn);   
    TIM2->CR1 |= TIM_CR1_CEN; // Start Timer 2 immediately            
    // TIM14 -> CR1 |= TIM_CR1_CEN; // Suggest removing this here and letting SW0 start it
}

void TIM2_IRQHandler(){
    if (TIM2 -> SR & TIM_SR_UIF){ 
        TIM2 -> SR &= ~TIM_SR_UIF; 
       timer2_triggered = 1; // Set flag for main loop to handle 
        prev_state = (7-current_state1);
        current_state1 = (current_state1 + 1) % 8; // Increment state
    }
}
//********************************************************************
// END OF PROGRAM
//********************************************************************