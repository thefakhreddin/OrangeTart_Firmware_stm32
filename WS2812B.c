#include "WS2812B.h"
#include "string.h"

#define LED_NUM 6
#define T0H 13
#define T1H 26

struct rgb_value {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

volatile static struct rgb_value rgb_values[LED_NUM] = {0};
volatile static uint8_t ws_data[2 * 24] = {0};

static void WS_UpdateData(uint8_t index);
static struct rgb_value WS_CodeToRGB(LEDColor led_color);

void WS2812B_Init(void)
{	
	// ##### TIM16 (WS2812B) #####
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; /* Enable TIM16 clock */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* Enable DMA1 clock */
	
	TIM16->PSC = 0;
	TIM16->ARR = 39;
	
	TIM16->CR1 |= TIM_CR1_ARPE; /* Auto-reload preload enable */
	TIM16->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	TIM16->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; /* PWM mode 1 and Preload enable */
	
	DMA1_Channel3->CCR = 0;
	DMA1_Channel3->CPAR = (uint32_t) (&(TIM16->DMAR)); /* (1) */
	DMA1_Channel3->CMAR = (uint32_t) (ws_data); /* (2) */
	DMA1_Channel3->CNDTR = sizeof(ws_data) / sizeof(uint8_t); /* (3) */
	DMA1_Channel3->CCR |= DMA_CCR_CIRC | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_DIR; /* (4) */
	
	TIM16->DCR = (0x00U << TIM_DCR_DBL_Pos) + ((((uint32_t) (&TIM16->CCR1)) - ((uint32_t) (&TIM16->CR1))) >> 2); /* (5) */
	TIM16->DIER |= TIM_DIER_UDE; /* (6) */
	TIM16->EGR |= TIM_EGR_UG;
	while((TIM16->EGR & TIM_EGR_UG) == TIM_EGR_UG); /* wait until the RESET of UG bit*/
	/* Enable UEV by setting UG bit to load data from preload to active registers */
	TIM16->EGR |= TIM_EGR_UG;
	TIM16->BDTR |= TIM_BDTR_MOE; /* Enable the TIM3 Main Output */
	TIM16->CCER |= TIM_CCER_CC1E; /* Enable capture/compare 1 output */
}

void WS_UpdateData(uint8_t index)
{
	if (index == LED_NUM)
	{
		for (uint8_t i = 0; i < 24; i++)
			ws_data[i] = 0;
	}
	else if (index == LED_NUM + 1)
	{
		for (uint8_t i = 0; i < 24; i++)
			ws_data[i + 24] = 0;
	}
	else if (index > LED_NUM + 1)
	{
		return;
	}
	else
	{
		if (index % 2)
		{
			for (uint8_t i = 0; i < 8; i++)
				ws_data[i + 24] = (rgb_values[index].g >> (7 - i)) & 1 ? T1H : T0H;
			
			for (uint8_t i = 0; i < 8; i++)
				ws_data[i + 8 + 24] = (rgb_values[index].r >> (7 - i)) & 1 ? T1H : T0H;
			
			for (uint8_t i = 0; i < 8; i++)
				ws_data[i + 16 + 24] = (rgb_values[index].b >> (7 - i)) & 1 ? T1H : T0H;
		}
		else
		{
			for (uint8_t i = 0; i < 8; i++)
				ws_data[i] = (rgb_values[index].g >> (7 - i)) & 1 ? T1H : T0H;
			
			for (uint8_t i = 0; i < 8; i++)
				ws_data[i + 8] = (rgb_values[index].r >> (7 - i)) & 1 ? T1H : T0H;
			
			for (uint8_t i = 0; i < 8; i++)
				ws_data[i + 16] = (rgb_values[index].b >> (7 - i)) & 1 ? T1H : T0H;
		}
	}
}

void WS_Send(void)
{
	WS_UpdateData(0);
	WS_UpdateData(1);
	
	DMA1_Channel3->CCR |= DMA_CCR_EN; /* Enable DMA channel 3 */
	TIM16->CR1 |= TIM_CR1_CEN; /* Enable TIM16 */
	
	for (int i = 0; i <= LED_NUM;)
	{
		while ((DMA1->ISR & DMA_ISR_HTIF3) != DMA_ISR_HTIF3); /* Wait until DMA half transfer is complete */
		DMA1->IFCR |= DMA_IFCR_CHTIF3 /* Clear channel 3 interrupt flag */;
		WS_UpdateData(i++ + 2);
		
		while ((DMA1->ISR & DMA_ISR_TCIF3) != DMA_ISR_TCIF3); /* Wait until DMA transfer is complete */
		DMA1->IFCR |= DMA_IFCR_CTCIF3 /* Clear channel 3 interrupt flag */;
		WS_UpdateData(i++ + 2);
	}
	
	DMA1_Channel3->CCR &= ~DMA_CCR_EN; /* Disable DMA channel 3 */
	DMA1_Channel3->CNDTR = sizeof(ws_data) / sizeof(uint8_t); /* Reset counter */
	TIM16->CR1 &= ~TIM_CR1_CEN; /* Disable TIM16 */
	TIM16->CNT = 0; /* Reset counter */
}


void WS_Queue(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
	if (index > LED_NUM - 1)
	{
		/* Invalid LED index */
		return;
	}
	
	rgb_values[index].r = r;
	rgb_values[index].g = g;
	rgb_values[index].b = b;
}

void WS_QueueSimple(LEDIndex led_index, LEDColor led_color)
{
  if (led_index == LED_ALL)
  {
		for (uint8_t i = 0; i < LED_NUM; i++)
		{
			struct rgb_value c = WS_CodeToRGB(led_color);
			WS_Queue(i, c.r, c.g, c.b);
		}
  }
	else
	{
		struct rgb_value c = WS_CodeToRGB(led_color);
		WS_Queue(led_index - 1, c.r, c.g, c.b);
	}
}

struct rgb_value WS_CodeToRGB(LEDColor led_color)
{
	struct rgb_value c;
  switch (led_color)
  {
    case OFF:
			c.r = 0;
			c.g = 0;
			c.b = 0;
      break;
    case WHITE:
			c.r = 50;
			c.g = 50;
			c.b = 50;
      break;
    case PURPLE:
			c.r = 50;
			c.g = 0;
			c.b = 100;
      break;
    case BLUE:
			c.r = 0;
			c.g = 0;
			c.b = 140;
      break;
    case GREEN:
			c.r = 0;
			c.g = 140;
			c.b = 0;
      break;
    case YELLOW:
			c.r = 80;
			c.g = 80;
			c.b = 0;
      break;
    case ORANGE:
			c.r = 120;
			c.g = 32;
			c.b = 0;
      break;
    case RED:
			c.r = 140;
			c.g = 0;
			c.b = 0;
      break;
    case PINK:
			c.r = 80;
			c.g = 6;
			c.b = 46;
      break;
    default:
      break;
  }
	return c;
}
