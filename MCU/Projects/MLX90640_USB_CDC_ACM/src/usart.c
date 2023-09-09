#include "stdio.h"
#include "gd32f3x0.h"

void usart_config(void)
{
	/* GPIO */
	rcu_periph_clock_enable(RCU_GPIOA);
	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9 | GPIO_PIN_10);
	gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9 | GPIO_PIN_10);
	
	/* USART */
	rcu_periph_clock_enable(RCU_USART0);
	usart_deinit(USART0);
	usart_clock_enable(USART0);
	usart_baudrate_set(USART0, 115200);
	usart_parity_config(USART0, USART_PM_NONE);
	usart_word_length_set(USART0, USART_WL_8BIT);
	usart_stop_bit_set(USART0, USART_STB_1BIT);
	usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
	usart_receive_config(USART0, USART_RECEIVE_ENABLE);
	usart_receive_fifo_enable(USART0);
	usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
	usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);

	usart_enable(USART0);
}

int fputc(int ch, FILE *f)
{
	usart_data_transmit(USART0, (uint16_t)ch);
	while (usart_flag_get(USART0, USART_FLAG_TC) == RESET);
	return ch;
}

#if 0
/* It seems to have some problems */
int fgetc(FILE * f)
{
	while (usart_flag_get(USART0, USART_FLAG_RBNE) == RESET);
	return (int)usart_data_receive(USART0);
}
#endif
