/**
 * @file app.c
 * @author Hoozz (huxiangjs@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "stdio.h"
#include "stdint.h"
#include "drv_usb_hw.h"
#include "cdc_acm_core.h"
#include "usart.h"
#include "systick.h"

// #define DEBUG

usb_core_driver cdc_acm;

#define I2C_FREQ_CONFIG             600000

#define I2C_TO_SERIAL_CMD_PING      0x00
#define I2C_TO_SERIAL_CMD_RESET     0x01
#define I2C_TO_SERIAL_CMD_WRITE     0x02
#define I2C_TO_SERIAL_CMD_READ      0x03
#define I2C_TO_SERIAL_CMD_WAIT      0x04

#define I2C_TO_SERIAL_CMD_MAX       5

#if defined(DEBUG)
static const char *cmd_desc[I2C_TO_SERIAL_CMD_MAX] = {"PING", "RESET", "WRITE", "READ", "WAIT"};
#endif

static void i2c_config(void)
{
	/* GPIO */
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7);
	gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_6 | GPIO_PIN_7);
	// gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_PIN_6 | GPIO_PIN_7);

	/* I2C */
	rcu_periph_clock_enable(RCU_I2C0);
	i2c_deinit(I2C0);
	// I2C Clock
	i2c_clock_config(I2C0, I2C_FREQ_CONFIG, I2C_DTCY_2);
	i2c_enable(I2C0);
}

static int8_t i2c_probe(uint8_t addr)
{
	int8_t ret;

	/* Wait for the bus to be free */
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) == SET);
	if (!delay_get_ms(0))
		return -1;
	
	/* Send start signal */
	i2c_start_on_bus(I2C0);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_SBSEND) == RESET);
	if (!delay_get_ms(0)){
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	
	/* Send slave device address */
	i2c_master_addressing(I2C0, addr << 1, I2C_TRANSMITTER);
	// while (i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) == RESET);
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

	/* Send stop signal */
	i2c_stop_on_bus(I2C0);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) == SET);
	if (!delay_get_ms(0))
		return -1;

	/* Make sure the ACK is OK */
	ret = (int8_t)(i2c_flag_get(I2C0, I2C_FLAG_AERR) == RESET);
	i2c_flag_clear(I2C0, I2C_FLAG_AERR);

	return ret;
}

static void i2c_scan(void)
{
	uint8_t i, j;

	printf("   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
	for (i=0; i<8; i++) {
		printf("%02X ", i<<4);
		for (j=0; j<=0xF; j++) {
			switch (i2c_probe(i<<4 | j)) {
			case 1:
				printf("%02X ", i<<4 | j);
				break;
			case 0:
				printf("-- ");
				break;
			case -1:
			default:
				printf("   ");
			}
		}
		printf("\r\n");
	}
}

int MLX90640_I2CGeneralReset(void)
{
	int ret;

	/* Wait for the bus to be free */
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) == SET);
	if (!delay_get_ms(0))
		return -1;
	
	/* Send start signal */
	i2c_start_on_bus(I2C0);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_SBSEND) == RESET);
	if (!delay_get_ms(0)){
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* Send slave device address */
	i2c_master_addressing(I2C0, 0x00 << 1, I2C_TRANSMITTER);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

	i2c_data_transmit(I2C0, 0x06);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_TBE) == RESET);
	if (!delay_get_ms(0)){
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	
	/* Send stop signal */
	i2c_stop_on_bus(I2C0);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) == SET);
	if (!delay_get_ms(0))
		return -1;

	/* Make sure the ACK is OK */
	ret = i2c_flag_get(I2C0, I2C_FLAG_AERR) == RESET ? 0 : -1;
	i2c_flag_clear(I2C0, I2C_FLAG_AERR);

	return ret;
}

int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
	uint16_t n;
	
	/* Wait for the bus to be free */
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) == SET);
	if (!delay_get_ms(0))
		return -1;
	
	/* Send start signal */
	i2c_start_on_bus(I2C0);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_SBSEND) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* Send slave device address (Write) */
	i2c_master_addressing(I2C0, slaveAddr << 1, I2C_TRANSMITTER);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

	/* High bit of on-chip address */
	i2c_data_transmit(I2C0, startAddress >> 8);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_TBE) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* Low bit of on-chip address */
	i2c_data_transmit(I2C0, startAddress & 0xFF);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_TBE) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* Retransmit start signal */
	i2c_start_on_bus(I2C0);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_SBSEND) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* Retransmit slave device address (Read) */
	i2c_master_addressing(I2C0, slaveAddr << 1, I2C_RECEIVER);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

	/* Receive data */
	n = nMemAddressRead - 1;
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);	
	while (n--) {
		/* High bit of data */
		delay_set_ms(0, 2);
		while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_RBNE) == RESET);
		if (!delay_get_ms(0)) {
			i2c_stop_on_bus(I2C0);
			return -1;
		}
		*data = i2c_data_receive(I2C0) << 8;

		/* Low bit of data */
		delay_set_ms(0, 2);
		while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_RBNE) == RESET);
		if (!delay_get_ms(0)) {
			i2c_stop_on_bus(I2C0);
			return -1;
		}
		*data |= i2c_data_receive(I2C0);
		data++;
	}
	/* High bit of data */
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_RBNE) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	*data = i2c_data_receive(I2C0) << 8;

	i2c_ack_config(I2C0, I2C_ACK_DISABLE);
	/* Low bit of data */
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_RBNE) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	*data |= i2c_data_receive(I2C0);

	/* Send stop signal */
	i2c_stop_on_bus(I2C0);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) == SET);
	if (!delay_get_ms(0))
		return -1;

	return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data)
{
	int ret;

	/* Wait for the bus to be free */
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) == SET);
	if (!delay_get_ms(0))
		return -1;
	
	/* Send start signal */
	i2c_start_on_bus(I2C0);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_SBSEND) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* Send slave device address */
	i2c_master_addressing(I2C0, slaveAddr << 1, I2C_TRANSMITTER);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

	/* High bit of on-chip address */
	i2c_data_transmit(I2C0, writeAddress >> 8);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_TBE) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* Low bit of on-chip address */
	i2c_data_transmit(I2C0, writeAddress & 0xFF);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_TBE) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* High bit of data */
	i2c_data_transmit(I2C0, data >> 8);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_TBE) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}

	/* Low bit of data */
	i2c_data_transmit(I2C0, data & 0xFF);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_TBE) == RESET);
	if (!delay_get_ms(0)) {
		i2c_stop_on_bus(I2C0);
		return -1;
	}
	
	/* Send stop signal */
	i2c_stop_on_bus(I2C0);
	delay_set_ms(0, 2);
	while (delay_get_ms(0) && i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) == SET);
	if (!delay_get_ms(0))
		return -1;

	/* Make sure the ACK is OK */
	ret = i2c_flag_get(I2C0, I2C_FLAG_AERR) == RESET ? 0 : -1;
	i2c_flag_clear(I2C0, I2C_FLAG_AERR);

	return ret;
}

int MLX90640_WaitFrame(uint8_t slaveAddr)
{
	uint16_t statusRegister;
	int error;

	do {
		error = MLX90640_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
		if(error)
			return error;
	} while (!(statusRegister & 0x0008));

	return 0;
}

#if defined(DEBUG)
static void hex_dump(void *data, uint32_t size)
{
	uint8_t *p = (uint8_t *)data;
	while (size--)
		printf("%02X ", *p++);
	printf("\r\n");
}
#endif

int main(void)
{
	int ret;
	uint32_t count = 0;
	uint16_t send_size;
	uint8_t slaveAddr;
	uint16_t startAddress;
	uint16_t nMemAddressRead;
	uint16_t writeData;
	
	/* select HXTAL as PLL preselection clock */
	rcu_pll_preselection_config(RCU_PLLPRESEL_HXTAL);

	systick_config();
	usart_config();

	printf("   system clock source: %s\r\n",
		rcu_system_clock_source_get() == RCU_SCSS_IRC8M ? "RCU_SCSS_IRC8M" :
		rcu_system_clock_source_get() == RCU_SCSS_HXTAL ? "RCU_SCSS_HXTAL" :
		rcu_system_clock_source_get() == RCU_SCSS_PLL ? "RCU_SCSS_PLL" : "Unknown");
	printf("system clock frequency: %9lu Hz\r\n", rcu_clock_freq_get(CK_SYS));
	printf("   AHB clock frequency: %9lu Hz\r\n", rcu_clock_freq_get(CK_AHB));
	printf("  APB1 clock frequency: %9lu Hz\r\n", rcu_clock_freq_get(CK_APB1));
	printf("  APB2 clock frequency: %9lu Hz\r\n", rcu_clock_freq_get(CK_APB2));
	printf("   ADC clock frequency: %9lu Hz\r\n", rcu_clock_freq_get(CK_ADC));
	printf("   CEC clock frequency: %9lu Hz\r\n", rcu_clock_freq_get(CK_CEC));
	printf(" USART clock frequency: %9lu Hz\r\n", rcu_clock_freq_get(CK_USART));

	i2c_config();
	printf("     I2C SCL frequency: %9lu Hz\r\n", I2C_FREQ_CONFIG);
	i2c_scan();

	usb_rcu_config();
	usb_timer_init();
	usbd_init(&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);
	usb_intr_config();

	count = 0;
	while (1) {
		if (USBD_CONFIGURED == cdc_acm.dev.cur_status) {
			if (NULL != cdc_acm.dev.class_data[CDC_COM_INTERFACE]) {
				usb_cdc_handler *cdc = (usb_cdc_handler *)cdc_acm.dev.class_data[CDC_COM_INTERFACE];
				send_size = 0;
				ret = 0;

				/* Receive */
				if (1U == cdc->packet_receive) {
					cdc_acm_data_receive(&cdc_acm);
					if (cdc->receive_length > 0) {
#if defined(DEBUG)
						printf("R: ");
						hex_dump(cdc->data, cdc->receive_length);
						printf("CMD: %s\r\n", cdc->data[0] < I2C_TO_SERIAL_CMD_MAX ?
						       cmd_desc[cdc->data[0]] : "UNKNOWN");

#endif
						switch (cdc->data[0]) {
						case I2C_TO_SERIAL_CMD_PING:
						case I2C_TO_SERIAL_CMD_RESET:
							send_size = 1 + 1;
							ret = MLX90640_I2CGeneralReset();
							cdc->data[1] = ret ? 0xff : 0;
							if (ret)
								i2c_scan();
							break;
						case I2C_TO_SERIAL_CMD_WRITE:
							if (cdc->receive_length < 6) {
#if defined(DEBUG)
								printf("write cmd fail\r\n");
#endif
								break;
							}
							slaveAddr = cdc->data[1];
							startAddress = *((uint16_t *)(cdc->data + 2));
							writeData = *((uint16_t *)(cdc->data + 4));
#if defined(DEBUG)
							printf("slaveAddr:%02x, startAddress:%04x, writeData:%04x\r\n",
							       slaveAddr, startAddress, writeData);
#endif
							send_size = 1 + 1;
							ret =  MLX90640_I2CWrite(slaveAddr, startAddress, writeData);
							cdc->data[1] = ret ? 0xff : 0;
							break;
						case I2C_TO_SERIAL_CMD_READ:
							if (cdc->receive_length < 6) {
#if defined(DEBUG)
								printf("read cmd fail\r\n");
#endif
								break;
							}
							slaveAddr = cdc->data[1];
							startAddress = *((uint16_t *)(cdc->data + 2));
							nMemAddressRead = *((uint16_t *)(cdc->data + 4));
#if defined(DEBUG)
							printf("slaveAddr:%02x, startAddress:%04x, nMemAddressRead:%04x\r\n",
							       slaveAddr, startAddress, nMemAddressRead);
#endif
							send_size = 1 + 1;
							ret = MLX90640_I2CRead(slaveAddr, startAddress,
									nMemAddressRead,
									(uint16_t *)(cdc->data + send_size));
							send_size += ret ? 0 : nMemAddressRead * 2;
							cdc->data[1] = ret ? 0xff : 0;
							break;
						case I2C_TO_SERIAL_CMD_WAIT:
							if (cdc->receive_length < 2) {
#if defined(DEBUG)
								printf("wait cmd fail\r\n");
#endif
								break;
							}
							slaveAddr = cdc->data[1];
#if defined(DEBUG)
							printf("slaveAddr:%02x\r\n", slaveAddr);
#endif
							send_size = 1 + 1;
							ret = MLX90640_WaitFrame(slaveAddr);
							cdc->data[1] = ret ? 0xff : 0;
							break;
						}
#if defined(DEBUG)
						printf("cmd result: %d\r\n\r\n", ret);
#endif
					}
				}

				/* Send */
				// send_size = USB_CDC_RX_LEN; // For test
				if (send_size) {
					count += send_size;
					cdc->packet_sent = 0U;
					usbd_ep_send(&cdc_acm, CDC_DATA_IN_EP, (uint8_t*)(cdc->data), send_size);
#if defined(DEBUG)
					printf("W: ");
					hex_dump(cdc->data, send_size);
#endif
					delay_set_ms(0, 500);
					while (delay_get_ms(0) && 1U != cdc->packet_sent);
					if (!delay_get_ms(0))
						printf("tx timeout\r\n");

				}
			}
		}

		/* display USB tx rate */
		if (!delay_get_ms(1)) {
			printf("tx: %6lubyte/s\r\n", count);
			count = 0;
			delay_set_ms(1, 1000);
		}
	}
}
