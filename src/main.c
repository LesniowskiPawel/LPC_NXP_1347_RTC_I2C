/*
 * @brief I2C example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
|||||||||||||||||||| READ THIS ! ||||||||||||||||||||||||||||
	CHANGES: SIMPLY I2C RTC DATA READ / WRITE  & SEND VIA USB TO TERMINAL ON REQUEST- 30.05.2019
	Author: Pv

	LPC1347 :: PIO0_4 -> SCL :: SDA->PIO0_5
	PCF : https://www.nxp.com/docs/en/data-sheet/PCF8563.pdf

	SETTINGS FOR SERIAL CONNECTIONS : 9600, 8b, 1b stop
	CHECK FILE CDC_UART.C - IN LINE 274 IT THIS INCLUDED HOW SEND DATA VIA USB

	I2C - RTC ADRESS -> YOU CAN NEED CHANGE ADRESS IN LINE NUMBER 392 OF THIS CODE
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
 */

#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "i2c_13xx.h"
#include "app_usbd_cfg.h"
#include "cdc_uart.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
uint8_t sec, min, hours; // variables time's

static USBD_HANDLE_T g_hUsb;
const  USBD_API_T *g_pUsbApi;

#define DEFAULT_I2C          I2C0

#define I2C_EEPROM_BUS       DEFAULT_I2C
#define I2C_IOX_BUS          DEFAULT_I2C

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000
#define N 15

#ifdef DEBUG_ENABLE

#endif

static int mode_poll;   /* Poll/Interrupt mode flag */
static I2C_ID_T i2cDev = DEFAULT_I2C; /* Currently active I2C device */

//--------------------- PIN FOR INTERRUPT EXT. -------------------------
#define GPIO_PININT_PIN     19	/* GPIO pin number mapped to PININT */
#define GPIO_PININT_PORT    1	/* GPIO port number mapped to PININT */
#define GPIO_PININT_INDEX   0	/* PININT index used for GPIO mapping */
#define PININT_IRQ_HANDLER  PIN_INT0_IRQHandler	/* GPIO interrupt IRQ function name */
#define PININT_NVIC_NAME    PIN_INT0_IRQn	/* GPIO interrupt NVIC interrupt name */

/* Data area for slave operations */
static uint8_t buffer[2][256];
static uint8_t iox_data[2]; /* PORT0 input port, PORT1 Output port */
static volatile uint32_t tick_cnt;


/*****************************************************************************
 * GLOBAL types/enumerations/variables
 ****************************************************************************/
int tmp;
static I2C_XFER_T xfer;

int flags;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

void PININT_IRQ_HANDLER(void);

static void Init_I2C_PinMux(void)
{
	/* Pin muxing set by board layer library.  (See board_sysinit.c files in
	   ./software/lpc_core/lpc_board/boards_13xx/...) */
}

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

/* Print data to console */
static void con_print_data(const uint8_t *dat, int sz)
{
	int i;
	if (!sz) return;
	for (i = 0; i < sz; i++) {
		if (!(i & 0xFF)) DEBUGOUT(" %02X: ", i);
			DEBUGOUT(" %02X ", dat[i]);
	}
	DEBUGOUT("|\r\n");
}

static void i2c_read_input(I2C_XFER_T *xfer)
{
	int tmp;

	tmp = 0xA1; // 1 0 1 0 0 0 0 adres RTC - 50 (10)
	tmp &= 0xFF;
	xfer->slaveAddr = tmp;
	xfer->rxBuff = 0;

		tmp = 8;
		tmp &= 0xFF;
		xfer->rxSz = tmp;
		xfer->rxBuff = buffer[1];

}

static void i2c_write_input(I2C_XFER_T *xfer) {
		int tmp, i;
		tmp = 4;
		tmp &= 0xFF;
		for (i = 0; i < tmp; i++) {
		xfer->txSz = tmp;
		xfer->txBuff = buffer[1];

	}
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(I2C0_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(I2C0_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	Init_I2C_PinMux();

	Chip_SYSCTL_PeriphReset(RESET_I2C0); //new
	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}

/* Function that probes all available slaves connected to an I2C bus */
static void i2c_probe_slaves(I2C_ID_T i2c)
{
	int i;
	uint8_t ch[4];

	DEBUGOUT("Searching available I2C devices... Wait\r\n");
	DEBUGOUT("\r\n|    00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
	DEBUGOUT("\r\n|___________________________________________________");
	for (i = 0; i <= 0x7F; i++) {
		if (!(i & 0x0F)) DEBUGOUT("\r\n%02X  ", i >> 4);
		if (i <= 7 || i > 0x78) {
			DEBUGOUT("   ");
			continue;
		}
		/* Address 0x48 points to LM75AIM device which needs 2 bytes be read */
		if(Chip_I2C_MasterRead(i2c, i, ch, 1 + (i == 0x50)) > 0) {
			DEBUGOUT(" 0x%02X", i); }
		else
			DEBUGOUT(" --");
	}
	DEBUGOUT("\r\n");
}

/*-------- IO Expansion slave device implementation ----------*/
/* Update IN/OUT port states to real devices */
void i2c_iox_update_regs(int ops)
{
	if (ops & 1) { /* update out port */
		Board_LED_Set(0, iox_data[1] & 1);
		Board_LED_Set(1, iox_data[1] & 2);
		Board_LED_Set(2, iox_data[1] & 4);
		Board_LED_Set(3, iox_data[1] & 8);
	}
}

/*-------------------- End of IO Expansion slave device ----------------------*/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	SysTick Interrupt Handler
 * @return	Nothing
 * @note	Systick interrupt handler updates the button status
 */
void SysTick_Handler(void)
{
	i2c_iox_update_regs(2);
	tick_cnt ++;
}

/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

void USB_IRQHandler(void)
{
	uint32_t *addr = (uint32_t *) LPC_USB->EPLISTSTART;

	/*	WORKAROUND for artf32289 ROM driver BUG:
	    As part of USB specification the device should respond
	    with STALL condition for any unsupported setup packet. The host will send
	    new setup packet/request on seeing STALL condition for EP0 instead of sending
	    a clear STALL request. Current driver in ROM doesn't clear the STALL
	    condition on new setup packet which should be fixed.
	 */
	if ( LPC_USB->DEVCMDSTAT & _BIT(8) ) {	/* if setup packet is received */
		addr[0] &= ~(_BIT(29));	/* clear EP0_OUT stall */
		addr[2] &= ~(_BIT(29));	/* clear EP0_IN stall */
	}
	USBD_API->hw->ISR(g_hUsb);
}

/* Find the address of interface descriptor for given class type. */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass)
{
	USB_COMMON_DESCRIPTOR *pD;
	USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
	uint32_t next_desc_adr;

	pD = (USB_COMMON_DESCRIPTOR *) pDesc;
	next_desc_adr = (uint32_t) pDesc;

	while (pD->bLength) {
		/* is it interface descriptor */
		if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

			pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
			/* did we find the right interface descriptor */
			if (pIntfDesc->bInterfaceClass == intfClass) {
				break;
			}
		}
		pIntfDesc = 0;
		next_desc_adr = (uint32_t) pD + pD->bLength;
		pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
	}

	return pIntfDesc;
}

/**
 * @brief	Main program body
 * @return	int
 */

int main(void)
{

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	DEBUGOUT("\r\n RTC FOR LPC1347 - P. Lesniowski - D. Zygadlo - L. Dyl \r\n"); // We editing this code, but we are author's - don't delete this

	//--------------------- USB CONNECTIONS INIT & SETTINGS ------------------------------------
	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	ErrorCode_t ret = LPC_OK;

	/* enable clocks and pinmux */
		Chip_USB_Init();

	/* initialize USBD ROM API pointer. */
		g_pUsbApi = (const USBD_API_T *) LPC_ROM_API->usbdApiBase;

		/* initialize call back structures */
		memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
		usb_param.usb_reg_base = LPC_USB0_BASE;
		/*	WORKAROUND for artf44835 ROM driver BUG:
		    Code clearing STALL bits in endpoint reset routine corrupts memory area
		    next to the endpoint control data. For example When EP0, EP1_IN, EP1_OUT,
		    EP2_IN are used we need to specify 3 here. But as a workaround for this
		    issue specify 4. So that extra EPs control structure acts as padding buffer
		    to avoid data corruption. Corruption of padding memory doesn’t affect the
		    stack/program behaviour.
		 */
		usb_param.max_num_ep = 3 + 1;
		usb_param.mem_base = USB_STACK_MEM_BASE;
		usb_param.mem_size = USB_STACK_MEM_SIZE;

		/* Set the USB descriptors */
		desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
		desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
		/* Note, to pass USBCV test full-speed only devices should have both
		   descriptor arrays point to same location and device_qualifier set to 0.
		 */
		desc.high_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
		desc.full_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
		desc.device_qualifier = 0;

		/* USB Initialization */
		ret = USBD_API->hw->Init(&g_hUsb, &desc, &usb_param);
		if (ret == LPC_OK) {
			DEBUGOUT("\r\n USB INIT OK !\r\n");
			/*	WORKAROUND for artf32219 ROM driver BUG:
			    The mem_base parameter part of USB_param structure returned
			    by Init() routine is not accurate causing memory allocation issues for
			    further components.
			 */
			usb_param.mem_base = USB_STACK_MEM_BASE + (USB_STACK_MEM_SIZE - usb_param.mem_size);

			/* Init UCOM - USB to UART bridge interface */
			ret = UCOM_init(g_hUsb, &desc, &usb_param);
			if(ret == 0) { DEBUGOUT("\r\n USB LPC_OK !\r\n"); }
			if (ret == LPC_OK) {
				/* Make sure USB and UART IRQ priorities are same for this example */
				NVIC_SetPriority(USB0_IRQn, 1);
				/*  enable USB interrupts */
				NVIC_EnableIRQ(USB0_IRQn);
				/* now connect */
				USBD_API->hw->Connect(g_hUsb, 1);
			}
		}

	//------------------------------END---------------------------------------

	//--------------------------I2C INIT & SETTINGS---------------------------------------
	i2c_app_init(I2C0, SPEED_100KHZ);
	//i2c_set_mode(i2cDev, 0);
	DEBUGOUT("---------------------------------------\r\n");
	i2c_probe_slaves(i2cDev); //check bus
	DEBUGOUT("\r\n");
	DEBUGOUT("\r\n Slave address is 0x%02X", 0x50);

	//---------------------------------------------------------------------------
		Board_LED_Set(0, true); // LED 2 ON and show status

			/* Configure GPIO pin as input */
			Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, GPIO_PININT_PORT, GPIO_PININT_PIN);

			/* Configure pin as GPIO with pullup */
			Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_PININT_PORT, GPIO_PININT_PIN,
								 (IOCON_FUNC0 | IOCON_MODE_PULLUP));

			/* Enable PININT clock */
			Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PINT);

			/* Configure interrupt channel for the GPIO pin in SysCon block */
			Chip_SYSCTL_SetPinInterrupt(GPIO_PININT_INDEX, GPIO_PININT_PORT, GPIO_PININT_PIN);

			/* Configure channel interrupt as edge sensitive and falling edge interrupt */
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_PININT_INDEX));
			Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(GPIO_PININT_INDEX));
			Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(GPIO_PININT_INDEX));

			/* Enable interrupt in the NVIC */
			NVIC_ClearPendingIRQ(PININT_NVIC_NAME);
			NVIC_EnableIRQ(PININT_NVIC_NAME);
			//------------------------------END---------------------------------------

	//loop endless
	while(1) {

		__WFI();

		if(flags) { // IF TRUE - READ TIME AND SHOW ON CONSOLE IN PROGRAM (dependent on the interruption)
			i2c_read_input(&xfer);
				tmp=Chip_I2C_MasterCmdRead(i2cDev, 0x50, 1, xfer.rxBuff, xfer.rxSz); // READ TIME - THERE YOU CHANGE ADRESS - SECOND PARAMETER (0x50)

					DEBUGOUT("\r\nRead %d bytes of data from slave 0x%02X.\r\n", tmp, 0x50);
					DEBUGOUT(" SEC\t| MIN\t| HOURS\t| DAYS\t| WEEK\t| MONTH\t| YEARS\t|\r\n"); // SHOW REGISTERS

						for (int i = 1; i < tmp; i++) {

								DEBUGOUT(" %02X\t|", xfer.rxBuff[i]);
						}
						DEBUGOUT("|\r\n");
						sec = (xfer.rxBuff[1]&0x0F) + (10*(xfer.rxBuff[1]>>4));
						min = (xfer.rxBuff[2]&0x0F) + (10*(xfer.rxBuff[2]>>4));
						hours = (xfer.rxBuff[3]&0x0F) + (10*(xfer.rxBuff[3]>>4));

						DEBUGOUT("  \r\n%d : %d : %d\r\n",hours, min, sec); // SHOW TIME

		}

		else { flags = 0; }
	}

	Chip_I2C_DeInit(I2C0);

	return 0;
}

void PININT_IRQ_HANDLER(void)
{
			flags = 1; 																// READ AND SHOW TIME FROM RTC
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_PININT_INDEX));
			Board_LED_Toggle(0); 													// TOGGLE LED FROM INTERRUPT FROM RTC
}
