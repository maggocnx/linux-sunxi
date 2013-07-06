#define KEYPAD_IRQ_NO      	(24)

#define __REG(x)                        (*(volatile unsigned int *)(x))
#define PIOC_REGS_BASE SW_VA_PORTC_IO_BASE
#define PIO_B 	2
#define PIO_C	3
#define PIO_D 	4
#define PIO_E 	5
#define PIO_F 	6
#define PIO_G 	7


//PORT Eq
#define SPI_CS0 	0
#define SPI_SCLK 	1
#define SPI_MOSI 	2
#define SPI_MISO 	3

#define PH_1A 4
#define PH_1B 5
#define PH_2A 6
#define PH_2B 7

#define SPI_SEL_0   8
#define SPI_SEL_1   9
#define PER_RST   15




#define KEYPAD_IRQ_PIN   10

#define PIN_MODE_INPUT 	0
#define PIN_MODE_OUTPUT 	1
#define PIN_MODE_SPI 	4
#define PIN_MODE_INT 	6

#define PULL_MODE_DISABLE  0
#define PULL_MODE_UP 	1
#define PULL_MODE_DOWN 	2

#define PIO_REG_CFG(n, i)               ((volatile unsigned int *)(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x00))
#define PIO_REG_DLEVEL(n, i)            ((volatile unsigned int *)(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x14))
#define PIO_REG_PULL(n, i)              ((volatile unsigned int *)(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x1C))
#define PIO_REG_DATA(n)                   ((volatile unsigned int *)(PIOC_REGS_BASE + ((n)-1)*0x24 + 0x10))
#define SPI_A13_REG(n)                   ((volatile unsigned int *)(SW_VA_SPI2_IO_BASE  + (n)))
#define CCM_A13_REG(n)                   ((volatile unsigned int *)(SW_VA_CCM_IO_BASE  + (n)))



#define PIO_REG_CFG_VALUE(n, i)          __REG(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x00)
#define PIO_REG_DLEVEL_VALUE(n, i)       __REG(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x14)
#define PIO_REG_PULL_VALUE(n, i)         __REG(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x1C)
#define PIO_REG_DATA_VALUE(n)            __REG(PIOC_REGS_BASE + ((n)-1)*0x24 + 0x10)
#define SPI_A13_REG_VALUE(n)            __REG(SW_VA_SPI2_IO_BASE + (n))


#define SPI_RXDATA		0x00 //SPI RX Data Register
#define SPI_TXDATA		0x04 //SPI TX Data Register
#define SPI_CTL		0x08 //SPI Control Register
#define SPI_INTCTL		0x0C //SPI Interrupt Control Register
#define SPI_ST			0x10 //SPI Status Register
#define SPI_DMACTL		0x14 //SPI DMA Control Register
#define SPI_WAIT		0x18 //SPI Wait Clock Counter Register
#define SPI_CCTL		0x1C //SPI Clock Rate Control Register
#define SPI_BC			0x20 //SPI Burst Counter Register
#define SPI_TC			0x24 //Spi Transmit Counter Register
#define SPI_FIFO_STA	0x28 //SPI FIFO Status Register


#define MCP_IODIR	0x00		 
#define MCP_IPOL	0x02
#define MCP_GPINTEN	0x04
#define MCP_DEFVAL	0x06
#define MCP_INTCON	0x08
#define MCP_IOCON	0x0A

#define IOCON_BANK	(1 << 7)
#define IOCON_SEQOP	(1 << 5)
#define IOCON_HAEN	(1 << 3)
#define IOCON_ODR	(1 << 2)
#define IOCON_INTPOL	(1 << 1)

#define MCP_GPPU	0x0C
#define MCP_INTF	0x0E
#define MCP_INTCAP	0x10
#define MCP_GPIO	0x12
#define MCP_OLAT	0x14

#define GPIO_A 		0x00
#define GPIO_B 		0x01

#define BB_SPI2_CS0			(0x00000001)
#define BB_SPI2_CLK			(0x00000002)
#define BB_SPI2_MOSI		(0x00000004)
#define BB_SPI2_MISO		(0x00000008)
#define SPISEL0				(0x00000100)
#define SPISEL1				(0x00000101)

#define MCP_WRITE 0x00400000
#define MCP_READ 0x00410000

typedef struct {
	u8	mcpreg;
	u8	mcpvalu;	
} ADR_REG;




#define NUMBER_OF_COLUMNS 		128
#define NUMBER_OF_ROWS 			64
#define NUMBER_OF_PAGES 		8
#define FIRST_PAGE 				0
#define FIRST_COLUMN 			0
#define LAST_PAGE 				7
#define LAST_COLUMN 			127

#define NORMAL       			0
#define INVERSE       			0x04
#define BIGSIZE       			0x02
#define UNDERLN       			0x80

/*---------------------------------------------------------------------------*/

									/* The following definitions are the command codes that are passed to the display via the data bus. */
#define DISPLAY_ON 				0xAF
#define DISPLAY_OFF 			0xAE
#define START_LINE_SET 			0x40
#define PAGE_ADDRESS_SET 		0xB0
									/* The Column Address is a two byte operation that writes the most significant
									bits of the address to D3 - D0 and then writes the least significant bits to
									D3- D0. Since the Column Address auto increments after each write, direct
									access is infrequent. */
#define COLUMN_ADDRESS_HIGH 	0x10
#define COLUMN_ADDRESS_LOW 		0x00
#define ADC_SELECT_NORMAL 		0xA0
#define ADC_SELECT_REVERSE 		0xA1
#define DISPLAY_NORMAL 			0xA6
#define DISPLAY_REVERSE 		0xA7
#define ALL_POINTS_ON 			0xA5
#define LCD_BIAS_1_9 			0xA2
#define LCD_BIAS_1_7 			0xA3
#define READ_MODIFY_WRITE 		0xE0 
#define END 					0xEE
#define RESET_DISPLAY 			0xE2
#define COMMON_OUTPUT_NORMAL 	0xC0
#define COMMON_OUTPUT_REVERSE 	0xC8
									/* The power control set value is obtained by OR'ing the values together to
									create the appropriate data value. For example:
									data = (POWER_CONTROL_SET | BOOSTER_CIRCUIT |
									VOLTAGE_REGULATOR | VOLTAGE_FOLLOWER);
									Only the bits that are desired need be OR'ed in because the initial value
									of POWER_CONTROL_SET sets them to zero. */
#define POWER_CONTROL_SET 		0x28
#define BOOSTER_CIRCUIT 		0x04
#define VOLTAGE_REGULATOR 		0x02
#define VOLTAGE_FOLLOWER 		0x01
									/* The initial value of the V5_RESISTOR_RATIO sets the Rb/Ra ratio to the
									smallest setting. The valid range of the ratio is:
									0x20 <= V5_RESISTOR_RATIO <= 0x27 */
#define V5_RESISTOR_RATIO 		0x26
									/* When the electronic volume command is input, the electronic volume register
									set command becomes enabled. Once the electronic volume mode has been set,
									no other command except for the electronic volume register command can be
									used. Once the electronic volume register set command has been used to set
									data into the register, then the electronic volume mode is released. */
#define ELECTRONIC_VOLUME_SET 	0x81
#define ELECTRONIC_VOLUME_INIT 	8

#define LCD_BUF_SIZE ((NUMBER_OF_COLUMNS*NUMBER_OF_ROWS)/8)



#define PRINTER_START	0
#define HEAD_LOAD_1PRT	3
#define	HEAD_2PRT		2
#define NEXT_STEP		1


#define PRINTER_WDT 	0x01
#define PRINTER_SENS	0x02
#define PRINTER_STB2	0x04
#define PRINTER_STB1	0x08
#define PRINTER_VOLT	0x10
#define PRINTER_MASK	(0x0F)

#define PRINTER_LATCH	0x0200

#define PRINTER_DOT		384

#define PRINTER_OFF 0

#define MAX_LINES 99999

// #define PR_BUF_SIZE (PRINTER_DOT*MAX_LINES)/8								
#define PR_BUF_SIZE 4096								



#define CLASS_NAME "gronic"
#define DISPLAY_DEVICE_NAME "backlcd"
#define PRINTER_DEVICE_NAME "thprint"




static void __exit gronic_exit(void);