#define KEYPAD_IRQ_NO      	(24)

#define __REG(x)                        (*(volatile unsigned int *)(x))
#define PIOC_REGS_BASE SW_VA_PORTC_IO_BASE
#define PIO_B 	2
#define PIO_C	3
#define PIO_D 	4
#define PIO_E 	5
#define PIO_F 	6
#define PIO_G 	7

#define SPI_CS0 	0
#define SPI_SCLK 	1
#define SPI_MOSI 	2
#define SPI_MISO 	3

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
#define MCP_IPOL	0x01
#define MCP_GPINTEN	0x02
#define MCP_DEFVAL	0x03
#define MCP_INTCON	0x04
#define MCP_IOCON	0x05
#define IOCON_BANK	(1 << 7)
#define IOCON_SEQOP	(1 << 5)
#define IOCON_HAEN	(1 << 3)
#define IOCON_ODR	(1 << 2)
#define IOCON_INTPOL	(1 << 1)

#define MCP_GPPU	0x06
#define MCP_INTF	0x07
#define MCP_INTCAP	0x08
#define MCP_GPIO	0x09
#define MCP_OLAT	0x0a

#define GPIO_A 		0x00
#define GPIO_B 		0x10



#define BB_SPI2_CS0			(0x00000001)
#define BB_SPI2_CLK			(0x00000002)
#define BB_SPI2_MOSI		(0x00000004)
#define BB_SPI2_MISO		(0x00000008)
#define SPISEL0				(0x00000100)
#define SPISEL1				(0x00000101)



typedef struct {
	u8	mcpreg;
	u8	mcpvalu;	
} ADR_REG;


      


