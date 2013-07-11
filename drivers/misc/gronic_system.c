#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/posix-timers.h>
#include <linux/kthread.h>
#include <mach/platform.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kfifo.h>

#include "gronic_system.h"


#define MS_TO_NS(x)	(x * 1E6L)

//*********** SPI 
__u32 e_port;

//************ Keypad
// struct gronic_keypad_data *key_data = NULL;
static struct input_dev *keypad_dev;

//*************LCD
char*  lcd_buf;
static struct device* display_device = NULL;
static int display_major; 

//******************Printer
__u8 step=0;
__u8 mcp3_A_reg=0x60;		  							
__u8 printer_status=PRINTER_START;
__u8 printer_paper;
__u32 lines_to_print=0;
static int printer_major; 

static DECLARE_KFIFO(printer_fifo, char,PR_BUF_SIZE);


//********** General 
struct  task_struct  *task;
static struct class* device_class = NULL;
static struct hrtimer hr_timer;

MODULE_AUTHOR("Gronic Systems GmbH");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("System' driver for itegrated peripherals");


void config_pin_mode(__u32  port, __u32  pin, __u32 mode){
	__u32 reg_val = 0;

	reg_val = (PIO_REG_CFG_VALUE(port,pin/8)) & ~ (0x07<<((pin & 0x07)<<2));
	reg_val =  reg_val | (mode<<((pin & 0x07)<<2));
	*PIO_REG_CFG(port,pin/8) = reg_val;
}

void config_pin_pull(__u32  port, __u32  pin, __u32 mode){
	__u32 reg_val = 0;

	reg_val = (PIO_REG_PULL_VALUE(port,pin/16)) & ~ (0x03<<((pin & 0x0F)<<1));
	reg_val =  reg_val | (mode<<((pin & 0x0F)<<1));

	*PIO_REG_PULL(port,0) = reg_val;
}

void set_pin_value(__u32 port, __u32 pin, __u32 value){
	__u32 reg_val = PIO_REG_DATA_VALUE(port);
	if(value)
		reg_val =  reg_val | (0x01 << pin) ; 
	else 
		reg_val = reg_val & ~(0x01 << pin); 

	*PIO_REG_DATA(port) = reg_val;
}

char  get_pin_value(__u32 port,  __u32 pin){
	__u32 pin_msk = 0x01 << pin;
	if ((PIO_REG_DATA_VALUE(port) &  pin_msk)>0 )
		return 0;
	else 
		return 1;
}

struct gronic_keypad_data {
	char  key;
};

void init_gpio(void){
	//led on 
	config_pin_mode(PIO_G,9,PIN_MODE_OUTPUT);

	//config spi
	config_pin_mode(PIO_E, SPI_MOSI, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_MISO, PIN_MODE_INPUT);
	config_pin_mode(PIO_E, SPI_CS0, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_SCLK, PIN_MODE_OUTPUT);


	//config printer
	config_pin_mode(PIO_E, PH_1A, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, PH_1B, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, PH_2A, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, PH_2B, PIN_MODE_OUTPUT);


	//Interupt key press
	config_pin_pull(PIO_B, KEYPAD_IRQ_PIN, PULL_MODE_UP);
	// config_pin_mode(PIO_B, KEYPAD_IRQ_PIN, PIN_MODE_INT);
	config_pin_mode(PIO_B, KEYPAD_IRQ_PIN, PIN_MODE_INPUT);


	config_pin_pull(PIO_E, 3, PULL_MODE_UP);
	config_pin_mode(PIO_E, 10, PIN_MODE_INPUT);
	config_pin_pull(PIO_E, 10, PULL_MODE_DOWN);

	//Init Select 
	config_pin_mode(PIO_E, SPI_SEL_0, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_SEL_1, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_B, PER_RST, PIN_MODE_OUTPUT);


	set_pin_value(PIO_B,PER_RST,1);
	set_pin_value(PIO_B,PER_RST,0);
	set_pin_value(PIO_B,PER_RST,1);
}



const ADR_REG KEYB_MCP[]={ 
   {0,0},
   { GPIO_A | MCP_IOCON,  IOCON_ODR }, // control     Bank, ODR
   { GPIO_B | MCP_IOCON,  IOCON_ODR }, // control     Bank, ODR
   { GPIO_A | MCP_IODIR, 0xF0 }, // GPIO-A Direction     output 4 bit Row
   { GPIO_B | MCP_IODIR, 0x1F}, // GPIO-B Direction     input 5 bit Column
   { GPIO_A | MCP_GPPU, 0x0F }, // GPIO-A Pull Up Res 
   { GPIO_B | MCP_GPPU, 0x1F }, // GPIO-B Pull Up Res    Res Column on
   { GPIO_A | MCP_GPINTEN, 0x00 }, // GPIO-A Interrupt on changed
   { GPIO_B | MCP_GPINTEN, 0x1F }, // GPIO-B Interrupt on changed  Interr. Column on
   { GPIO_A | MCP_OLAT, 0x00}, // GPIO-A default output level 
   { GPIO_A | MCP_IPOL, 0x00 }, 
   { GPIO_B | MCP_IPOL, 0x00 }, 
   {0xFF,0}
};


const ADR_REG CONTR_MCP[]={
   {3,0},
   { GPIO_A | MCP_IOCON,  IOCON_ODR }, // control     Bank, ODR
   { GPIO_B | MCP_IOCON,  IOCON_ODR }, // control     Bank, ODR
   { GPIO_A | MCP_IODIR, 0x02 }, // GPIO-A Direction     output Bit1 input
   { GPIO_B | MCP_IODIR, 0x00 }, // GPIO-B Direction     output LCD DB8..0
   { GPIO_A | MCP_GPPU, 0x00 }, // GPIO-A Pull Up Res 
   { GPIO_B | MCP_GPPU, 0xFF }, // GPIO-B Pull Up Res    Res DB on
   { GPIO_A | MCP_GPINTEN, 0x00 }, // GPIO-A Interrupt on changed
   { GPIO_B | MCP_GPINTEN, 0x00 }, // GPIO-B Interrupt on changed   
   { GPIO_A | MCP_OLAT, 0x60  }, // GPIO-A default output level 
   { GPIO_B | MCP_OLAT, 0xFF }, // GPIO-B default output level
   { GPIO_A | MCP_IPOL, 0x00 },
   { GPIO_B | MCP_IPOL, 0x00 },
   {0xFF,0   }
};

void spi_thermal_config( void){

	*SPI_A13_REG(SPI_CTL)	= 0x000A0013;		// DEL,PHA,MODE,ENABLE
	*SPI_A13_REG(SPI_WAIT)	= 0x00000080;		// WAIT Timer
	*SPI_A13_REG(SPI_CCTL)	= 0x00000010;		// Clock Freq.
}

u32 SPI_MCP(__u8 mcp_chip, __u32 sdata){

	 u32 smask = (1<<23);
	 u32 rdat  = (0x00000000);

	 e_port = (PIO_REG_DATA_VALUE(PIO_E) & 0x000000F0) | (mcp_chip << 8)  ; // select MCP-Chip  CS low
	 *PIO_REG_DATA(PIO_E) = e_port;

	  do{
		if((sdata & smask) == 0)e_port = e_port & ~BB_SPI2_MOSI;   // clk low
		else     e_port = e_port |  BB_SPI2_MOSI;
		*PIO_REG_DATA( PIO_E) = e_port;          // MOSI bit out
		*PIO_REG_DATA( PIO_E) = e_port | BB_SPI2_CLK;      // clk high
		if(PIO_REG_DATA_VALUE(PIO_E) & BB_SPI2_MISO) rdat = rdat | smask;
		smask = smask >> 1;
	 }while(smask);
	 
	 e_port = e_port | BB_SPI2_CS0;  
	 *PIO_REG_DATA( PIO_E) = e_port;           // CS high
	 return(rdat);
}


void mcp23_init( const ADR_REG *chip ){
	u8 addr = chip->mcpreg;
	chip++;
	while(chip->mcpreg != 0xFF){
		SPI_MCP(addr, MCP_WRITE | (chip->mcpreg<<8) | chip->mcpvalu);
		chip++;
	}
}

void disp_data(__u8 lcd_dat){
	mcp3_A_reg |= 0x80;         // A0= high 
	SPI_MCP(3, MCP_WRITE | (( GPIO_B | MCP_OLAT) << 8) | lcd_dat);
	SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (mcp3_A_reg & ~0x20) );  // CS= low
	SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (mcp3_A_reg | 0x20) );  // CS= high
}

void disp_cmd(__u8 lcd_cmd){
	mcp3_A_reg &= ~0x80;         // A0= low 
	SPI_MCP(3, MCP_WRITE | (( GPIO_B | MCP_OLAT) << 8) | lcd_cmd);
	SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (mcp3_A_reg & ~0x20) );  // CS= low
	SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (mcp3_A_reg | 0x20) );  // CS= high
}

__u8 LCD_image(char * image){
	__u8 page,i,j,v,mask_v,mask_h;
	__u8 im_buf[128];

	for(page=0; page<8; page++){
		memset(im_buf,0,128);
		mask_v=0x01;
		for(v=0;v<8;v++){
			  for(j=0;j<16;j++){                                                                                                          // von horizontal pixel nach vertikal pixel wandeln
				mask_h=0x80;
				for(i=0;i<8;i++){
					if(*image & mask_h) im_buf[(j*8)+i] |= mask_v;
					mask_h=mask_h>>1;
				}
				image++;
			   }
			   mask_v = mask_v<<1;
		}

		disp_cmd(START_LINE_SET);
		disp_cmd(PAGE_ADDRESS_SET + page);
		disp_cmd(COLUMN_ADDRESS_HIGH);
		disp_cmd(COLUMN_ADDRESS_LOW);
		for(i=0;i<NUMBER_OF_COLUMNS;i++){                                                // 0..127
			disp_data(im_buf[i]);
		}
	}
	return(0);
}
	
/******************************************************************************
This performs all of the necessary initialization of the SED-1565 controller
Init_Display();
******************************************************************************/
u8 LCD_vol=8;
#define  LCDVOL_MAX 25


int display_open(struct inode *inode, struct file *filp) {

  /* Success */
	return 0;
}

int display_release(struct inode *inode, struct file *filp) {
 
  /* Success */
	return 0;
}


ssize_t display_write( struct file *filp, char *buf, size_t count, loff_t *f_pos) {
	char *tmp;
	int bytes_read;
	tmp=buf+count-1;

	if(count>LCD_BUF_SIZE) count = LCD_BUF_SIZE;
	
	bytes_read = copy_from_user(lcd_buf,buf,count);

	printk("%d ---- %s ", bytes_read, count);

	LCD_image(lcd_buf);
	return count;
}

struct file_operations lcd_fops = {
	write : display_write,
	open: display_open,
	release: display_release
};

int register_lcd(void){
	int result;

	 display_major = register_chrdev(0, DISPLAY_DEVICE_NAME, &lcd_fops);
		if (result < 0) {
				printk("memory: cannot obtain major number %d\n", display_major);
			return result;
	}



	display_device = device_create(device_class, NULL, MKDEV(display_major, 0), NULL, DISPLAY_DEVICE_NAME);
	if (IS_ERR(display_device)) {
		printk("failed to create device '%s_%s'\n", CLASS_NAME, DISPLAY_DEVICE_NAME);
		result = PTR_ERR(display_device);
		goto fail;
	}


	lcd_buf = kmalloc(LCD_BUF_SIZE, GFP_KERNEL); 
	if (!lcd_buf) { 
			result = -ENOMEM;
			goto fail; 
	} 
	memset(lcd_buf,0,LCD_BUF_SIZE);

	LCD_image((char*) lcd_buf);

	printk("Gronic registering Backlcd : %d \n", display_major); 
	return 0;

  fail:
		gronic_exit(); 
		return result;
}


void Init_LCD(void){
	disp_cmd(0);
	disp_cmd(RESET_DISPLAY);
	disp_cmd(LCD_BIAS_1_9);
	disp_cmd(ADC_SELECT_NORMAL);
	disp_cmd(COMMON_OUTPUT_REVERSE);
	disp_cmd(V5_RESISTOR_RATIO);
	disp_cmd(ELECTRONIC_VOLUME_SET);
	disp_cmd(LCD_vol);
	disp_cmd(START_LINE_SET);
	disp_cmd((POWER_CONTROL_SET | VOLTAGE_REGULATOR |	VOLTAGE_FOLLOWER | BOOSTER_CIRCUIT));

	disp_cmd(DISPLAY_NORMAL);
	disp_cmd(DISPLAY_ON);
}


void key_press_handler(void){
	__u32  key;
	static __u32 last_key = 0;
	__u8 col_reg, row_reg;

	col_reg = SPI_MCP(0, MCP_READ | (( GPIO_B | MCP_INTCAP) << 8) | 0) & 0x1F;

	SPI_MCP(0, MCP_WRITE | (( GPIO_A | MCP_IODIR) << 8) | 0xFF);
	SPI_MCP(0, MCP_WRITE | (( GPIO_B | MCP_IODIR) << 8) | 0xE0);
	SPI_MCP(0, MCP_WRITE | (( GPIO_B | MCP_OLAT) << 8) | 0x00);

	row_reg = SPI_MCP(0, MCP_READ | (( GPIO_A | MCP_GPIO) << 8) | 0) & 0x0F;
	
	SPI_MCP(0, MCP_WRITE | (( GPIO_A | MCP_IODIR) << 8) | 0xF0);
	SPI_MCP(0, MCP_WRITE | (( GPIO_B | MCP_IODIR) << 8) | 0x1F);
	SPI_MCP(0, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | 0x00);

	SPI_MCP(0, MCP_READ | (( GPIO_B | MCP_INTCAP) << 8) | 0) & 0x1F;

	switch((__u32)(col_reg<<4)|row_reg){
		case 0x0FE : key = KEY_UP ;break;
		case 0x0FD : key = KEY_F2 ;break;
		case 0x0FB : key = KEY_DOWN ;break;
		case 0x17E : key = KEY_1 ;break;
		case 0x17D : key = KEY_2 ;break;
		case 0x17B : key = KEY_3 ;break;
		case 0x1BE : key = KEY_4 ;break;
		case 0x1BD : key = KEY_5 ;break;
		case 0x1BB : key = KEY_6 ;break;
		case 0x1DE : key = KEY_7 ;break;
		case 0x1DD : key = KEY_8 ;break;
		case 0x1DB : key = KEY_9 ;break;
		case 0x1ED : key = KEY_0 ;break;
		case 0x1EE : key = KEY_DOT ;break;
		case 0x1EB : key = KEY_QUESTION;break;
		case 0x177 : key = KEY_A;break;
		case 0x1B7 :  key  = KEY_BACKSPACE ;break;
		case 0x1D7 :  key  = KEY_ENTER;break;
		case 0x1FF :   key   = 0 ;break;
		default : key = 0; break;
	}

	if(key){
		lines_to_print = 240;
		last_key = key;
		input_report_key(keypad_dev, key,true);
	}
	else{
		input_report_key(keypad_dev,last_key,false);
	}
	input_sync(keypad_dev);
}



int init_keypad(void){
	int error;

	keypad_dev = input_allocate_device();
	if (!keypad_dev) {
		printk(KERN_ERR "Gronic : Not enough memory\n");
		error = -ENOMEM;
	}

	keypad_dev->name = "Gronic Keypad";
	keypad_dev->evbit[0] = BIT_MASK(EV_KEY);
		
	keypad_dev->keybit[BIT_WORD(KEY_1 )]= BIT_MASK(KEY_1 )  |  BIT_MASK(KEY_2 ) | BIT_MASK(KEY_3 )| BIT_MASK(KEY_4 )| BIT_MASK(KEY_5 )| BIT_MASK(KEY_6 )| 
					BIT_MASK(KEY_7 )| BIT_MASK(KEY_8 )| BIT_MASK(KEY_9 )| BIT_MASK(KEY_0)| BIT_MASK(KEY_BACKSPACE ); 
	keypad_dev->keybit[BIT_WORD(KEY_DOT )] |= BIT_MASK(KEY_DOT );
	keypad_dev->keybit[BIT_WORD(KEY_QUESTION)]|= BIT_MASK(KEY_QUESTION);
	keypad_dev->keybit[BIT_WORD(KEY_A )]|= BIT_MASK(KEY_A );
	keypad_dev->keybit[BIT_WORD(KEY_BACKSPACE)]|= BIT_MASK(KEY_BACKSPACE );
	keypad_dev->keybit[BIT_WORD(KEY_ENTER )]|= BIT_MASK(KEY_ENTER);
	keypad_dev->keybit[BIT_WORD(KEY_UP )] |= BIT_MASK(KEY_UP) | BIT_MASK(KEY_DOWN) ;
	keypad_dev->keybit[BIT_WORD(KEY_F2)]  |= BIT_MASK(KEY_F2);


	error = input_register_device(keypad_dev);
	if (error) {
		printk(KERN_ERR "Gronic : Failed to register device\n");
		goto err_free_dev;
	}

	return 0;

 err_free_dev:
	input_free_device(keypad_dev);
	return error;
}

void step_in(void){				   						// stepper +1
	switch(step & 0xF0){
		case 0xA0:	step = 0x90;	break;	// 1
		case 0x90:	step = 0x50; 	break; 	// 2
		case 0x50:	step = 0x60; 	break; 	// 3
		case 0x60:	step = 0xA0; 	break; 	// 4
		default:    step = 0x90;	break;	// 1
	}

	 *PIO_REG_DATA(PIO_E) = (e_port & 0xF) | step;
}

void step_out(void){	   								// stepper -1
	switch(step & 0xF0){
		case 0xA0:	step = 0x60;	break;	// 4
		case 0x60:	step = 0x50; 	break; 	// 3
		case 0x50:	step = 0x90; 	break; 	// 2
		case 0x90:	step = 0xA0; 	break; 	// 1
		default:    step = 0x60;	break;	// 4
	}
	 *PIO_REG_DATA(PIO_E) = (e_port & 0xF) | step;
}


void head_out(void){					   			// Thermal Head out
__u8 hmask;
__u8 n=PRINTER_DOT/8;
__u8 out;

__u8 line_buf[n];
__u8 *out_ptr = &line_buf; 


	if(kfifo_peek_len(&printer_fifo) > n)
		kfifo_out(&printer_fifo,line_buf,n);
	else{
		kfifo_reset(&printer_fifo);
		return;
	}


	e_port = PRINTER_LATCH | step | BB_SPI2_CS0;
	 *PIO_REG_DATA(PIO_E) = e_port;

	while(n--){
		hmask = 0x80;
		out = *out_ptr++;
		do{
			if((out & hmask) == 0)
				e_port = e_port & ~BB_SPI2_MOSI;
			else					 
				e_port = e_port |  BB_SPI2_MOSI;
			*PIO_REG_DATA(PIO_E) = e_port;
			*PIO_REG_DATA(PIO_E) = e_port | BB_SPI2_CLK ;
			hmask = hmask >> 1;
	 		*PIO_REG_DATA(PIO_E) = e_port ;
		}while(hmask);
	}

	 *PIO_REG_DATA(PIO_E) = e_port ;
	e_port = (e_port & ~BB_SPI2_CS0) | PRINTER_LATCH;					        // Printer Latch low
	 *PIO_REG_DATA(PIO_E) = e_port ;
	 *PIO_REG_DATA(PIO_E) = e_port ;
	e_port = e_port | BB_SPI2_CS0;											// Printer Latch high
	 *PIO_REG_DATA(PIO_E) = e_port;
}



__u8 th_printer(void){				  					// Timer 

	if(lines_to_print==0){
		if(printer_status){
			mcp3_A_reg &= ~PRINTER_MASK;
			printer_paper=SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | mcp3_A_reg );	// Printer off
			printer_status=PRINTER_OFF;
			e_port &= 0x0F;		
			 *PIO_REG_DATA(PIO_E) = e_port;
		}
		return 0;
	}

	switch(printer_status){

		case PRINTER_START:
			mcp3_A_reg = mcp3_A_reg | PRINTER_VOLT | PRINTER_WDT;
			SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | mcp3_A_reg );	// Printer on
			mcp3_A_reg = mcp3_A_reg & ~PRINTER_WDT;
			printer_paper=SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | mcp3_A_reg );	// Printer Sensor
			e_port = e_port | step;	
			 *PIO_REG_DATA(PIO_E) = e_port; 	// Stepper Motor on
			printer_status=HEAD_LOAD_1PRT;
		break;

		case NEXT_STEP:
			step_out();
			lines_to_print--;
			printer_status = HEAD_LOAD_1PRT;
		// break;

		case HEAD_LOAD_1PRT:
			head_out();
			mcp3_A_reg = ( mcp3_A_reg | PRINTER_VOLT | PRINTER_WDT | PRINTER_STB1 ) & ~( PRINTER_STB2 ) ;
			printer_paper=SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | mcp3_A_reg );	// Printer Sensor

			printer_status--;
		break;

		case HEAD_2PRT:
			mcp3_A_reg = ( mcp3_A_reg | PRINTER_VOLT  | PRINTER_STB2 ) & ~PRINTER_STB1;
			printer_paper=SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | mcp3_A_reg );	// Printer Sensor
			step_out();

			printer_status--;
		break;

		

		default:
			printer_status--;
		break;

	}
	return 1;
}


int printer_open(struct inode *inode, struct file *filp) {
  /* Success */
	return 0;
}

int printer_release(struct inode *inode, struct file *filp) {
  /* Success */
	return 0;
}

ssize_t printer_write( struct file *filp, char *buf, size_t count, loff_t *f_pos) {
	__u32 bytes_to_read;
	__u32 copied;

	if(count>PR_IN_SIZE)
		bytes_to_read = PR_IN_SIZE;
	else 
		bytes_to_read = count;

	copied = kfifo_in(&printer_fifo, buf,bytes_to_read);

	lines_to_print += (copied * 8) /PRINTER_DOT;
	printk("%d lines to print xount : %d \n",lines_to_print,count) ;
	return copied;
}

struct file_operations pr_fops = {
	write : printer_write,
	open: printer_open,
	release: printer_release
};

int register_printer(void){
	int result;

	 INIT_KFIFO(printer_fifo);


	 printer_major = register_chrdev(0, PRINTER_DEVICE_NAME, &pr_fops);
		if (result < 0) {
			printk("memory: cannot obtain major number %d\n", printer_major);
			return result;
	}

	display_device = device_create(device_class, NULL, MKDEV(printer_major, 0), NULL, PRINTER_DEVICE_NAME);
	if (IS_ERR(display_device)) {
		printk("failed to create device '%s_%s'\n", CLASS_NAME, PRINTER_DEVICE_NAME);
		result = PTR_ERR(display_device);
		goto fail;
	}

	printk("Gronic registering Printer : %d \n", printer_major); 
	return 0;

  fail:
	gronic_exit(); 
	return result;
}


__u8 cnt;
unsigned long timer_interval_ns = 250e6;

enum hrtimer_restart gronic_timer_callback( struct hrtimer *timer_for_restart )
{
	__u32 reg_val;
  	ktime_t currtime , interval;
	
	set_pin_value(PIO_G,9,(cnt++ & 1)); 

	reg_val = PIO_REG_DATA_VALUE(PIO_B) ;
	if((reg_val & (1<<KEYPAD_IRQ_PIN)) ==0) { //Key Pressed
		key_press_handler();
	}
	
	
	if(th_printer() > 0){
		timer_interval_ns = 450e3;
	}
	else{
		timer_interval_ns = 250e6;
	}

	currtime  = ktime_get();
  	interval = ktime_set(0,timer_interval_ns); 
  	hrtimer_forward(timer_for_restart, currtime , interval);

	return HRTIMER_RESTART;
}



static int __init gronic_init(void) {
	ktime_t ktime;

	pr_info("Gronic system driver init\n");

	init_gpio();
	init_keypad();
 	mcp3_A_reg = 0x60;

	mcp23_init( KEYB_MCP );						// Tastatur initialisieren
	mcp23_init( CONTR_MCP );					// Drucker initialisieren
	


	device_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(device_class)) {
		printk("failed to register device class '%s'\n", CLASS_NAME);
		goto fail;
	}

	register_lcd();
	register_printer();
	Init_LCD();

	ktime = ktime_set( 0, timer_interval_ns );
	hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	hr_timer.function = &gronic_timer_callback;
 	hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL );

	return 0;

fail : 
	printk("Fail registering device class");
	return 1; 
}


static void __exit gronic_exit(void) {

	int ret;

	//Stop timer
  	ret = hrtimer_cancel( &hr_timer );
  	if (ret){
  		udelay(50);
  		printk("The timer was still in use...\n");
  	}
  		
  	//Remove LCD 

  	// device_remove_file(parrot_device, &dev_attr_fifo);
	// device_remove_file(parrot_device, &dev_attr_reset);
	device_destroy(device_class, MKDEV(display_major, 0));
	unregister_chrdev(display_major, DISPLAY_DEVICE_NAME);

	memset(lcd_buf,0,LCD_BUF_SIZE);
	LCD_image((char*) lcd_buf);

	 if (lcd_buf) {
	     	kfree(lcd_buf);
	 }
	
	//Remove Printer

	device_destroy(device_class, MKDEV(printer_major, 0));
	unregister_chrdev(printer_major, DISPLAY_DEVICE_NAME);

	class_unregister(device_class);								
	class_destroy(device_class);	

	//Remove Keypad
	input_unregister_device(keypad_dev);

	// exit();
	pr_info("Gronic system driver exit \n");
}

module_init(gronic_init);
module_exit(gronic_exit);