
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
 
#define LCD_RESET 	_IOW('L',0x1,int)
#define LCD_DCX 	_IOW('L',0x2,int)
#define LCD_POWER 	_IOW('L',0x3,int)
 
#define GPIO_LOW	0x0000
#define GPIO_HIGH	0x0001
 
#define LCD_HEIGHT	128
#define LCD_WIDTH	160
 
/***************************** command ***************************/
 
#define	ST7735_SleepOut			0x11	
#define	ST7735_FullColor		        0xB1	//in normal mode
#define     ST7735_8Colors			0xB2	//in idle mode
#define	ST7735_InPartialMode	        0xB3	//
#define     ST7735_INVCTR			0xB4	//display inversion control
#define	ST7735_PWCTR1			0xC0	//power control 1
#define	ST7735_PWCTR2			0xC1	//power control 2
#define	ST7735_PWCTR3			0xC2	//power control 3
#define	ST7735_PWCTR4			0xC3	//power control 4
#define	ST7735_PWCTR5			0xC4	//power control 5
#define	ST7735_VMCTR1			0xC5	//VCOM	control 1
#define ST7735_MADCTL			0x36	//Memory data access control
#define ST7735_GMCTRP1			0xE0	//Gamma '+'polarity Correction characteristics setting
#define ST7735_GMCTRN1			0xE1	//Gamma '-'polarity Correction characteristics setting
#define ST7735_COLMOD			0x3A	//interface pixel format
#define ST7735_CASET			0x2A	//column address set
#define ST7735_RASET			0x2B	//Row Address Set
#define ST7735_RAMWR			0x2C	//Memory write
#define ST7735_DISPON			0x29	//display on
 
static int spi_gpio_fd;
static int spi_fd;
static const char *device = "/dev/spidev32766.0";
static const char *spi_gpio_device = "/dev/spi_gpio";
 
static uint32_t mode=SPI_MODE_0;
static uint8_t bits = 8;
static uint32_t speed = 15000000;
static uint16_t delay=0;
 
 
static void pabort(const char *s)
{
	perror(s);
	abort();
}
 
static void transfer(int fd, uint8_t *tx, uint8_t *rx, size_t len)
{
	int ret;
 
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
 
	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}
 
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	
}
 
static void transfer_command(int fd, uint8_t command,size_t len)
{
	uint8_t value=command;
	ioctl(spi_gpio_fd, LCD_DCX, GPIO_LOW);
	transfer(fd,&value,NULL,len);	
}
 
static void transfer_command_parameter(int fd, uint8_t parameter,size_t len)
{
	uint8_t value=parameter;	
	ioctl(spi_gpio_fd, LCD_DCX, GPIO_HIGH);
	transfer(fd,&value,NULL,len);	
}
 
static void transfer_data(int fd, uint8_t *tx, uint8_t *rx, size_t len)
{
	ioctl(spi_gpio_fd, LCD_DCX, GPIO_HIGH);
	transfer(fd,tx,rx,len);	
}
 
static void set_window_regs(uint8_t x_start, uint8_t x_end, uint8_t y_start, uint8_t y_end)	//设置需要显示的窗口大小，起始位置
{	
	uint8_t command=0x00;
	uint8_t data=0x00;
 
	//x方向大小
	transfer_command(spi_fd, ST7735_CASET, 1);
	transfer_command_parameter(spi_fd, 0x00,1);
	transfer_command_parameter(spi_fd, x_start,1);
	transfer_command_parameter(spi_fd, 0x00,1);
	transfer_command_parameter(spi_fd, x_end-1,1);
 
 
	//y方向大小
	transfer_command(spi_fd, ST7735_RASET, 1);
	transfer_command_parameter(spi_fd, 0x00,1);
	transfer_command_parameter(spi_fd, y_start,1);
	transfer_command_parameter(spi_fd, 0x00,1);
	transfer_command_parameter(spi_fd, y_end-1,1);
 
}
static void clean_window(uint8_t x_start, uint8_t x_end, uint8_t y_start, uint8_t y_end, uint16_t back_color)
{
	//缓存上色
	uint8_t rgb[(x_end-x_start)*(y_end-y_start)*2];
	for(int i=0;i<(x_end-x_start)*(y_end-y_start)*2;i++){
		if( 0 == i%2){
			rgb[i]= (back_color & 0xFF00)>>8;
		}else{
			rgb[i]= back_color & 0x00FF;
		}
	}
 
	//开窗口
	set_window_regs(x_start,x_end,y_start,y_end);
 
	//缓存写入显存
	transfer_command(spi_fd, ST7735_RAMWR, 1);
 
	for(int i=0;i<(x_end-x_start);i++){
		transfer_data(spi_fd, &rgb[i*(y_end-y_start)*2], NULL, (y_end-y_start)*2);
	}	
}
static void spi_lcd_reset(unsigned int value)
{
	ioctl(spi_gpio_fd, LCD_RESET, value);	//reset芯片
}
 
static void spi_lcd_power(unsigned int value)
{
	ioctl(spi_gpio_fd, LCD_POWER, value);
}
 
 
static void spi_init(void)
{
	int ret;
	
	//set spi_mode
	ret = ioctl(spi_fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		pabort("can't set spi mode");
 
	ret = ioctl(spi_fd, SPI_IOC_RD_MODE32, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
 
	//bits per word
	ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");
 
	ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");
 
	//max speed hz
	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");
 
	ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");
 
	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
 
}
 
static void st7735_lcd_init(void)
{
	spi_lcd_reset(GPIO_LOW);
	usleep(1000);
	spi_lcd_reset(GPIO_HIGH);
	usleep(1000);
 
	transfer_command(spi_fd, ST7735_SleepOut, 1);
	usleep(100*1000);
 
	transfer_command(spi_fd, ST7735_FullColor, 1);	//选择需要调整的参数时是什么类型的
	transfer_command_parameter(spi_fd, 0x05,1);		//发送芯片参数
	transfer_command_parameter(spi_fd, 0x3c,1);
	transfer_command_parameter(spi_fd, 0x3c,1);
 
	transfer_command(spi_fd, ST7735_8Colors, 1);
	transfer_command_parameter(spi_fd, 0x05,1);
	transfer_command_parameter(spi_fd, 0x3c,1);
	transfer_command_parameter(spi_fd, 0x3c,1);
 
	transfer_command(spi_fd, ST7735_InPartialMode, 1);
	transfer_command_parameter(spi_fd, 0x05,1);
	transfer_command_parameter(spi_fd, 0x3c,1);
	transfer_command_parameter(spi_fd, 0x3c,1);
	transfer_command_parameter(spi_fd, 0x05,1);
	transfer_command_parameter(spi_fd, 0x3c,1);
	transfer_command_parameter(spi_fd, 0x3c,1);
 
	transfer_command(spi_fd, ST7735_INVCTR, 1);
	transfer_command_parameter(spi_fd, 0x03,1);
 
	transfer_command(spi_fd, ST7735_PWCTR1, 1);
	transfer_command_parameter(spi_fd, 0x28,1);
	transfer_command_parameter(spi_fd, 0x08,1);
	transfer_command_parameter(spi_fd, 0x04,1);
 
	transfer_command(spi_fd, ST7735_PWCTR2, 1);
	transfer_command_parameter(spi_fd, 0xc0,1);
 
	transfer_command(spi_fd, ST7735_PWCTR3, 1);
	transfer_command_parameter(spi_fd, 0x0d,1);
	transfer_command_parameter(spi_fd, 0x00,1);
 
 
	transfer_command(spi_fd, ST7735_PWCTR4, 1);
	transfer_command_parameter(spi_fd, 0x8d,1);
	transfer_command_parameter(spi_fd, 0x2a,1);
 
	transfer_command(spi_fd, ST7735_PWCTR5, 1);
	transfer_command_parameter(spi_fd, 0x8d,1);
	transfer_command_parameter(spi_fd, 0xee,1);
 
	transfer_command(spi_fd, ST7735_VMCTR1, 1);
	transfer_command_parameter(spi_fd, 0x1a,1);
 
	transfer_command(spi_fd, ST7735_MADCTL, 1);
	transfer_command_parameter(spi_fd, 0xa0,1);
 
	transfer_command(spi_fd, ST7735_GMCTRP1, 1);
	transfer_command_parameter(spi_fd, 0x04,1);
	transfer_command_parameter(spi_fd, 0x22,1);
	transfer_command_parameter(spi_fd, 0x07,1);
	transfer_command_parameter(spi_fd, 0x0a,1);
	transfer_command_parameter(spi_fd, 0x2e,1);
	transfer_command_parameter(spi_fd, 0x30,1);
	transfer_command_parameter(spi_fd, 0x25,1);
	transfer_command_parameter(spi_fd, 0x2a,1);
	transfer_command_parameter(spi_fd, 0x28,1);
	transfer_command_parameter(spi_fd, 0x26,1);
	transfer_command_parameter(spi_fd, 0x2e,1);
	transfer_command_parameter(spi_fd, 0x3a,1);
	transfer_command_parameter(spi_fd, 0x00,1);
	transfer_command_parameter(spi_fd, 0x01,1);
	transfer_command_parameter(spi_fd, 0x03,1);
	transfer_command_parameter(spi_fd, 0x13,1);
 
	transfer_command(spi_fd, ST7735_GMCTRN1, 1);
	transfer_command_parameter(spi_fd, 0x04,1);
	transfer_command_parameter(spi_fd, 0x16,1);	
	transfer_command_parameter(spi_fd, 0x06,1);
	transfer_command_parameter(spi_fd, 0x0d,1);
	transfer_command_parameter(spi_fd, 0x2d,1);
	transfer_command_parameter(spi_fd, 0x26,1);
	transfer_command_parameter(spi_fd, 0x23,1);
	transfer_command_parameter(spi_fd, 0x27,1);
	transfer_command_parameter(spi_fd, 0x27,1);
	transfer_command_parameter(spi_fd, 0x25,1);
	transfer_command_parameter(spi_fd, 0x2d,1);
	transfer_command_parameter(spi_fd, 0x3b,1);
	transfer_command_parameter(spi_fd, 0x00,1);
	transfer_command_parameter(spi_fd, 0x01,1);
	transfer_command_parameter(spi_fd, 0x04,1);
	transfer_command_parameter(spi_fd, 0x13,1);
 
	transfer_command(spi_fd, ST7735_COLMOD, 1);
	transfer_command_parameter(spi_fd, 0x05,1);
 
	transfer_command(spi_fd, ST7735_DISPON, 1);
 
 
}
 
int main(int argc, char *argv[])
{
	spi_gpio_fd = open(spi_gpio_device, O_RDWR);
	if (spi_gpio_fd < 0)
		pabort("can't open device");
	
	spi_fd = open(device, O_RDWR);
	if (spi_fd < 0)
		pabort("can't open device");
 
	spi_init();
	st7735_lcd_init();	//LCD芯片初始化
	clean_window(0, LCD_WIDTH, 0, LCD_HEIGHT, 0xF800);	//设置LCD屏为一种颜色-红色,使用RGB565
	spi_lcd_power(1);	//LCD屏背光设置
 
	sleep(10);
	close(spi_gpio_fd);
	close(spi_fd);
 
	return 0;
}