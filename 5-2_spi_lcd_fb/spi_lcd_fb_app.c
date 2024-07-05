#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/fb.h>

#define FB_NAME "/dev/fb0"

int fd_fb = 0;
int g_fb_size;
unsigned char *fb = NULL;

int main(int argc, char **argv)
{
	int i, j;
	struct fb_var_screeninfo screen_info;

	if((fd_fb = open(FB_NAME, O_RDWR, 0)) < 0)
	{
		printf("Unable to open fb\n");
		return 0;
	}

	if(ioctl(fd_fb, FBIOGET_VSCREENINFO, &screen_info) < 0)
	{
		printf("\nError: ioctl get info.\n");
		return 0;
	}

	g_fb_size = screen_info.xres * screen_info.yres * screen_info.bits_per_pixel / 8;
	printf("x = %d, y = %d, bpp = %d\r\n", screen_info.xres, screen_info.yres, screen_info.bits_per_pixel);

	fb = (unsigned char *)mmap(0, g_fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_fb, 0);
	if((int)fb <= 0)
	{
		printf("\nError: failed to map framebuffer device 0 to memory.\n");
		return 0;
	}

//	memset(fb, 0x00, g_fb_size);
//	memset(fb, 0xFF, g_fb_size);

	//RGB
	//Red
	for(j = 0; j < (screen_info.yres / 3); j++)
	{
		for(i = 0; i < screen_info.xres; i += 1)
		{
			if(screen_info.bits_per_pixel == 16)
			{
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8)] = 0x00;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 1] = 0xF8;
			}
			else
			{
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8)] = 0x00;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 1] = 0x00;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 2] = 0xFF;
			}
		}
	}

	//Green
	for(j = (screen_info.yres / 3); j < (screen_info.yres * 2 / 3); j++)
	{
		for(i = 0; i < screen_info.xres; i += 1)
		{
			if(screen_info.bits_per_pixel == 16)
			{
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8)] = 0xE0;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 1] = 0x07;
			}
			else
			{
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8)] = 0x00;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 1] = 0xFF;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 2] = 0x00;
			}
		}
	}

	//Blue
	for(j = (screen_info.yres * 2 / 3); j < screen_info.yres; j++)
	{
		for(i = 0; i < screen_info.xres; i += 1)
		{
			if(screen_info.bits_per_pixel == 16)
			{
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8)] = 0x1F;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 1] = 0x00;
			}
			else
			{
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8)] = 0xFF;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 1] = 0x00;
				fb[(j * screen_info.xres + i) * (screen_info.bits_per_pixel / 8) + 2] = 0x00;
			}
		}
	}

	// ioctl(fd_fb, MXCFB_MPU_REFRESH_PANEL, NULL);

	munmap(fb, g_fb_size);
	close(fd_fb);

	return 0;
}

