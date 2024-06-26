#include "string.h"
#include "stdio.h"
#include "unistd.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include "spi_lcd_drv.h"


int main(int argc, char *argv[])
{
    int fd, retvalue;
    char *filename;
    struct oled_buf buffer;

    if ((argc < 6) && (argc > 8))
    {
        printf("Usage: %s %s <cmd> <x1> <y1> [<x2> <y2>] <wite/black>\n", argv[0], argv[1]);
        printf("cmd list:\n draw_point\n draw_rect\n draw_rect_filled\n draw_line\n");
        return -1;
    }

    filename = argv[1];

    /* 打开驱动文件 */
    fd = open(filename, O_RDWR);
    if(fd < 0)
    {
        printf("Can't open file %s\r\n", filename);
        return -1;
    }

    if(!strcmp((const char *)("draw_point"), (const char *)(argv[2])))
    {
        if (argc != 6)
        {
            printf("Usage: %s %s draw_point <x1> <y1> <wite/black>\n", argv[0], argv[1]);
            return -1;
        }
        buffer.x1 = atoi(argv[3]);
        buffer.y1 = atoi(argv[4]);
        if(!strcmp((const char *)("wite"), (const char *)(argv[7])))
        {
            buffer.color = OLED_COLOR_NORMAL;
        }
        else if(!strcmp((const char *)("black"), (const char *)(argv[7])))
        {
            buffer.color = OLED_COLOR_REVERSED;
        }
        else 
        {
            printf("Usage: %s %s draw_point <x1> <y1> <wite/black>\n", argv[0], argv[1]);
            return -1;
        }
        retvalue = ioctl(fd, IOC_OLED_DRAW_POINT, &buffer);
        if(retvalue < 0)
        {
            printf("draw failed\n");
            return retvalue;
        }
        printf("draw sucessfully\n");
    }
    else
    {
        if (argc != 8)
        {
            printf("Usage: %s %s <cmd> <x1> <y1> <x2> <y2> <wite/black>\n", argv[0], argv[1]);
            return -1;
        }
        buffer.x1 = atoi(argv[3]);
        buffer.y1 = atoi(argv[4]);
        buffer.x2 = atoi(argv[5]);
        buffer.y2 = atoi(argv[6]);
        if(!strcmp((const char *)("wite"), (const char *)(argv[7])))
        {
            buffer.color = OLED_COLOR_NORMAL;
        }
        else if(!strcmp((const char *)("black"), (const char *)(argv[7])))
        {
            buffer.color = OLED_COLOR_REVERSED;
        }
        else 
        {
            printf("Usage: %s %s <cmd> <x1> <y1> <x2> <y2> <wite/black>\n", argv[0], argv[1]);
            return -1;
        }
        if(!strcmp((const char *)("draw_line"), (const char *)(argv[2])))
        {
            retvalue = ioctl(fd, IOC_OLED_DRAW_LINE, &buffer);
        }
        else if(!strcmp((const char *)("draw_rect"), (const char *)(argv[2])))
        {
            retvalue = ioctl(fd, IOC_OLED_DRAW_RECTANGLE, &buffer);
        }
        else if(!strcmp((const char *)("draw_rect_filled"), (const char *)(argv[2])))
        {
            retvalue = ioctl(fd, IOC_OLED_DRAW_RECTANGLE_FILLED, &buffer);
        }
        else
        {
            printf("Usage: %s %s <cmd> <x1> <y1> <x2> <y2> <wite/black>\n", argv[0], argv[1]);
            return -1;
        }
    }

    /* 关闭设备 */
    retvalue = close(fd);
    if(retvalue < 0){
        printf("Can't close file %s\r\n", filename);
        return -1;
    }

    return 0;
}