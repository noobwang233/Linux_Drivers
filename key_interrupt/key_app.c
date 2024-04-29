#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"

int main(int argc, char *argv[])
{
    int fd, retvalue;
    int led_fd;
    char *filename;
    unsigned char cnt = 0;
    char readbuf[1];
    char value = 0;

    if(argc != 2)
    {
        printf("Error Usage!\r\n");
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
    
    while(1)
    {
        retvalue = read(fd, readbuf, 1);
        if(retvalue < 0){
            printf("read file %s failed!\r\n", filename);
            return -1;
        }
        printf("key status: %s \r\n", (readbuf[0] == 1 ? "pushed":"released"));
        led_fd = open("/dev/led_dev_0", O_RDWR);
        if(led_fd < 0)
        {
            printf("Can't open file /dev/led_dev_0\r\n");
            return -1;
        }
        retvalue = write(led_fd, &readbuf[0], 1);
        if(retvalue < 0){
            printf("write file /dev/led_dev_0 failed!\r\n");
            return -1;
        }
        retvalue = close(led_fd);
        if(retvalue < 0){
            printf("Can't close file /dev/led_dev_0\r\n");
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