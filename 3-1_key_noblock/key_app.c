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
    char *ledfilename;
    unsigned char num = 0;
    char readbuf[1];
    char value = 0;
    fd_set writefds;

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
    num = atoi(&filename[strlen(filename) - 1]);
    printf("key %d!\n", num);
    while(1)
    {
        retvalue = read(fd, readbuf, 1);
        if(retvalue < 0){
            printf("read file %s failed!\r\n", filename);
            return -1;
        }
        printf("key %d status: %s \r\n", num,(readbuf[0] == 1 ? "pushed":"released"));
        // sprintf(ledfilename, "/dev/led_dev_%d", num);
        // led_fd = open(ledfilename, O_RDWR | O_NONBLOCK);
        // if(led_fd < 0)
        // {
        //     printf("Can't open file %s\r\n", ledfilename);
        //     return -1;
        // }
        // FD_ZERO(&writefds);
        // FD_SET(led_fd, &writefds);
        // retvalue = select(led_fd + 1, NULL, &writefds, NULL, NULL);
        // switch (retvalue) 
        // {
        //     case 0: /* 超时 */ 
        //     /* 用户自定义超时处理 */
        //         printf("wirte %s time out!\n", ledfilename);
        //         break; 
        //     case -1: /* 错误 */ 
        //     /* 用户自定义错误处理 */ 
        //         printf("wirte %s error!\n", ledfilename);
        //         return -1;
        //         break; 
        //     default: 
        //         if(FD_ISSET(led_fd, &writefds)) 
        //         {
        //             retvalue = write(led_fd, &readbuf[0], 1);
        //             if(retvalue < 0){
        //                 printf("write file %s failed!\r\n", ledfilename);
        //                 return -1;
        //             }
        //         }
        // }

        // retvalue = close(led_fd);
        // if(retvalue < 0){
        //     printf("Can't close file %s\r\n", ledfilename);
        //     return -1;
        // }
    }

    /* 关闭设备 */
    retvalue = close(fd);
    if(retvalue < 0){
        printf("Can't close file %s\r\n", filename);
        return -1;
    }

    return 0;
}