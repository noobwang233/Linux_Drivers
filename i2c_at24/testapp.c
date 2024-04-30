#include "string.h"
#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <signal.h>

int main(int argc, char *argv[])
{
    int fd, retvalue;
    char *filename;
    char *readbuf;

    filename = argv[1];

    /* 打开驱动文件 */
    fd = open(filename, O_RDWR);
    if(fd < 0)
    {
        printf("Can't open file %s\r\n", filename);
        return -1;
    }

    if(!strcmp((const char *)("w"), (const char *)(argv[2])))
    {
        if(argc != 5)
        {
            printf("Error args num, usage: %s /dev/at24xx w(r) <address> <data>\n", argv[0]);
            return -1;
        }
        retvalue = write(fd, argv[4], (strlen(argv[4]) > 8 ? 8 : strlen(argv[4])));
        if(retvalue < 0)
        {
            return retvalue;
        }
    }
    else if(!strcmp((const char *)("r"), (const char *)(argv[2])))
    {
        if(argc != 5)
        {
            printf("Error args num, usage: %s /dev/at24xx r <address> <read count>\n", argv[0]);
            return -1;
        }
        readbuf = (char *)malloc(atoi(argv[4]) * sizeof(char));
        retvalue = read(fd, readbuf, atoi(argv[4]));
        if(retvalue < 0)
        {
            return retvalue;
        }
        printf("address %s: %s \n", argv[3], readbuf);
        free(readbuf);
    }
    else 
    {
        return -1;
    }

    /* 关闭设备 */
    retvalue = close(fd);
    if(retvalue < 0){
        printf("Can't close file %s\r\n", filename);
        return -1;
    }

    return 0;
}