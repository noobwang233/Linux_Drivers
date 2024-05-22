#include "string.h"
#include "stdio.h"
#include "unistd.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <stdio.h>
#include <sys/ioctl.h>


#define IOC_AT24C02_READ  100
#define IOC_AT24C02_WRITE 101

struct at24_buf{
    int addr;
    int len;
    char *data;
};

int main(int argc, char *argv[])
{
    int fd, retvalue;
    char *filename;
    struct at24_buf buffer;

	if ((argc != 4) && (argc != 5))
	{
		printf("Usage: %s <dev> r <addr> <read count>\n", argv[0]);
		printf("       %s <dev> w <addr> <data>\n", argv[0]);
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

    if(!strcmp((const char *)("w"), (const char *)(argv[2])))
    {
        buffer.addr = strtoul(argv[3], NULL, 0); //address
        buffer.len = strlen(argv[4]);                            //wirte count
        buffer.data = (char *)malloc(buffer.len * sizeof(char));
        memcpy(buffer.data, argv[4], buffer.len);
        retvalue = ioctl(fd, IOC_AT24C02_WRITE, &buffer);
        if(retvalue < 0)
        {
            return retvalue;
        }
        printf("wirte %s to adress %s sucessfully\n", argv[2], argv[3]);
        free(buffer.data);
    }
    else if(!strcmp((const char *)("r"), (const char *)(argv[2])))
    {
        buffer.addr = strtoul(argv[3], NULL, 0); //address
        buffer.len = strtoul(argv[4], NULL, 0); //read count
        buffer.data = (char *)malloc(atoi(argv[4]) * sizeof(char));
        retvalue = ioctl(fd, IOC_AT24C02_READ, &buffer);
        if(retvalue < 0)
        {
            return retvalue;
        }
        printf("address %s: %s \r\n", argv[3], buffer.data);
        free(buffer.data);
    }
    else 
    {
		printf("Usage: %s <dev> r <addr> <read count>\n", argv[0]);
		printf("       %s <dev> w <addr> <data>\n", argv[0]);
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