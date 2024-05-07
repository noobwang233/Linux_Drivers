#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"

static char usrdata[] = {"usr data!"};

int main(int argc, char *argv[])
{
    int fd, retvalue, i;
    char status;

	/* 1. 判断参数 */
    if(argc != 3)
    {
        printf("Usage: %s <led_dev> <on | off>\n", argv[0]);
        return -1;
    }

	/* 2. 打开文件 */
	fd = open(argv[1], O_RDWR);
	if (fd == -1)
	{
		printf("Can't open file %s\n", argv[1]);
		return -1;
	}

	/* 3. 写文件 */
	if (0 == strcmp(argv[2], "off"))
	{
		status = 1;
		write(fd, &status, 1);
	}
	else if (0 == strcmp(argv[2], "on"))
	{
		status = 0;
		write(fd, &status, 1);
	}
    else
    {
		printf("Error cmd! Usage: %s <dev> <on | off> \n", argv[1]);
		return -1;
    }
//占用设备30s
	for(i = 0; i < 30; i++)
	{
		sleep(5);
		printf("delay %d s\r\n", i);
	}
    /* 关闭设备 */
    retvalue = close(fd);
    if(retvalue < 0){
        printf("Can't close file %s\r\n", argv[1]);
        return -1;
    }

    return 0;
}
