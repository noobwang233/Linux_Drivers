#include "stdio.h"  //printf
//close read write 使用
#include "unistd.h"
#include "string.h" //字符串比较使用
//open使用
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


int main(int argc, char *argv[])
{
    int fd, retvalue, i;
    char status;

	/* 1. 判断参数 */
    if(((0 == strcmp(argv[2], "read")) && argc != 3) || ((0 == strcmp(argv[2], "write")) && argc != 4))
    {
        printf("Usage: %s <test_dev> <read | write> [0 | 1]\n", argv[0]);
        return -1;
    }

	/* 2. 打开文件 */
	fd = open(argv[1], O_RDWR);
	if (fd == -1)
	{
		printf("Can't open file %s\n", argv[1]);
		return -1;
	}

    /* 3. 判断读写 */
    if (0 == strcmp(argv[2], "read"))
    {
        /* 4.读文件 */
        read(fd,&status,1);
        printf("read dev data %d", status);
    }
    else
    {
        /* 5. 写文件 */
        if (0 == strcmp(argv[2], "1"))
        {
            status = 1;
            write(fd, &status, 1);
        }
        else if (0 == strcmp(argv[2], "0"))
        {
            status = 0;
            write(fd, &status, 1);
        }
        else
        {
            printf("Usage: %s <test_dev> <read | write> [0 | 1]\n", argv[0]);
            return -1;
        }
    }

    /* 关闭设备 */
    retvalue = close(fd);
    if(retvalue < 0){
        printf("Can't close file %s\r\n", argv[1]);
        return -1;
    }

    return 0;
}