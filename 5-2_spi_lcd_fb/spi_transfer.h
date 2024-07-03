#ifndef _SPI_TRANSFER_H
#define _SPI_TRANSFER_H
#include <linux/spi/spi.h>
#include <linux/types.h>



void write_command(struct spi_device *spi, u8 cmd);
void write_datas(struct spi_device *spi, u8 *data,int len);
void write_data_u16(struct spi_device *spi, u16 data);
void write_data_u8(struct spi_device *spi, u8 data);



#endif