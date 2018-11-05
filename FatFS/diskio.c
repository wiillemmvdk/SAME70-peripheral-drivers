/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "../HSMCI.h"	/* SD routine functions */

#define DN_MCI		0	/* Physical drive number for MCI */

/* \brief	initialize the disk
 * \param	BYTE pdrv	Physical drive nmuber to identify the drive 
*/
DSTATUS disk_initialize ( BYTE pdrv )
{
	if (pdrv == DN_MCI) {
		return SD_CardInit();
	} else {
		return RES_PARERR;
	}
}



/* \brief get the Disk status
 * \param	BYTE pdrv	Physical drive nmuber to identify the drive 
 */
DSTATUS disk_status ( BYTE pdrv )
{
	if (pdrv == DN_MCI) {
		return SD_getStatus();
	} else {
		return RES_PARERR;
	}	
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	if (pdrv == DN_MCI) {
		return SD_readBlocks(buff, count, sector);
	} else {
		return RES_PARERR;
	}
	
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive number to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	if (pdrv == DN_MCI) {
		return SD_writeBlocks(buff, count, sector);
	} else {
		return RES_PARERR;
	}
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	if (pdrv == DN_MCI) {
		return SD_IOctl(cmd, buff);		
	} else {
		return RES_PARERR;
	}
}

