/*
 * fat.c
 *
 *  Created on: Dec 19, 2013
 *      Author: Jed Frey
 */

/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "shell.h"

#include "fat.h"
#include "ff.h"
#include "fattime.h"

/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
FATFS MMC_FS;

/**
 * MMC driver instance.
 */
MMCDriver MMCD1;

/* FS mounted and ready.*/
static bool fs_ready = FALSE;

/* Maximum speed SPI configuration (24MHz, CPHA=0, CPOL=0, MSb first).*/
static const SPIConfig hs_spicfg =
{ 
  NULL, 
  SPI_SD_PORT, SPI_SD_MASK, SPI_SD_SEL, SPI_SD_DIS,
  SPI_CTAR_8_FAST,
};

/* Low speed SPI configuration (375kHz, CPHA=0, CPOL=0, MSb first).*/
static const SPIConfig ls_spicfg = 
{ 
  NULL, 
  SPI_SD_PORT, SPI_SD_MASK, SPI_SD_SEL, SPI_SD_DIS,
  SPI_CTAR_8_SLOW,
};

/* MMC/SD over SPI driver configuration.*/
MMCConfig mmccfg = {&SPID1, &ls_spicfg, &hs_spicfg};
/* Generic large buffer.*/
static char fbuff[1024];

/*
 * Scan Files in a path and print them to the character stream.
 */
FRESULT scan_files(BaseSequentialStream *chp, char *path) {
	FRESULT res;
	FILINFO fno;
	DIR dir;
	int fyear,fmonth,fday,fhour,fminute,fsecond;

	int i;
	char *fn;

#if _USE_LFN
	fno.lfname = 0;
	fno.lfsize = 0;
#endif
	/*
	 * Open the Directory.
	 */
	res = f_opendir(&dir, path);
	if (res == FR_OK) {
		/*
		 * If the path opened successfully.
		 */
		i = strlen(path);
		while (true) {
			/*
			 * Read the Directory.
			 */
			res = f_readdir(&dir, &fno);
			/*
			 * If the directory read failed or the
			 */
			if (res != FR_OK || fno.fname[0] == 0) {
				break;
			}
			/*
			 * If the directory or file begins with a '.' (hidden), continue
			 */
			if (fno.fname[0] == '.') {
				continue;
			}
			fn = fno.fname;
			/*
			 * Extract the date.
			 */
			fyear = (((FATTIME_YEARS_MASK >>4) & fno.fdate) >> 9)+FAT_BASE_YEAR;
			fmonth= ((FATTIME_MONTHS_MASK>>4) & fno.fdate) >> 5;
			fday  = ((FATTIME_DAYS_MASK >>4) & fno.fdate);
			/*
			 * Extract the time.
			 */
			fhour   = (FATTIME_HOURS_MASK & fno.ftime) >> 11;
			fminute = (FATTIME_MINUTES_MASK & fno.ftime) >> 5;
			fsecond = (FATTIME_SECONDS_MASK & fno.ftime)*2;
			/*
			 * Print date and time of the file.
			 */
			chprintf(chp, "%4d-%02d-%02d %02d:%02d:%02d ", fyear, fmonth, fday, fhour, fminute, fsecond);
			/*
			 * If the 'file' is a directory.
			 */
			if (fno.fattrib & AM_DIR) {
				/*
				 * Add a slash to the end of the path
				 */
				path[i++] = '/';
				strcpy(&path[i], fn);
				/*
				 * Print that it is a directory and the path.
				 */
				chprintf(chp, "<DIR> %s/\r\n", path);
				/*
				 * Recursive call to scan the files.
				 */
				res = scan_files(chp, path);
				if (res != FR_OK) {
					break;
				}
				path[--i] = 0;
			} else {
				/*
				 * Otherwise print the path as a file.
				 */
				chprintf(chp, "      %s/%s\r\n", path, fn);
			}
		}
	} else {
		chprintf(chp, "FS: f_opendir() failed\r\n");
	}
	return res;
}

void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;

	(void)argc;
	(void)argv;
  
  mmcConnect(&MMCD1);
	/*
	 * Attempt to mount the drive.
	 */
	err = f_mount(&SDC_FS,"",0);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_mount() failed. Is the SD card inserted?\r\n");
		verbose_error(chp, err);
    mmcDisconnect(&MMCD1);
		return;
	}
	chprintf(chp, "FS: f_mount() succeeded\r\n");
}

/*
void cmd_mkfs(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;
  
	if (argc!=1) {
		chprintf(chp, "Usage: mkfs [partition]\r\n");
		chprintf(chp, "       Formats partition [partition]\r\n");
		return;
	}
	chprintf(chp, "FS: f_mkfs(%s,0,0) Started\r\n",argv[0]);
  
	err = f_mkfs("", 0, 0);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_mkfs() failed\r\n");
		verbose_error(chp, err);
		return;
	}
	chprintf(chp, "FS: f_mkfs() Finished\r\n");
	return;
}
*/

void cmd_unmount(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;
	(void)argc;
	(void)argv;
  
  mmcDisconnect(&MMCD1);
	err = f_mount(NULL,"",1);
  
	if (err != FR_OK) {
		chprintf(chp, "FS: f_mount() unmount failed\r\n");
		verbose_error(chp, err);
		return;
	}	
	return;
}

void cmd_free(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;
	uint32_t clusters;
	FATFS *fsp;
	(void)argc;
	(void)argv;

	err = f_getfree("", &clusters, &fsp);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_getfree() failed\r\n");
		return;
	}
	/*
	 * Print the number of free clusters and size free in B, KiB and MiB.
	 */
	chprintf(chp,"FS: %lu free clusters\r\n    %lu sectors per cluster\r\n",
		clusters, (uint32_t)SDC_FS.csize);
	chprintf(chp,"%lu B free\r\n",
		clusters * (uint32_t)SDC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE);
	chprintf(chp,"%lu KB free\r\n",
		(clusters * (uint32_t)SDC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE)/(1024));
	chprintf(chp,"%lu MB free\r\n",
		(clusters * (uint32_t)SDC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE)/(1024*1024));
}

void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;
	/*
	 * Set the file path buffer to 0
	 */
	memset(fbuff,0,sizeof(fbuff));
	scan_files(chp, fbuff);
}

void cmd_hello(BaseSequentialStream *chp, int argc, char *argv[]) {
	FIL fsrc;   /* file object */
	FRESULT err;
	int written;
	(void)argv;
	/*
	 * Print the input arguments.
	 */
	if (argc > 0) {
		chprintf(chp, "Usage: hello\r\n");
		chprintf(chp, "       Creates hello.txt with 'Hello World'\r\n");
		return;
	}
	/*
	 * Open the text file
	 */
	err = f_open(&fsrc, "hello.txt", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_open(\"hello.txt\") failed.\r\n");
		verbose_error(chp, err);
		return;
	} else {
		chprintf(chp, "FS: f_open(\"hello.txt\") succeeded\r\n");
	}
	/*
	 * Write text to the file.
	 */
  
	written = f_puts ("Hello World", &fsrc);
	if (written == -1) {
		chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") failed\r\n");
	} else {
		chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") succeeded\r\n");
	}
	/*
	 * Close the file
	 */
	f_close(&fsrc);
}

void cmd_mkdir(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;
	if (argc != 1) {
		chprintf(chp, "Usage: mkdir dirName\r\n");
		chprintf(chp, "       Creates directory with dirName (no spaces)\r\n");
		return;
	}
	/*
	 * Attempt to make the directory with the name given in argv[0]
	 */
	err=f_mkdir(argv[0]);
	if (err != FR_OK) {
		/*
		 * Display failure message and reason.
		 */
		chprintf(chp, "FS: f_mkdir(%s) failed\r\n",argv[0]);
		verbose_error(chp, err);
		return;
	} else {
		chprintf(chp, "FS: f_mkdir(%s) succeeded\r\n",argv[0]);
	}
	return;
}

void cmd_setlabel(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;
	if (argc != 1) {
		chprintf(chp, "Usage: setlabel label\r\n");
		chprintf(chp, "       Sets FAT label (no spaces)\r\n");
		return;
	}
	/*
	 * Attempt to set the label with the name given in argv[0].
	 */
	err=f_setlabel(argv[0]);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_setlabel(%s) failed.\r\n");
		verbose_error(chp, err);
		return;
	} else {
		chprintf(chp, "FS: f_setlabel(%s) succeeded.\r\n");
	}
	return;
}

void cmd_getlabel(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;
	char lbl[12];
	DWORD sn;
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: getlabel\r\n");
		chprintf(chp, "       Gets and prints FAT label\r\n");
		return;
	}
	memset(lbl,0,sizeof(lbl));
	/*
	 * Get volume label & serial of the default drive
	 */
	err = f_getlabel("", lbl, &sn);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_getlabel failed.\r\n");
		verbose_error(chp, err);
		return;
	}
	/*
	 * Print the label and serial number
	 */
	chprintf(chp, "LABEL: %s\r\n",lbl);
	chprintf(chp, "  S/N: 0x%X\r\n",sn);
	return;
}

/*
 * Print a text file to screen
 */
void cmd_cat(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;
	FIL fsrc;   /* file object */
	char Buffer[255];
	UINT ByteToRead=sizeof(Buffer);
	UINT ByteRead;
	/*
	 * Print usage
	 */
	if (argc != 1) {
		chprintf(chp, "Usage: cat filename\r\n");
		chprintf(chp, "       Echos filename (no spaces)\r\n");
		return;
	}
	/*
	 * Attempt to open the file, error out if it fails.
	 */
	err=f_open(&fsrc, argv[0], FA_READ);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_open(%s) failed.\r\n",argv[0]);
		verbose_error(chp, err);
		return;
	}
	/*
	 * Do while the number of bytes read is equal to the number of bytes to read
	 * (the buffer is filled)
	 */
	do {
		/*
		 * Clear the buffer.
		 */
		memset(Buffer,0,sizeof(Buffer));
		/*
		 * Read the file.
		 */
		err=f_read(&fsrc,Buffer,ByteToRead,&ByteRead);
		if (err != FR_OK) {
			chprintf(chp, "FS: f_read() failed\r\n");
			verbose_error(chp, err);
			f_close(&fsrc);
			return;
		}
		chprintf(chp, "%s", Buffer);
	} while (ByteRead>=ByteToRead);
	chprintf(chp,"\r\n");
	/*
	 * Close the file.
	 */
	f_close(&fsrc);
	return;
}

void verbose_error(BaseSequentialStream *chp, FRESULT err) {
	chprintf(chp, "\t%s.\r\n",fresult_str(err));
}

char* fresult_str(FRESULT stat) {
	char str[255];
	memset(str,0,sizeof(str));
	switch (stat) {
		case FR_OK:
			return "Succeeded";
		case FR_DISK_ERR:
			return "A hard error occurred in the low level disk I/O layer";
		case FR_INT_ERR:
			return "Assertion failed";
		case FR_NOT_READY:
			return "The physical drive cannot work";
		case FR_NO_FILE:
			return "Could not find the file";
		case FR_NO_PATH:
			return "Could not find the path";
		case FR_INVALID_NAME:
			return "The path name format is invalid";
		case FR_DENIED:
			return "Access denied due to prohibited access or directory full";
		case FR_EXIST:
			return "Access denied due to prohibited access";
		case FR_INVALID_OBJECT:
			return "The file/directory object is invalid";
		case FR_WRITE_PROTECTED:
			return "The physical drive is write protected";
		case FR_INVALID_DRIVE:
			return "The logical drive number is invalid";
		case FR_NOT_ENABLED:
			return "The volume has no work area";
		case FR_NO_FILESYSTEM:
			return "There is no valid FAT volume";
		case FR_MKFS_ABORTED:
			return "The f_mkfs() aborted due to any parameter error";
		case FR_TIMEOUT:
			return "Could not get a grant to access the volume within defined period";
		case FR_LOCKED:
			return "The operation is rejected according to the file sharing policy";
		case FR_NOT_ENOUGH_CORE:
			return "LFN working buffer could not be allocated";
		case FR_TOO_MANY_OPEN_FILES:
			return "Number of open files > _FS_SHARE";
		case FR_INVALID_PARAMETER:
			return "Given parameter is invalid";
		default:
			return "Unknown";
	}
	return "";
}

