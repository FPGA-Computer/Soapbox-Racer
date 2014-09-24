/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "power.h"
#include "i2c_lld.h"
#include "dma.h"
#include "ff.h"
#include "string.h"
#include "chprintf.h"
#include "shell.h"
#include "fat.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
	size_t n, size;
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}
	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static const ShellCommand commands[] = {
//	{"mkfs", cmd_mkfs},
	{"mount", cmd_mount},
	{"unmount", cmd_unmount},
	{"getlabel", cmd_getlabel},
	{"setlabel", cmd_setlabel},
	{"tree", cmd_tree},
	{"free", cmd_free},
	{"mkdir", cmd_mkdir},
	{"hello", cmd_hello},
	{"cat", cmd_cat},
	{"mem", cmd_mem},  
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD1,
  commands
};

/*
 * LED blinker thread.
 */
 
//static THD_WORKING_AREA(waShell_Term,256);
static THD_WORKING_AREA(waThread1,256);

static void Thread1(void *arg)
{  (void)arg;
  
    chRegSetThreadName("LEDBlinker");
    while (TRUE)
		{
       GPIOA->PTOR=1<<PORTA_EZP_CS;
       chThdSleepMilliseconds(250);
    }
}

/*
 * Application entry point.
 */

int main(void)
{
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
  thread_t *shelltp = NULL;
  
//  RunMode();
  chSysInit();
  DMA_Init();           // Need to be called before halInit()
  halInit();

  
  sdStart(&SD1,NULL);
  
  mmcObjectInit(&MMCD1);
  mmcStart(&MMCD1, &mmccfg);

  chThdSleepMilliseconds(50);

  /*
   * Creates the blinker thread.
   */

/*
   * Shell manager initialization.
   */
  shellInit();
  
  chThdCreateStatic(waThread1, sizeof(waThread1),
                    NORMALPRIO-1,(tfunc_t)Thread1, NULL);

  /*
  chThdCreateStatic(waShell_Term, sizeof(waShell_Term),
                    NORMALPRIO,(tfunc_t)Shell_Term,"\nChibiOS 3.0\n");  
*/
  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */

  while (TRUE) {
    if (!shelltp)
      shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO+1);
    else if (chThdTerminatedX(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
   }
}
