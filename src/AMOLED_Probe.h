/*
 * AMOLED_Probe.h
 * ============================================================================
 *  Reads the identification register of the AMOLED controller.
 *
 *  There are two versions of the module in the wild:
 *      SH8601   -> 0xDA "Read ID1" returns 0x86   (no GRAM offset)
 *      CO5300   -> 0xDA "Read ID1" returns 0xFF   (466 visible columns are
 *                                                  stored 6 pixels further)
 *
 *  The probe bit-bangs the four QSPI data lines in single lane SPI mode, so
 *  it has to run BEFORE the QSPI peripheral takes the pins over.  AMOLED::
 *  begin() does exactly that.
 * ============================================================================
 */
#pragma once

#include <stdint.h>

/* the two IDs the "Read ID1" command can return */
#define AMOLED_PANEL_ID_SH8601  0x86
#define AMOLED_PANEL_ID_CO5300  0xFF

#ifdef __cplusplus
extern "C" {
#endif

/* returns the raw ID byte (0x86 SH8601, 0xFF CO5300, anything else = unknown) */
uint8_t amoledProbePanelId(void);

#ifdef __cplusplus
}
#endif
