/***********************************************************************/
/*  This file is part of the uVision/ARM development tools             */
/*  Copyright (c) 2011 Keil Software. All rights reserved.             */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Flash Programming Functions adapted for               */
/*               Holtek HT32 Series Flash                              */
/*                                                                     */
/***********************************************************************/

#include "..\FlashOS.H"        // FlashOS Structures

typedef          unsigned long     u32;
typedef volatile unsigned long    vu32;


/* Peripheral Memory Map                               */
#define FMC_BASE              (0x40080000)

#define FMC                   ((FMC_TypeDef*) FMC_BASE)
#define FLASH_BASE            (0x00000000)
#define FLASH_OPT_BASE        (0x1FF00000)

/* Flash Memory Controller Registers                   */
typedef struct {
  vu32 TADR;                                     /* offset  0x000 Flash Target Address Register (TADR)                       */
  vu32 WRDR;                                     /* offset  0x004 Flash Write Data Register (WRDR)                           */
  vu32 RESERVED0[1];                             /* offset  0x008                                                            */
  vu32 OCMR;                                     /* offset  0x00C Flash Operation Command Register (OCMR)                    */
  vu32 OPCR;                                     /* offset  0x010 Flash Operation Control Register (OPCR)                    */
  vu32 OIER;                                     /* offset  0x014 Flash Operation Interrupt Enable Register (OIER)           */
  vu32 OISR;                                     /* offset  0x018 Flash Operation Interrupt and Status Register (OISR)       */
  vu32 RESERVED1[1];                             /* offset  0x01C                                                            */
  vu32 PPSR[4];                                  /* offset  0x020 Flash Page Erase/Program Protection Status Register (PPSR) */
  vu32 CPSR;                                     /* offset  0x030 Flash Security Protection Status Register (CPSR)           */
  vu32 RESERVED2[51];
  vu32 VMCR;                                     /* offset  0x100 Flash Vector Mapping Control Register (VMCR)               */
  vu32 RESERVED3[31];
  vu32 MDID;                                     /* offset  0x180 Flash Manufacturer and Device ID Register (MDID)           */
  vu32 PNSR;                                     /* offset  0x184 Flash Page Number Status Register (PNSR)                   */
  vu32 PSSR;                                     /* offset  0x188 Flash Page Size Status Register (PSSR)                     */
} FMC_TypeDef;


/* Flash Operation Control Register (OPCR) definitions */
#define OPCR_OPM                   (0x0F <<  1)   /* OPM mask */
#define OPCR_OPM_IDLE              (0x06 <<  1)   /* Idle (default) */
#define OPCR_OPM_CMD_TO_MAIN       (0x0A <<  1)   /* Commit command to main Flash */
#define OPCR_OPM_FINISH_ON_MAIN    (0x0E <<  1)   /* All operation finished on main Flash */

/* Flash Operation Command Register (OCMR) definitions */
#define OCMR_CMD_IDLE              (0x00 <<  0)   /* Idle - default */
#define OCMR_CMD_WORD_PROGRAM      (0x04 <<  0)   /* Word program */
#define OCMR_CMD_PAGE_ERASE        (0x08 <<  0)   /* Page erase */
#define OCMR_CMD_MASS_ERASE        (0x0A <<  0)   /* Mass erase */

/* Flash Operation Interrupt and Status Register (OISR) definitions */
#define OISR_ITADF                 (0x01 <<  1)   /* Invalid Target Address Flag */
#define OISR_OBEF                  (0x01 <<  2)   /* Option Byte Checksum Error Flag */
#define OISR_IOCMF                 (0x01 <<  3)   /* Invalid Operation Command Flag */
#define OISR_OREF                  (0x01 <<  4)   /* Operation Error Flag */
#define OISR_ERR                   (OISR_ITADF | OISR_OBEF | OISR_IOCMF | OISR_OREF)
#define OISR_PPEF                  (0x01 << 17)   /* Page Erase/Program Protected Error Flag */

/* Flash Operation Interrupt Enable Register (OIER) definitions */
#define OIER_ITADIEN               (0x01 <<  1)   /* Invalid Target Address Enable */
#define OIER_OBEIEN                (0x01 <<  2)   /* Option Byte Checksum Error Enable */
#define OIER_IOCMIE                (0x01 <<  3)   /* Invalid Operation Command Enable */
#define OIER_OREIEN                (0x01 <<  4)   /* Operation Error Enable */
#define OIER_EN                    (OIER_ITADIEN | OIER_OBEIEN | OIER_IOCMIE | OIER_OREIEN)


/* Flash Vector Mapping Control Register (VMCR) definitions */
#define VMCR_VMCB_MAIN             (0x03 <<  0)   /* Main Flash Memory Area */


/* global variables */
          int fidx  = 0;
          int fsize = 0;
          int psize = 0;
unsigned long fbase;
unsigned long lastErase = 0xFFFFFFFF;


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
*/
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  *((vu32*)0x40088300) = 1;

  FMC->VMCR = VMCR_VMCB_MAIN;                     /* set vector mapping to Flash Memory Area */
  FMC->OIER = OIER_EN;                            /* enable error flag     */
  FMC->OISR = OISR_ERR;                           /* reset error flag      */

#ifdef FLASH_MEM
  switch (FMC->MDID) {                            /* select device         */
    case 0x03761552:
      fbase = FLASH_BASE;                         /* set used base address */
      psize = FMC->PSSR;                          /* set sector size       */
      fsize = psize * (FMC->PNSR - 1);            /* calculate flash size  */
      break;

    default:
      fbase = FLASH_BASE;                         /* set used base address */
      psize = FMC->PSSR;                          /* set sector size       */
      fsize = psize * (FMC->PNSR - 0);            /* calculate flash size  */
      break;
  }
#endif

#ifdef FLASH_OPT
      fbase = FLASH_OPT_BASE;                     /* set used base address */
      psize = FMC->PSSR;                          /* set sector size       */
      fsize = psize * (            1);            /* calculate flash size  */
#endif

  return (0);
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
*/
int UnInit (unsigned long fnc) {

  FMC->OPCR = OPCR_OPM_IDLE;
  FMC->OCMR = OCMR_CMD_IDLE;

  return (0);
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
*/
int EraseChip (void) {

  FMC->TADR = fbase;
  FMC->OCMR = OCMR_CMD_MASS_ERASE;
  FMC->OPCR = OPCR_OPM_CMD_TO_MAIN;

  /* Wait until all operations have been finished */
  while ((FMC->OPCR & OPCR_OPM) != OPCR_OPM_FINISH_ON_MAIN) __nop();

  if (FMC->OISR & (OISR_ERR | OISR_PPEF)) {       /* check for error */
    FMC->OISR = OISR_ERR;                         /* reset error flag */
    return (1);
  }

  return (0);
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
*/
int EraseSector (unsigned long adr) {

  adr = (adr    ) & ~(psize - 1);                 /* Adjust address for page  */

  // Skip erased address
  if ((adr) == lastErase)
  {
    return (0);
  }

  if ((adr         ) >= (fbase + fsize)) {        /* Check adr, provent address over range */
    return (1);
  }

  FMC->TADR = adr;
  FMC->OCMR = OCMR_CMD_PAGE_ERASE;
  FMC->OPCR = OPCR_OPM_CMD_TO_MAIN;
  lastErase = adr;
  
  /* Wait until all operations have been finished */
  while ((FMC->OPCR & OPCR_OPM) != OPCR_OPM_FINISH_ON_MAIN) __nop();

  if (FMC->OISR & (OISR_ERR | OISR_PPEF)) {       /* check for error */
    FMC->OISR = OISR_ERR;                         /* reset error flag */
    return (1);
  }

  return (0);
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
*/
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  sz  = (sz  + 3) & ~3;                           /* Adjust size    for Words */
  adr = (adr    ) & ~3;                           /* Adjust address for Words */

  if ((adr + sz - 4) >= (fbase + fsize)) {        /* Check adr, provent address over range */
    return (1);
  }

  FMC->OCMR = OCMR_CMD_WORD_PROGRAM;
  while (sz) {

    FMC->TADR = adr;
    FMC->WRDR = *((u32 *) buf);
    FMC->OPCR = OPCR_OPM_CMD_TO_MAIN;

    /* Wait until all operations have been finished */
    while ((FMC->OPCR & OPCR_OPM) != OPCR_OPM_FINISH_ON_MAIN) __nop();

    if (FMC->OISR & (OISR_ERR | OISR_PPEF)) {       /* check for error */
      FMC->OISR = OISR_ERR;                         /* reset error flag */
      return (1);
    }

    adr += 4;                                     /* Go to next Word */
    buf += 4;
    sz  -= 4;
  }

  return (0);
}
