/***********************************************************************/
/*  This file is part of the uVision/ARM development tools             */
/*  Copyright (c) 2011 Keil Software. All rights reserved.             */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Device Description for Holtek HT32 Series Flash       */
/*                                                                     */
/***********************************************************************/

#include "..\FlashOS.H"           // FlashOS Structures


#ifdef FLASH_MEM

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,                // Driver Version, do not modify!
   "HT32 Series Flash",           // Device Name
   ONCHIP,                        // Device Type
   0x00000000,                    // Device Start Address
   0x00100000,                    // Device Size in Bytes (Max 1024kB)
   512,                           // Programming Page Size
   0,                             // Reserved, must be 0
   0xFF,                          // Initial Content of Erased Memory
   100,                           // Program Page Timeout 100 mSec
   500,                           // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   512, 0x000000,                 // Sector Size
   SECTOR_END
};

#endif


#ifdef FLASH_OPT

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,                // Driver Version, do not modify!
   "HT32 Series Flash Options",   // Device Name
   ONCHIP,                        // Device Type
   0x1FF00000,                    // Device Start Address
   0x00001000,                    // Device Size in Bytes (Max 4kB)
   512,                           // Programming Page Size
   0,                             // Reserved, must be 0
   0xFF,                          // Initial Content of Erased Memory
   100,                           // Program Page Timeout 100 mSec
   500,                           // Erase Sector Timeout 500 mSec

// Specify Size and Address of Sectors
   512, 0x000000,                 // Sector Size
   SECTOR_END
};

#endif
