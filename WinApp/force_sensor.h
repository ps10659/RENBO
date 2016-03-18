#ifndef _FORCE_SENSOR_H_
#define _FORCE_SENSOR_H_

#include <windows.h>
#include <crtdbg.h>
#include <iostream>
#include <iomanip>

#include "..\JR3PCI_WDM_driver\JR3PCI\JR3PCIIoctls.h"
#include "jr3pci_ft.h"

typedef struct ft_data
{
	short mx[2];
	short my[2];
	short mz[2];
	short fx[2];
	short fy[2];
	short fz[2];
} ft_data;

ULONG GetSupportedChannels(HANDLE hJr3PciDevice);
void WriteWord(HANDLE hJr3PciDevice, UCHAR ucChannel, ULONG ulOffset, USHORT usData);
WORD ReadWord(HANDLE hJr3PciDevice, UCHAR ucChannel, ULONG ulOffset);

bool InitForceSensor();
void getForceData(struct ft_data* fts);
void ForceDataLoop();

static HANDLE hJr3PciDevice;

#endif