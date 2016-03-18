#include "force_sensor.h"

ULONG GetSupportedChannels(HANDLE hJr3PciDevice)
{
	JR3PCI_SUPPORTED_CHANNELS_RESPONSE_PARAMS SupportedChannelsResponseParams;

	DWORD dwBytesReturned = 0;
	BOOL bSuccess = DeviceIoControl(
		hJr3PciDevice,					// handle to device
		IOCTL_JR3PCI_SUPPORTED_CHANNELS,					// operation
		NULL,				// input data buffer
		0,  // size of input data buffer
		&SupportedChannelsResponseParams,				// output data buffer
		sizeof(JR3PCI_SUPPORTED_CHANNELS_RESPONSE_PARAMS), // size of output data buffer
		&dwBytesReturned,						// byte count
		NULL);									// overlapped information
	
	_ASSERTE(bSuccess && (dwBytesReturned == sizeof(JR3PCI_SUPPORTED_CHANNELS_RESPONSE_PARAMS)));

	return SupportedChannelsResponseParams.ulSupportedChannels;	
}

void WriteWord(HANDLE hJr3PciDevice, UCHAR ucChannel, ULONG ulOffset, USHORT usData)
{
	JR3PCI_WRITE_WORD_REQUEST_PARAMS WriteWordRequestParams;
	WriteWordRequestParams.ucChannel = ucChannel;
	WriteWordRequestParams.ulOffset = ulOffset;
	WriteWordRequestParams.usData = usData;

	JR3PCI_WRITE_WORD_RESPONSE_PARAMS WriteWordResponseParams;

	DWORD dwBytesReturned = 0;
	BOOL bSuccess = DeviceIoControl(
		hJr3PciDevice,					// handle to device
		IOCTL_JR3PCI_WRITE_WORD,					// operation
		&WriteWordRequestParams,				// input data buffer
		sizeof(JR3PCI_WRITE_WORD_REQUEST_PARAMS),  // size of input data buffer
		&WriteWordResponseParams,				// output data buffer
		sizeof(JR3PCI_WRITE_WORD_RESPONSE_PARAMS), // size of output data buffer
		&dwBytesReturned,						// byte count
		NULL);									// overlapped information
	
	_ASSERTE(bSuccess && (dwBytesReturned == sizeof(JR3PCI_WRITE_WORD_RESPONSE_PARAMS)));
	_ASSERTE(WriteWordResponseParams.iStatus == JR3PCI_STATUS_OK);
}

WORD ReadWord(HANDLE hJr3PciDevice, UCHAR ucChannel, ULONG ulOffset)
{
	JR3PCI_READ_WORD_REQUEST_PARAMS ReadWordRequestParams;
	ReadWordRequestParams.ucChannel = ucChannel;
	ReadWordRequestParams.ulOffset = ulOffset;

	JR3PCI_READ_WORD_RESPONSE_PARAMS ReadWordResponseParams;

	DWORD dwBytesReturned = 0;
	BOOL bSuccess = DeviceIoControl(
		hJr3PciDevice,					// handle to device
		IOCTL_JR3PCI_READ_WORD,					// operation
		&ReadWordRequestParams,				// input data buffer
		sizeof(JR3PCI_READ_WORD_REQUEST_PARAMS),  // size of input data buffer
		&ReadWordResponseParams,				// output data buffer
		sizeof(JR3PCI_READ_WORD_RESPONSE_PARAMS), // size of output data buffer
		&dwBytesReturned,						// byte count
		NULL);									// overlapped information
	
	_ASSERTE(bSuccess && (dwBytesReturned == sizeof(JR3PCI_READ_WORD_RESPONSE_PARAMS)));
	_ASSERTE(ReadWordResponseParams.iStatus == JR3PCI_STATUS_OK);
	
	return ReadWordResponseParams.usData;
}


bool InitForceSensor()
{
	std::cout << "" << std::endl;
	char szDeviceName[30];
	sprintf_s(szDeviceName, "\\\\.\\JR3PCI%d", 1);

	hJr3PciDevice = CreateFile(
		szDeviceName,					// file name
		GENERIC_READ | GENERIC_WRITE,   // access mode
		0,								// share mode
		NULL,							// SD
		OPEN_EXISTING,					// how to create
		0,								// file attributes
		NULL);							// handle to template file

	if(hJr3PciDevice == INVALID_HANDLE_VALUE)
	{
		printf("Failed to open a handle to device '%s'.\r\n", szDeviceName);
		//continue;
		return false;
	}
	printf("Handle to device '%s' opened successfully.\r\n", szDeviceName);

	ULONG ulSupportedChannels = GetSupportedChannels(hJr3PciDevice);
	printf("This device supports %d DSP channel(s).\r\n", ulSupportedChannels);

	force_sensor_data vfsd[2]; // Max 4 channels on any JR3 card.
	ULONG ulNumWords = sizeof(vfsd[0]) / sizeof(short);
		
	for(ULONG ulChannelIndex = 0; ulChannelIndex < 2; ulChannelIndex++)
	{
		short * pusForceSensorData = (short *) &vfsd[ulChannelIndex];
			
		for(ULONG ulOffset = 0; ulOffset < ulNumWords; ulOffset++)
			pusForceSensorData[ulOffset] = ReadWord(hJr3PciDevice, (UCHAR) ulChannelIndex, ulOffset);

		char * pszCopyright = (char*) vfsd[ulChannelIndex].copyright;
		char szGoodCopyrightString[] = " C o p y r i g h t   J R 3   1 9 9 3 - 2 0 0 0";

		// Check Channel copyright string.
		bool bSuccess = memcmp(pszCopyright, szGoodCopyrightString, sizeof(szGoodCopyrightString)) == 0;
		_ASSERTE(bSuccess);

		if(!bSuccess)
		{
			printf("Failed to read copyright string from channel %d!\r\n", ulChannelIndex);
			getchar();
			break;
		}
		printf("Successfully read copyright string from channel %d\r\n", ulChannelIndex);
	}

	return true;
}

void getForceData(struct ft_data* fts)
{
	//extern HANDLE hJr3PciDevice;
	static force_sensor_data vfsd[2]; // 2 channels on this JR3 card.
	static ULONG ulNumWords = sizeof(vfsd[0]) / sizeof(short);
	static short * pusForceSensorData;

	
	for(ULONG ulChannelIndex = 0; ulChannelIndex < 2; ulChannelIndex++)
	{
		pusForceSensorData = (short *) &vfsd[ulChannelIndex];

		//for(ULONG ulOffset=ulNumWords-1; ulOffset>=0; ulOffset--)

		for(ULONG ulOffset=0; ulOffset<ulNumWords; ulOffset++)
		{
			pusForceSensorData[ulOffset] = ReadWord(hJr3PciDevice, (UCHAR) ulChannelIndex, ulOffset);
		}
		fts->mx[ulChannelIndex] = vfsd[ulChannelIndex].filter1.fz;
		fts->my[ulChannelIndex] = vfsd[ulChannelIndex].filter1.mx;
		fts->mz[ulChannelIndex] = vfsd[ulChannelIndex].filter1.my;
		fts->fx[ulChannelIndex] = 0;//vfsd[ulChannelIndex].filter1.fx;
		fts->fy[ulChannelIndex] = 0;//vfsd[ulChannelIndex].filter1.fy;
		fts->fz[ulChannelIndex] = vfsd[ulChannelIndex].filter1.fy;
	}
	std::cout << std::setw(5)<< fts->mx[0] << std::setw(5)<< fts->my[0] << std::setw(5)<< fts->mz[0];
	std::cout << std::setw(5)<< fts->fx[0] << std::setw(5)<< fts->fy[0] << std::setw(5)<< fts->fz[0];

	std::cout << std::setw(5)<< fts->mx[1] << std::setw(5)<< fts->my[1] << std::setw(5)<< fts->mz[1];
	std::cout << std::setw(5)<< fts->fx[1] << std::setw(5)<< fts->fy[1] << std::setw(5)<< fts->fz[1] << std::endl;

}

void ForceDataLoop()
{
	extern HANDLE hJr3PciDevice;
	force_sensor_data vfsd[2]; // Max 4 channels on any JR3 card.
	ULONG ulNumWords = sizeof(vfsd[0]) / sizeof(short);
	short * pusForceSensorData;
	short mx[2], my[2], mz[2], fx[2], fy[2], fz[2];

	while(true)
	{	
		for(ULONG ulChannelIndex = 0; ulChannelIndex < 2; ulChannelIndex++)
		{
			pusForceSensorData = (short *) &vfsd[ulChannelIndex];

			for(ULONG ulOffset=0; ulOffset<ulNumWords; ulOffset++)
				pusForceSensorData[ulOffset] = ReadWord(hJr3PciDevice, (UCHAR) ulChannelIndex, ulOffset);
		
			mx[ulChannelIndex] = vfsd[ulChannelIndex].filter0.mx;
			my[ulChannelIndex] = vfsd[ulChannelIndex].filter0.my;
			mz[ulChannelIndex] = vfsd[ulChannelIndex].filter0.mz;
			fx[ulChannelIndex] = vfsd[ulChannelIndex].filter0.fx;
			fy[ulChannelIndex] = vfsd[ulChannelIndex].filter0.fy;
			fz[ulChannelIndex] = vfsd[ulChannelIndex].filter0.fz;
		}
		std::cout << std::setw(5)<< mx[0] << std::setw(5)<< my[0] << std::setw(5)<< mz[0];
		std::cout << std::setw(5)<< fx[0] << std::setw(5)<< fy[0] << std::setw(5)<< fz[0];

		std::cout << std::setw(5)<< mx[1] << std::setw(5)<< my[1] << std::setw(5)<< mz[1];
		std::cout << std::setw(5)<< fx[1] << std::setw(5)<< fy[1] << std::setw(5)<< fz[1] << std::endl;

		Sleep(100);
	}
}