#include "util.h"



void ListFiles(LPCSTR targetFile)
{
	// List all the files in the directory
	WIN32_FIND_DATA FindFileData; 
	HANDLE hFind;
	int file_nums = 0;

	_tprintf (TEXT("Target files: %s\n"), targetFile);
	hFind = FindFirstFile(targetFile, &FindFileData);

	do
	{
		if (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			_tprintf(TEXT("  %s   <DIR>\n"), FindFileData.cFileName);
		}
		else
		{
			file_list[file_nums] = FindFileData.cFileName;			
			_tprintf(TEXT("   %d. %s\n"), file_nums, FindFileData.cFileName);
			file_nums++;
		}
	}
	while (FindNextFile(hFind, &FindFileData) != 0);
	FindClose(hFind);
}