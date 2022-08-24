
#include "stdafx.h"
#include "BinaryFile.h"
#include "memory.h"


MemoryFile::MemoryFile()
	:BinaryFile(false)
{
	m_pFile=(FILE*)0x01;
	readCounter=0;
}

MemoryFile::~MemoryFile()
{
	m_pFile=NULL; // 
}

void MemoryFile::_packArray(void *_buffer, int count, size_t size)
{
	ASSERT(sizeof(char)==1);
	buffer.resize(buffer.size()+1);
	buffer.back().resize(count*size);

	memcpy(&buffer.back()[0], _buffer, count*size);
	//printf("%d\n", *((int*)(&buffer.back()[0])));
	//printf(",%d\n", *((int*)(_buffer)));
	//Msg::print("pack %d\n", buffer.size());
}
void MemoryFile::_unpackArray(void *_buffer, int count, size_t size)
{
	//Msg::print("unpack %d/%d %d %d\n", readCounter, buffer.size(), buffer[readCounter].size(), count*size);
	//printf("%d %d\n", *((int*)(&buffer[readCounter][0])), count*size);
	memcpy(_buffer, &buffer[readCounter][0], count*size);
	readCounter++;
}


void MemoryFile::close()
{
	buffer.clear();
	readCounter=0;
}
