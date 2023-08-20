
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
#include "FBX/miniz.h"
class ZipFileData
{
	public:
		tmz_zip_archive zip_archive;
		tmz_bool status;
		tmz_zip_archive_file_stat  stat;
		void errorcheck()
		{
			if (!status)
			{
				std::cout << "zip file appears invalid..." << std::endl;
			}
		}
};
#define ZDATA (*((ZipFileData*)(data)))
ZipFile::ZipFile()
{
	data=(void*)new ZipFileData();
	memset(&ZDATA.zip_archive, 0, sizeof(tmz_zip_archive));

}
ZipFile::~ZipFile()
{
	delete &ZDATA;
	data=NULL;
}
void ZipFile::openRead(const char* filename)
{
	ZDATA.status = tmz_zip_reader_init_file(&ZDATA.zip_archive, filename, 0);
	ZDATA.errorcheck();
}
int ZipFile::getNumFiles() const
{
	return tmz_zip_reader_get_num_files(&ZDATA.zip_archive);
}

std::string ZipFile::getFileName(int file_index) const
{
	std::string temp;
	tmz_uint len=tmz_zip_reader_get_filename(&ZDATA.zip_archive, tmz_uint (file_index), 
			NULL, 0);
	temp.resize(len+1);


	tmz_zip_reader_get_filename(&ZDATA.zip_archive, tmz_uint (file_index), 
			&temp[0],
			len+1);
	
	return temp;
}
unsigned int ZipFile::getFileSize(int file_index) const
{
	ZDATA.status =tmz_zip_reader_file_stat(&ZDATA.zip_archive, (tmz_uint )file_index, 
		&ZDATA.stat);
	ZDATA.errorcheck();
	return (unsigned int)ZDATA.stat.m_uncomp_size;
}
std::string ZipFile::getFileContent(int file_index) const
{
	unsigned int fileSize=getFileSize(file_index);

	std::string out;
	out.resize(fileSize+1);
	out[fileSize]=0;

	ZDATA.status= tmz_zip_reader_extract_to_mem(&ZDATA.zip_archive, file_index, &out[0], fileSize, 0);
	ZDATA.errorcheck();
	return out;
}
