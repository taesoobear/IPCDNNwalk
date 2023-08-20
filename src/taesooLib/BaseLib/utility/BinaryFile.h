#ifndef BASELIB_BINARY_FILE_H
#define BASELIB_BINARY_FILE_H

#include "tfile.h" // BinaryFile is defined here.


// this can be used for safely copying objects.
class MemoryFile : public BinaryFile
{
	int readCounter;
	std::vector<std::string> buffer;
public:
	MemoryFile(); 
	~MemoryFile();
	// usage
	// MemoryFile a;
	// a.pack(...)
	// a.pack(...)
	// a.close()
	//
	// a.unpack(...
	// a.unpack(...)
	// a.close()

	void openRead(){readCounter=0;}
	void openWrite(){buffer.clear();}
	virtual void _packArray(void *buffer, int count, size_t size);
	virtual void _unpackArray(void *buffer, int count, size_t size);
	virtual void close();
};

class ZipFile 
{
	void* data;
	public:
	ZipFile();
	~ZipFile();
	void openRead(const char* filename);
	int getNumFiles() const;
	std::string getFileName(int file_index) const;
	unsigned int getFileSize(int file_index) const;
	std::string getFileContent(int file_index) const;
};
#endif
