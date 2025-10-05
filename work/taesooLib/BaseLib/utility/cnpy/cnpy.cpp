//Copyright (C) 2011  Carl Rogers
//Released under MIT License
//license available in LICENSE file, or at http://www.opensource.org/licenses/mit-license.php

//#define USE_ZLIB
#ifdef USE_ZLIB
#include<zlib.h>
#else
#endif
#include"cnpy.h"
#include<complex>
#include<cstdlib>
#include<algorithm>
#include<cstring>
#include<iomanip>
#include<stdint.h>
#include<stdexcept>
#include <regex>
#include "../BinaryFile.h"

char cnpy::BigEndianTest() {
    int x = 1;
    return (((char *)&x)[0]) ? '<' : '>';
}

char cnpy::map_type(const std::type_info& t)
{
    if(t == typeid(float) ) return 'f';
    if(t == typeid(double) ) return 'f';
    if(t == typeid(long double) ) return 'f';

    if(t == typeid(int) ) return 'i';
    if(t == typeid(char) ) return 'i';
    if(t == typeid(short) ) return 'i';
    if(t == typeid(long) ) return 'i';
    if(t == typeid(long long) ) return 'i';

    if(t == typeid(unsigned char) ) return 'u';
    if(t == typeid(unsigned short) ) return 'u';
    if(t == typeid(unsigned long) ) return 'u';
    if(t == typeid(unsigned long long) ) return 'u';
    if(t == typeid(unsigned int) ) return 'u';

    if(t == typeid(bool) ) return 'b';

    if(t == typeid(std::complex<float>) ) return 'c';
    if(t == typeid(std::complex<double>) ) return 'c';
    if(t == typeid(std::complex<long double>) ) return 'c';

    else return '?';
}

template<> std::vector<char>& cnpy::operator+=(std::vector<char>& lhs, const std::string rhs) {
    lhs.insert(lhs.end(),rhs.begin(),rhs.end());
    return lhs;
}

template<> std::vector<char>& cnpy::operator+=(std::vector<char>& lhs, const char* rhs) {
    //write in little endian
    size_t len = strlen(rhs);
    lhs.reserve(len);
    for(size_t byte = 0; byte < len; byte++) {
        lhs.push_back(rhs[byte]);
    }
    return lhs;
}

char cnpy::parse_npy_header(unsigned char* buffer,size_t& word_size, std::vector<size_t>& shape, bool& fortran_order) {
    //std::string magic_string(buffer,6);
    uint8_t major_version = *reinterpret_cast<uint8_t*>(buffer+6);
    uint8_t minor_version = *reinterpret_cast<uint8_t*>(buffer+7);
    uint16_t header_len = *reinterpret_cast<uint16_t*>(buffer+8);
    std::string header(reinterpret_cast<char*>(buffer+9),header_len);

    size_t loc1, loc2;

    //fortran order
    loc1 = header.find("fortran_order")+16;
    fortran_order = (header.substr(loc1,4) == "True" ? true : false);

    //shape
    loc1 = header.find("(");
    loc2 = header.find(")");
    if (loc1 == std::string::npos || loc2 == std::string::npos)
        throw std::runtime_error("parse_npy_header: failed to find header keyword: '(' or ')'");

    std::regex num_regex("[0-9][0-9]*");
    std::smatch sm;
    shape.clear();

    std::string str_shape = header.substr(loc1+1,loc2-loc1-1);
    while(std::regex_search(str_shape, sm, num_regex)) {
        shape.push_back(std::stoi(sm[0].str()));
        str_shape = sm.suffix().str();
    }

    //endian, word size, data type
    //byte order code | stands for not applicable. 
    //not sure when this applies except for byte array
    loc1 = header.find("descr")+9;
    bool littleEndian = (header[loc1] == '<' || header[loc1] == '|' ? true : false);
    assert(littleEndian);

    char type = header[loc1+1];
    //assert(type == map_type(T));

    std::string str_ws = header.substr(loc1+2);
    loc2 = str_ws.find("'");
    word_size = atoi(str_ws.substr(0,loc2).c_str());
	if (type=='U') word_size*=4;
	return type;
}

char cnpy::parse_npy_header(FILE* fp, size_t& word_size, std::vector<size_t>& shape, bool& fortran_order) {  
    char buffer[256];
    size_t res = fread(buffer,sizeof(char),11,fp);       
    if(res != 11)
        throw std::runtime_error("parse_npy_header: failed fread");
    std::string header = fgets(buffer,256,fp);
    assert(header[header.size()-1] == '\n');

    size_t loc1, loc2;

    //fortran order
    loc1 = header.find("fortran_order");
    if (loc1 == std::string::npos)
        throw std::runtime_error("parse_npy_header: failed to find header keyword: 'fortran_order'");
    loc1 += 16;
    fortran_order = (header.substr(loc1,4) == "True" ? true : false);

    //shape
    loc1 = header.find("(");
    loc2 = header.find(")");
    if (loc1 == std::string::npos || loc2 == std::string::npos)
        throw std::runtime_error("parse_npy_header: failed to find header keyword: '(' or ')'");

    std::regex num_regex("[0-9][0-9]*");
    std::smatch sm;
    shape.clear();

    std::string str_shape = header.substr(loc1+1,loc2-loc1-1);
    while(std::regex_search(str_shape, sm, num_regex)) {
        shape.push_back(std::stoi(sm[0].str()));
        str_shape = sm.suffix().str();
    }

    //endian, word size, data type
    //byte order code | stands for not applicable. 
    //not sure when this applies except for byte array
    loc1 = header.find("descr");
    if (loc1 == std::string::npos)
        throw std::runtime_error("parse_npy_header: failed to find header keyword: 'descr'");
    loc1 += 9;
    bool littleEndian = (header[loc1] == '<' || header[loc1] == '|' ? true : false);
    assert(littleEndian);

    char type = header[loc1+1];
    //assert(type == map_type(T));

    std::string str_ws = header.substr(loc1+2);
    loc2 = str_ws.find("'");
    word_size = atoi(str_ws.substr(0,loc2).c_str());
	if (type=='U') word_size*=4;
	return type;
}

void cnpy::parse_zip_footer(FILE* fp, uint16_t& nrecs, size_t& global_header_size, size_t& global_header_offset)
{
    std::vector<char> footer(22);
    fseek(fp,-22,SEEK_END);
    size_t res = fread(&footer[0],sizeof(char),22,fp);
    if(res != 22)
        throw std::runtime_error("parse_zip_footer: failed fread");

    uint16_t disk_no, disk_start, nrecs_on_disk, comment_len;
    disk_no = *(uint16_t*) &footer[4];
    disk_start = *(uint16_t*) &footer[6];
    nrecs_on_disk = *(uint16_t*) &footer[8];
    nrecs = *(uint16_t*) &footer[10];
    global_header_size = *(uint32_t*) &footer[12];
    global_header_offset = *(uint32_t*) &footer[16];
    comment_len = *(uint16_t*) &footer[20];

    assert(disk_no == 0);
    assert(disk_start == 0);
    assert(nrecs_on_disk == nrecs);
    assert(comment_len == 0);
}

cnpy::NpyArray load_the_npy_file(FILE* fp) {
    std::vector<size_t> shape;
    size_t word_size;
    bool fortran_order;
    char type=cnpy::parse_npy_header(fp,word_size,shape,fortran_order);

    cnpy::NpyArray arr(shape, word_size, fortran_order, type);
    size_t nread = fread(arr.data<char>(),1,arr.num_bytes(),fp);
    if(nread != arr.num_bytes())
        throw std::runtime_error("load_the_npy_file: failed fread");
    return arr;
}

#ifdef USE_ZLIB
cnpy::NpyArray load_the_npz_array(FILE* fp, uint32_t compr_bytes, uint32_t uncompr_bytes) {

    std::vector<unsigned char> buffer_compr(compr_bytes);
    std::vector<unsigned char> buffer_uncompr(uncompr_bytes);
    size_t nread = fread(&buffer_compr[0],1,compr_bytes,fp);
    if(nread != compr_bytes)
        throw std::runtime_error("load_the_npy_file: failed fread");

    int err;
    z_stream d_stream;

    d_stream.zalloc = Z_NULL;
    d_stream.zfree = Z_NULL;
    d_stream.opaque = Z_NULL;
    d_stream.avail_in = 0;
    d_stream.next_in = Z_NULL;
    err = inflateInit2(&d_stream, -MAX_WBITS);

    d_stream.avail_in = compr_bytes;
    d_stream.next_in = &buffer_compr[0];
    d_stream.avail_out = uncompr_bytes;
    d_stream.next_out = &buffer_uncompr[0];

    err = inflate(&d_stream, Z_FINISH);
    err = inflateEnd(&d_stream);

    std::vector<size_t> shape;
    size_t word_size;
    bool fortran_order;
    char type=cnpy::parse_npy_header(&buffer_uncompr[0],word_size,shape,fortran_order);

    cnpy::NpyArray array(shape, word_size, fortran_order, type);

    size_t offset = uncompr_bytes - array.num_bytes();
    memcpy(array.data<unsigned char>(),&buffer_uncompr[0]+offset,array.num_bytes());

    return array;
}

#else
cnpy::NpyArray load_the_npz_array(FILE* fp, uint32_t compr_bytes, uint32_t uncompr_bytes) {

    std::vector<unsigned char> buffer_compr(compr_bytes);
    std::vector<unsigned char> buffer_uncompr(uncompr_bytes);
    size_t nread = fread(&buffer_compr[0],1,compr_bytes,fp);
    if(nread != compr_bytes)
        throw std::runtime_error("load_the_npy_file: failed fread");

    int err;
    z_stream d_stream;

    d_stream.zalloc = Z_NULL;
    d_stream.zfree = Z_NULL;
    d_stream.opaque = Z_NULL;
    d_stream.avail_in = 0;
    d_stream.next_in = Z_NULL;
    err = inflateInit2(&d_stream, -MAX_WBITS);

    d_stream.avail_in = compr_bytes;
    d_stream.next_in = &buffer_compr[0];
    d_stream.avail_out = uncompr_bytes;
    d_stream.next_out = &buffer_uncompr[0];

    err = inflate(&d_stream, Z_FINISH);
    err = inflateEnd(&d_stream);

    std::vector<size_t> shape;
    size_t word_size;
    bool fortran_order;
    char type=cnpy::parse_npy_header(&buffer_uncompr[0],word_size,shape,fortran_order);

    cnpy::NpyArray array(shape, word_size, fortran_order, type);

    size_t offset = uncompr_bytes - array.num_bytes();
    memcpy(array.data<unsigned char>(),&buffer_uncompr[0]+offset,array.num_bytes());

    return array;
}
#endif

cnpy::npz_t cnpy::npz_load(std::string fname) {
    cnpy::npz_t arrays;  
#if 0
	// original implementation which is buggy
    FILE* fp = fopen(fname.c_str(),"rb");

    if(!fp) {
        throw std::runtime_error("npz_load: Error! Unable to open file "+fname+"!");
    }


    while(1) {
        std::vector<char> local_header(30);
        size_t headerres = fread(&local_header[0],sizeof(char),30,fp);
        if(headerres != 30)
            throw std::runtime_error("npz_load: failed fread");

        //if we've reached the global header, stop reading
        if(local_header[2] != 0x03 || local_header[3] != 0x04) break;

        //read in the variable name
        uint16_t name_len = *(uint16_t*) &local_header[26];
        std::string varname(name_len,' ');
        size_t vname_res = fread(&varname[0],sizeof(char),name_len,fp);
        if(vname_res != name_len)
            throw std::runtime_error("npz_load: failed fread");

        //erase the lagging .npy        
        varname.erase(varname.end()-4,varname.end());

        //read in the extra field
        uint16_t extra_field_len = *(uint16_t*) &local_header[28];
        if(extra_field_len > 0) {
            std::vector<char> buff(extra_field_len);
            size_t efield_res = fread(&buff[0],sizeof(char),extra_field_len,fp);
            if(efield_res != extra_field_len)
                throw std::runtime_error("npz_load: failed fread");
        }

        uint16_t compr_method = *reinterpret_cast<uint16_t*>(&local_header[0]+8);
        uint32_t compr_bytes = *reinterpret_cast<uint32_t*>(&local_header[0]+18);
        uint32_t uncompr_bytes = *reinterpret_cast<uint32_t*>(&local_header[0]+22);

        if(compr_method == 0) {arrays[varname] = load_the_npy_file(fp);}
        else {arrays[varname] = load_the_npz_array(fp,compr_bytes,uncompr_bytes);}

    }

	fclose(fp);

#else
	// taesoo's implementation
	ZipFile zipFile;
	zipFile.openRead(fname.c_str());

	for(int i=0, nf=zipFile.getNumFiles(); i<nf; i++)
	{
		std::string varname=zipFile.getFileName(i);
		//erase the lagging .npy        
		varname.erase(varname.end()-6,varname.end());

		std::string buffer_uncompr=zipFile.getFileContent(i);
		if(buffer_uncompr.size()<=11)
			throw std::runtime_error("parse_npy_header: failed fread");

		std::vector<size_t> shape;
		size_t word_size;
		bool fortran_order;
		char type=cnpy::parse_npy_header((unsigned char*)&buffer_uncompr[0],word_size,shape,fortran_order);

		//printf("%s %d %d %d %s\n", varname.c_str(), content.size(), content[11], content[12], &content[11]); 

		cnpy::NpyArray array(shape, word_size, fortran_order, type);
		size_t uncompr_bytes =buffer_uncompr.size()-1;
		size_t offset = uncompr_bytes - array.num_bytes();
		memcpy(array.data<unsigned char>(),(unsigned char*)&buffer_uncompr[0]+offset,array.num_bytes());
		arrays[varname] = array;
	}
#endif
	return arrays;  
}
cnpy::NpyArray cnpy::npz_load(std::string fname, std::string varname) {
	FILE* fp = fopen(fname.c_str(),"rb");

	if(!fp) throw std::runtime_error("npz_load: Unable to open file "+fname);

	while(1) {
		std::vector<char> local_header(30);
		size_t header_res = fread(&local_header[0],sizeof(char),30,fp);
		if(header_res != 30)
			throw std::runtime_error("npz_load: failed fread");

		//if we've reached the global header, stop reading
		if(local_header[2] != 0x03 || local_header[3] != 0x04) break;

		//read in the variable name
		uint16_t name_len = *(uint16_t*) &local_header[26];
		std::string vname(name_len,' ');
		size_t vname_res = fread(&vname[0],sizeof(char),name_len,fp);      
		if(vname_res != name_len)
			throw std::runtime_error("npz_load: failed fread");
		vname.erase(vname.end()-4,vname.end()); //erase the lagging .npy

		//read in the extra field
		uint16_t extra_field_len = *(uint16_t*) &local_header[28];
		fseek(fp,extra_field_len,SEEK_CUR); //skip past the extra field

		uint16_t compr_method = *reinterpret_cast<uint16_t*>(&local_header[0]+8);
		uint32_t compr_bytes = *reinterpret_cast<uint32_t*>(&local_header[0]+18);
		uint32_t uncompr_bytes = *reinterpret_cast<uint32_t*>(&local_header[0]+22);

		if(vname == varname) {
			NpyArray array  = (compr_method == 0) ? load_the_npy_file(fp) : load_the_npz_array(fp,compr_bytes,uncompr_bytes);
			fclose(fp);
			return array;
		}
		else {
			//skip past the data
			uint32_t size = *(uint32_t*) &local_header[22];
			fseek(fp,size,SEEK_CUR);
		}
	}

    fclose(fp);

    //if we get here, we haven't found the variable in the file
    throw std::runtime_error("npz_load: Variable name "+varname+" not found in "+fname);
}

cnpy::NpyArray cnpy::npy_load(std::string fname) {

    FILE* fp = fopen(fname.c_str(), "rb");

    if(!fp) throw std::runtime_error("npy_load: Unable to open file "+fname);

    NpyArray arr = load_the_npy_file(fp);

    fclose(fp);
    return arr;
}

#include "../../math/hyperMatrixN.h"
void NPYarray::npz_save(std::string zipname, std::string fname, hypermatrixn const& mat, std::string mode)
{
	std::vector<float> temp;
	std::vector<size_t> shape;
	int n=mat.pages();
	int m=mat.rows();
	int o=mat.cols();
	int stride=m*o;
	int numElts=n*m*o;
	temp.resize(numElts);
	shape.resize(3);
	shape[0]=mat.pages();
	shape[1]=mat.rows();
	shape[2]=mat.cols();

	for(int i=0; i<n; i++)
	{
		matrixnView page=mat.page(i);
		for(int j=0; j<m; j++)
		{ 
			double* row=page[j];
			for(int k=0; k<o; k++)
				temp[i*stride+j*o+k]=(float)row[k];
		}
	}
	cnpy::npz_save<float>(zipname, fname, &temp[0], shape,mode);  
}
void NPYarray::npz_save(std::string zipname, std::string fname, matrixn const& mat, std::string mode)
{
	std::vector<float> temp;
	std::vector<size_t> shape;
	int m=mat.rows();
	int o=mat.cols();
	int numElts=m*o;
	temp.resize(numElts);
	shape.resize(2);
	shape[0]=mat.rows();
	shape[1]=mat.cols();

	for(int j=0; j<m; j++)
	{ 
		double* row=mat[j];
		for(int k=0; k<o; k++)
			temp[j*o+k]=(float)row[k];
	}
	cnpy::npz_save<float>(zipname, fname, &temp[0], shape,mode);  
}
void NPYarray::npz_save(std::string zipname, std::string fname, vectorn const& vec, std::string mode)
{
	std::vector<float> temp;
	std::vector<size_t> shape;
	int numElts=vec.size();
	temp.resize(numElts);
	shape.resize(1);
	shape[0]=numElts;

	for(int k=0; k<numElts; k++) temp[k]=(float)vec[k];
	cnpy::npz_save<float>(zipname, fname, &temp[0], shape,mode);  
}

namespace cnpy{
	// by taesoo
void npz_save_U(std::string zipname, std::string fname, std::vector<int>& data, const std::vector<size_t>& shape, std::string mode = "w")
{
	//first, append a .npy to the fname
	fname += ".npy";

	//now, on with the show
	FILE* fp = NULL;
	uint16_t nrecs = 0;
	size_t global_header_offset = 0;
	std::vector<char> global_header;

	if(mode == "a") fp = fopen(zipname.c_str(),"r+b");

	if(fp) {
		//zip file exists. we need to add a new npy file to it.
		//first read the footer. this gives us the offset and size of the global header
		//then read and store the global header.
		//below, we will write the the new data at the start of the global header then append the global header and footer below it
		size_t global_header_size;
		cnpy::parse_zip_footer(fp,nrecs,global_header_size,global_header_offset);
		fseek(fp,global_header_offset,SEEK_SET);
		global_header.resize(global_header_size);
		size_t res = fread(&global_header[0],sizeof(char),global_header_size,fp);
		if(res != global_header_size){
			throw std::runtime_error("npz_save: header read error while adding to existing zip");
		}
		fseek(fp,global_header_offset,SEEK_SET);
	}
	else {
		fp = fopen(zipname.c_str(),"wb");
	}
	size_t wordSize=data.size()/shape[0];

	std::vector<char> npy_header = _create_npy_header('U', wordSize, shape);

	size_t nels = std::accumulate(shape.begin(),shape.end(),1,std::multiplies<size_t>());
	size_t nbytes = data.size()*4 + npy_header.size();
	//std::cout<<"nbytes"<<nbytes<<" "<<nels<<shape[0]<<std::endl;

	//get the CRC of the data to be added
	uint32_t crc = tmz_crc32(0L,(uint8_t*)&npy_header[0],npy_header.size());
	crc = tmz_crc32(crc,(uint8_t*)(&data[0]),nels*wordSize*4);

	//build the local header
	std::vector<char> local_header;
	local_header += "PK"; //first part of sig
	local_header += (uint16_t) 0x0403; //second part of sig
	local_header += (uint16_t) 20; //min version to extract
	local_header += (uint16_t) 0; //general purpose bit flag
	local_header += (uint16_t) 0; //compression method
	local_header += (uint16_t) 0; //file last mod time
	local_header += (uint16_t) 0;     //file last mod date
	local_header += (uint32_t) crc; //crc
	local_header += (uint32_t) nbytes; //compressed size
	local_header += (uint32_t) nbytes; //uncompressed size
	local_header += (uint16_t) fname.size(); //fname length
	local_header += (uint16_t) 0; //extra field length
	local_header += fname;

	//build global header
	global_header += "PK"; //first part of sig
	global_header += (uint16_t) 0x0201; //second part of sig
	global_header += (uint16_t) 20; //version made by
	global_header.insert(global_header.end(),local_header.begin()+4,local_header.begin()+30);
	global_header += (uint16_t) 0; //file comment length
	global_header += (uint16_t) 0; //disk number where file starts
	global_header += (uint16_t) 0; //internal file attributes
	global_header += (uint32_t) 0; //external file attributes
	global_header += (uint32_t) global_header_offset; //relative offset of local file header, since it begins where the global header used to begin
	global_header += fname;

	//build footer
	std::vector<char> footer;
	footer += "PK"; //first part of sig
	footer += (uint16_t) 0x0605; //second part of sig
	footer += (uint16_t) 0; //number of this disk
	footer += (uint16_t) 0; //disk where footer starts
	footer += (uint16_t) (nrecs+1); //number of records on this disk
	footer += (uint16_t) (nrecs+1); //total number of records
	footer += (uint32_t) global_header.size(); //nbytes of global headers
	footer += (uint32_t) (global_header_offset + nbytes + local_header.size()); //offset of start of global headers, since global header now starts after newly written array
	footer += (uint16_t) 0; //zip file comment length

	//write everything
	fwrite(&local_header[0],sizeof(char),local_header.size(),fp);
	fwrite(&npy_header[0],sizeof(char),npy_header.size(),fp);
	fwrite(&data[0],wordSize*4,nels,fp);
	fwrite(&global_header[0],sizeof(char),global_header.size(),fp);
	fwrite(&footer[0],sizeof(char),footer.size(),fp);
	fclose(fp);
}
}
void NPYarray::npz_save(std::string zipname, std::string fname, TStrings const& vec, std::string mode)
{
	std::vector<int> temp;
	std::vector<size_t> shape;
	int numElts=vec.size();
	int wordSize=1;
	for(int k=0; k<numElts; k++) 
		wordSize=MAX(wordSize, vec[k].length());

	temp.resize(numElts*wordSize);
	shape.resize(1);
	shape[0]=numElts;
	for(int k=0; k<temp.size(); k++) 
		temp[k]=0;

	for(int k=0; k<numElts; k++) 
	{
		for(int j=0; j<vec[k].length(); j++)
			temp[k*wordSize+j]=(int)vec[k][j];
	}
	cnpy::npz_save_U(zipname, fname, temp, shape,mode);  
}
