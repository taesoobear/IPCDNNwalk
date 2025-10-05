/* miniz.c v1.16 beta r1 - public domain deflate/inflate, zlib-subset, ZIP reading/writing/appending, PNG writing
   See "unlicense" statement at the end of this file.
   Rich Geldreich <richgel99@gmail.com>, last updated Oct. 13, 2013
   Implements RFC 1950: http://www.ietf.org/rfc/rfc1950.txt and RFC 1951: http://www.ietf.org/rfc/rfc1951.txt

   Most API's defined in miniz.c are optional. For example, to disable the archive related functions just define
   TMINIZ_NO_ARCHIVE_APIS, or to get rid of all stdio usage define TMINIZ_NO_STDIO (see the list below for more macros).

   * Low-level Deflate/Inflate implementation notes:

     Compression: Use the "tdefl" API's. The compressor supports raw, static, and dynamic blocks, lazy or
     greedy parsing, match length filtering, RLE-only, and Huffman-only streams. It performs and compresses
     approximately as well as zlib.

     Decompression: Use the "tinfl" API's. The entire decompressor is implemented as a single function
     coroutine: see tinfl_decompress(). It supports decompression into a 32KB (or larger power of 2) wrapping buffer, or into a memory
     block large enough to hold the entire file.

     The low-level tdefl/tinfl API's do not make any use of dynamic memory allocation.

   * zlib-style API notes:

     miniz.c implements a fairly large subset of zlib. There's enough functionality present for it to be a drop-in
     zlib replacement in many apps:
        The z_stream struct, optional memory allocation callbacks
        deflateInit/deflateInit2/deflate/deflateReset/deflateEnd/deflateBound
        inflateInit/inflateInit2/inflate/inflateEnd
        compress, compress2, compressBound, uncompress
        CRC-32, Adler-32 - Using modern, minimal code size, CPU cache friendly routines.
        Supports raw deflate streams or standard zlib streams with adler-32 checking.

     Limitations:
      The callback API's are not implemented yet. No support for gzip headers or zlib static dictionaries.
      I've tried to closely emulate zlib's various flavors of stream flushing and return status codes, but
      there are no guarantees that miniz.c pulls this off perfectly.

   * PNG writing: See the tdefl_write_image_to_png_file_in_memory() function, originally written by
     Alex Evans. Supports 1-4 bytes/pixel images.

   * ZIP archive API notes:

     The ZIP archive API's where designed with simplicity and efficiency in mind, with just enough abstraction to
     get the job done with minimal fuss. There are simple API's to retrieve file information, read files from
     existing archives, create new archives, append new files to existing archives, or clone archive data from
     one archive to another. It supports archives located in memory or the heap, on disk (using stdio.h),
     or you can specify custom file read/write callbacks.

     - Archive reading: Just call this function to read a single file from a disk archive:

      void *tmz_zip_extract_archive_file_to_heap(const char *pZip_filename, const char *pArchive_name,
        size_t *pSize, tmz_uint zip_flags);

     For more complex cases, use the "tmz_zip_reader" functions. Upon opening an archive, the entire central
     directory is located and read as-is into memory, and subsequent file access only occurs when reading individual files.

     - Archives file scanning: The simple way is to use this function to scan a loaded archive for a specific file:

     int tmz_zip_reader_locate_file(tmz_zip_archive *pZip, const char *pName, const char *pComment, tmz_uint flags);

     The locate operation can optionally check file comments too, which (as one example) can be used to identify
     multiple versions of the same file in an archive. This function uses a simple linear search through the central
     directory, so it's not very fast.

     Alternately, you can iterate through all the files in an archive (using tmz_zip_reader_get_num_files()) and
     retrieve detailed info on each file by calling tmz_zip_reader_file_stat().

     - Archive creation: Use the "tmz_zip_writer" functions. The ZIP writer immediately writes compressed file data
     to disk and builds an exact image of the central directory in memory. The central directory image is written
     all at once at the end of the archive file when the archive is finalized.

     The archive writer can optionally align each file's local header and file data to any power of 2 alignment,
     which can be useful when the archive will be read from optical media. Also, the writer supports placing
     arbitrary data blobs at the very beginning of ZIP archives. Archives written using either feature are still
     readable by any ZIP tool.

     - Archive appending: The simple way to add a single file to an archive is to call this function:

      tmz_bool tmz_zip_add_mem_to_archive_file_in_place(const char *pZip_filename, const char *pArchive_name,
        const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags);

     The archive will be created if it doesn't already exist, otherwise it'll be appended to.
     Note the appending is done in-place and is not an atomic operation, so if something goes wrong
     during the operation it's possible the archive could be left without a central directory (although the local
     file headers and file data will be fine, so the archive will be recoverable).

     For more complex archive modification scenarios:
     1. The safest way is to use a tmz_zip_reader to read the existing archive, cloning only those bits you want to
     preserve into a new archive using using the tmz_zip_writer_add_from_zip_reader() function (which compiles the
     compressed file data as-is). When you're done, delete the old archive and rename the newly written archive, and
     you're done. This is safe but requires a bunch of temporary disk space or heap memory.

     2. Or, you can convert an tmz_zip_reader in-place to an tmz_zip_writer using tmz_zip_writer_init_from_reader(),
     append new files as needed, then finalize the archive which will write an updated central directory to the
     original archive. (This is basically what tmz_zip_add_mem_to_archive_file_in_place() does.) There's a
     possibility that the archive's central directory could be lost with this method if anything goes wrong, though.

     - ZIP archive support limitations:
     No zip64 or spanning support. Extraction functions can only handle unencrypted, stored or deflated files.
     Requires streams capable of seeking.

   * This is a header file library, like stb_image.c. To get only a header file, either cut and paste the
     below header, or create miniz.h, #define TMINIZ_HEADER_FILE_ONLY, and then include miniz.c from it.

   * Important: For best perf. be sure to customize the below macros for your target platform:
     #define TMINIZ_USE_UNALIGNED_LOADS_AND_STORES 1
     #define TMINIZ_LITTLE_ENDIAN 1
     #define TMINIZ_HAS_64BIT_REGISTERS 1

   * On platforms using glibc, Be sure to "#define _LARGEFILE64_SOURCE 1" before including miniz.c to ensure miniz
     uses the 64-bit variants: fopen64(), stat64(), etc. Otherwise you won't be able to process large files
     (i.e. 32-bit stat() fails for me on files > 0x7FFFFFFF bytes).
*/
#pragma once





/* Defines to completely disable specific portions of miniz.c: 
   If all macros here are defined the only functionality remaining will be CRC-32, adler-32, tinfl, and tdefl. */

/* Define TMINIZ_NO_STDIO to disable all usage and any functions which rely on stdio for file I/O. */
/*#define TMINIZ_NO_STDIO */

/* If TMINIZ_NO_TIME is specified then the ZIP archive functions will not be able to get the current time, or */
/* get/set file times, and the C run-time funcs that get/set times won't be called. */
/* The current downside is the times written to your archives will be from 1979. */
/*#define TMINIZ_NO_TIME */

/* Define TMINIZ_NO_ARCHIVE_APIS to disable all ZIP archive API's. */
/*#define TMINIZ_NO_ARCHIVE_APIS */

/* Define TMINIZ_NO_ARCHIVE_WRITING_APIS to disable all writing related ZIP archive API's. */
/*#define TMINIZ_NO_ARCHIVE_WRITING_APIS */

/* Define TMINIZ_NO_ZLIB_APIS to remove all ZLIB-style compression/decompression API's. */
/*#define TMINIZ_NO_ZLIB_APIS */

/* Define TMINIZ_NO_ZLIB_COMPATIBLE_NAME to disable zlib names, to prevent conflicts against stock zlib. */
/*#define TMINIZ_NO_ZLIB_COMPATIBLE_NAMES */

/* Define TMINIZ_NO_MALLOC to disable all calls to malloc, free, and realloc. 
   Note if TMINIZ_NO_MALLOC is defined then the user must always provide custom user alloc/free/realloc
   callbacks to the zlib and archive API's, and a few stand-alone helper API's which don't provide custom user
   functions (such as tdefl_compress_mem_to_heap() and tinfl_decompress_mem_to_heap()) won't work. */
/*#define TMINIZ_NO_MALLOC */

#if defined(__TINYC__) && (defined(__linux) || defined(__linux__))
/* TODO: Work around "error: include file 'sys\utime.h' when compiling with tcc on Linux */
#define TMINIZ_NO_TIME
#endif

#include <stddef.h>

#if !defined(TMINIZ_NO_TIME) && !defined(TMINIZ_NO_ARCHIVE_APIS)
#include <time.h>
#endif

#if defined(_M_IX86) || defined(_M_X64) || defined(__i386__) || defined(__i386) || defined(__i486__) || defined(__i486) || defined(i386) || defined(__ia64__) || defined(__x86_64__)
/* TMINIZ_X86_OR_X64_CPU is only used to help set the below macros. */
#define TMINIZ_X86_OR_X64_CPU 1
#endif

#if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) || TMINIZ_X86_OR_X64_CPU
/* Set TMINIZ_LITTLE_ENDIAN to 1 if the processor is little endian. */
#define TMINIZ_LITTLE_ENDIAN 1
#endif

#if TMINIZ_X86_OR_X64_CPU
/* Set TMINIZ_USE_UNALIGNED_LOADS_AND_STORES to 1 on CPU's that permit efficient integer loads and stores from unaligned addresses. */
#define TMINIZ_USE_UNALIGNED_LOADS_AND_STORES 1
#endif

#if defined(_M_X64) || defined(_WIN64) || defined(__MINGW64__) || defined(_LP64) || defined(__LP64__) || defined(__ia64__) || defined(__x86_64__)
/* Set TMINIZ_HAS_64BIT_REGISTERS to 1 if operations on 64-bit integers are reasonably fast (and don't involve compiler generated calls to helper functions). */
#define TMINIZ_HAS_64BIT_REGISTERS 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------- zlib-style API Definitions. */

/* For more compatibility with zlib, miniz.c uses unsigned long for some parameters/struct members. Beware: tmz_ulong can be either 32 or 64-bits! */
typedef unsigned long tmz_ulong;

/* tmz_free() internally uses the tmz_FREE() macro (which by default calls free() unless you've modified the tmz_MALLOC macro) to release a block allocated from the heap. */
void tmz_free(void *p);

#define tmz_ADLER32_INIT (1)
/* tmz_adler32() returns the initial adler-32 value to use when called with ptr==NULL. */
tmz_ulong tmz_adler32(tmz_ulong adler, const unsigned char *ptr, size_t buf_len);

#define tmz_CRC32_INIT (0)
/* tmz_crc32() returns the initial CRC-32 value to use when called with ptr==NULL. */
tmz_ulong tmz_crc32(tmz_ulong crc, const unsigned char *ptr, size_t buf_len);

/* Compression strategies. */
enum
{
    tmz_DEFAULT_STRATEGY = 0,
    tmz_FILTERED = 1,
    tmz_HUFFMAN_ONLY = 2,
    tmz_RLE = 3,
    tmz_FIXED = 4
};

/* Method */
#define tmz_DEFLATED 8

/* Heap allocation callbacks.
Note that tmz_alloc_func parameter types purpsosely differ from zlib's: items/size is size_t, not unsigned long. */
typedef void *(*tmz_alloc_func)(void *opaque, size_t items, size_t size);
typedef void(*tmz_free_func)(void *opaque, void *address);
typedef void *(*tmz_realloc_func)(void *opaque, void *address, size_t items, size_t size);

/* Compression levels: 0-9 are the standard zlib-style levels, 10 is best possible compression (not zlib compatible, and may be very slow), tmz_DEFAULT_COMPRESSION=tmz_DEFAULT_LEVEL. */
enum
{
	tmz_NO_COMPRESSION = 0,
	tmz_BEST_SPEED = 1,
	tmz_BEST_COMPRESSION = 9,
	tmz_UBER_COMPRESSION = 10,
	tmz_DEFAULT_LEVEL = 6,
	tmz_DEFAULT_COMPRESSION = -1
};

#define tmz_VERSION "10.0.0"
#define tmz_VERNUM 0xA000
#define tmz_VER_MAJOR 10
#define tmz_VER_MINOR 0
#define tmz_VER_REVISION 0
#define tmz_VER_SUBREVISION 0

#ifndef TMINIZ_NO_ZLIB_APIS

/* Flush values. For typical usage you only need tmz_NO_FLUSH and tmz_FINISH. The other values are for advanced use (refer to the zlib docs). */
enum
{
    tmz_NO_FLUSH = 0,
    tmz_PARTIAL_FLUSH = 1,
    tmz_SYNC_FLUSH = 2,
    tmz_FULL_FLUSH = 3,
    tmz_FINISH = 4,
    tmz_BLOCK = 5
};

/* Return status codes. tmz_PARAM_ERROR is non-standard. */
enum
{
    tmz_OK = 0,
    tmz_STREAM_END = 1,
    tmz_NEED_DICT = 2,
    tmz_ERRNO = -1,
    tmz_STREAM_ERROR = -2,
    tmz_DATA_ERROR = -3,
    tmz_MEM_ERROR = -4,
    tmz_BUF_ERROR = -5,
    tmz_VERSION_ERROR = -6,
    tmz_PARAM_ERROR = -10000
};

/* Window bits */
#define tmz_DEFAULT_WINDOW_BITS 15

struct tmz_internal_state;

/* Compression/decompression stream struct. */
typedef struct tmz_stream_s
{
    const unsigned char *next_in; /* pointer to next byte to read */
    unsigned int avail_in;        /* number of bytes available at next_in */
    tmz_ulong total_in;            /* total number of bytes consumed so far */

    unsigned char *next_out; /* pointer to next byte to write */
    unsigned int avail_out;  /* number of bytes that can be written to next_out */
    tmz_ulong total_out;      /* total number of bytes produced so far */

    char *msg;                       /* error msg (unused) */
    struct tmz_internal_state *state; /* internal state, allocated by zalloc/zfree */

    tmz_alloc_func zalloc; /* optional heap allocation function (defaults to malloc) */
    tmz_free_func zfree;   /* optional heap free function (defaults to free) */
    void *opaque;         /* heap alloc function user pointer */

    int data_type;     /* data_type (unused) */
    tmz_ulong adler;    /* adler32 of the source or uncompressed data */
    tmz_ulong reserved; /* not used */
} tmz_stream;

typedef tmz_stream *tmz_streamp;

/* Returns the version string of miniz.c. */
const char *tmz_version(void);

/* tmz_deflateInit() initializes a compressor with default options: */
/* Parameters: */
/*  pStream must point to an initialized tmz_stream struct. */
/*  level must be between [tmz_NO_COMPRESSION, tmz_BEST_COMPRESSION]. */
/*  level 1 enables a specially optimized compression function that's been optimized purely for performance, not ratio. */
/*  (This special func. is currently only enabled when TMINIZ_USE_UNALIGNED_LOADS_AND_STORES and TMINIZ_LITTLE_ENDIAN are defined.) */
/* Return values: */
/*  tmz_OK on success. */
/*  tmz_STREAM_ERROR if the stream is bogus. */
/*  tmz_PARAM_ERROR if the input parameters are bogus. */
/*  tmz_MEM_ERROR on out of memory. */
int tmz_deflateInit(tmz_streamp pStream, int level);

/* tmz_deflateInit2() is like tmz_deflate(), except with more control: */
/* Additional parameters: */
/*   method must be tmz_DEFLATED */
/*   window_bits must be tmz_DEFAULT_WINDOW_BITS (to wrap the deflate stream with zlib header/adler-32 footer) or -tmz_DEFAULT_WINDOW_BITS (raw deflate/no header or footer) */
/*   mem_level must be between [1, 9] (it's checked but ignored by miniz.c) */
int tmz_deflateInit2(tmz_streamp pStream, int level, int method, int window_bits, int mem_level, int strategy);

/* Quickly resets a compressor without having to reallocate anything. Same as calling tmz_deflateEnd() followed by tmz_deflateInit()/tmz_deflateInit2(). */
int tmz_deflateReset(tmz_streamp pStream);

/* tmz_deflate() compresses the input to output, consuming as much of the input and producing as much output as possible. */
/* Parameters: */
/*   pStream is the stream to read from and write to. You must initialize/update the next_in, avail_in, next_out, and avail_out members. */
/*   flush may be tmz_NO_FLUSH, tmz_PARTIAL_FLUSH/tmz_SYNC_FLUSH, tmz_FULL_FLUSH, or tmz_FINISH. */
/* Return values: */
/*   tmz_OK on success (when flushing, or if more input is needed but not available, and/or there's more output to be written but the output buffer is full). */
/*   tmz_STREAM_END if all input has been consumed and all output bytes have been written. Don't call tmz_deflate() on the stream anymore. */
/*   tmz_STREAM_ERROR if the stream is bogus. */
/*   tmz_PARAM_ERROR if one of the parameters is invalid. */
/*   tmz_BUF_ERROR if no forward progress is possible because the input and/or output buffers are empty. (Fill up the input buffer or free up some output space and try again.) */
int tmz_deflate(tmz_streamp pStream, int flush);

/* tmz_deflateEnd() deinitializes a compressor: */
/* Return values: */
/*  tmz_OK on success. */
/*  tmz_STREAM_ERROR if the stream is bogus. */
int tmz_deflateEnd(tmz_streamp pStream);

/* tmz_deflateBound() returns a (very) conservative upper bound on the amount of data that could be generated by deflate(), assuming flush is set to only tmz_NO_FLUSH or tmz_FINISH. */
tmz_ulong tmz_deflateBound(tmz_streamp pStream, tmz_ulong source_len);

/* Single-call compression functions tmz_compress() and tmz_compress2(): */
/* Returns tmz_OK on success, or one of the error codes from tmz_deflate() on failure. */
int tmz_compress(unsigned char *pDest, tmz_ulong *pDest_len, const unsigned char *pSource, tmz_ulong source_len);
int tmz_compress2(unsigned char *pDest, tmz_ulong *pDest_len, const unsigned char *pSource, tmz_ulong source_len, int level);

/* tmz_compressBound() returns a (very) conservative upper bound on the amount of data that could be generated by calling tmz_compress(). */
tmz_ulong tmz_compressBound(tmz_ulong source_len);

/* Initializes a decompressor. */
int tmz_inflateInit(tmz_streamp pStream);

/* tmz_inflateInit2() is like tmz_inflateInit() with an additional option that controls the window size and whether or not the stream has been wrapped with a zlib header/footer: */
/* window_bits must be tmz_DEFAULT_WINDOW_BITS (to parse zlib header/footer) or -tmz_DEFAULT_WINDOW_BITS (raw deflate). */
int tmz_inflateInit2(tmz_streamp pStream, int window_bits);

/* Decompresses the input stream to the output, consuming only as much of the input as needed, and writing as much to the output as possible. */
/* Parameters: */
/*   pStream is the stream to read from and write to. You must initialize/update the next_in, avail_in, next_out, and avail_out members. */
/*   flush may be tmz_NO_FLUSH, tmz_SYNC_FLUSH, or tmz_FINISH. */
/*   On the first call, if flush is tmz_FINISH it's assumed the input and output buffers are both sized large enough to decompress the entire stream in a single call (this is slightly faster). */
/*   tmz_FINISH implies that there are no more source bytes available beside what's already in the input buffer, and that the output buffer is large enough to hold the rest of the decompressed data. */
/* Return values: */
/*   tmz_OK on success. Either more input is needed but not available, and/or there's more output to be written but the output buffer is full. */
/*   tmz_STREAM_END if all needed input has been consumed and all output bytes have been written. For zlib streams, the adler-32 of the decompressed data has also been verified. */
/*   tmz_STREAM_ERROR if the stream is bogus. */
/*   tmz_DATA_ERROR if the deflate stream is invalid. */
/*   tmz_PARAM_ERROR if one of the parameters is invalid. */
/*   tmz_BUF_ERROR if no forward progress is possible because the input buffer is empty but the inflater needs more input to continue, or if the output buffer is not large enough. Call tmz_inflate() again */
/*   with more input data, or with more room in the output buffer (except when using single call decompression, described above). */
int tmz_inflate(tmz_streamp pStream, int flush);

/* Deinitializes a decompressor. */
int tmz_inflateEnd(tmz_streamp pStream);

/* Single-call decompression. */
/* Returns tmz_OK on success, or one of the error codes from tmz_inflate() on failure. */
int tmz_uncompress(unsigned char *pDest, tmz_ulong *pDest_len, const unsigned char *pSource, tmz_ulong source_len);

/* Returns a string description of the specified error code, or NULL if the error code is invalid. */
const char *tmz_error(int err);

/* Redefine zlib-compatible names to miniz equivalents, so miniz.c can be used as a drop-in replacement for the subset of zlib that miniz.c supports. */
/* Define TMINIZ_NO_ZLIB_COMPATIBLE_NAMES to disable zlib-compatibility if you use zlib in the same project. */
#ifndef TMINIZ_NO_ZLIB_COMPATIBLE_NAMES
typedef unsigned char Byte;
typedef unsigned int uInt;
typedef tmz_ulong uLong;
typedef Byte Bytef;
typedef uInt uIntf;
typedef char charf;
typedef int intf;
typedef void *voidpf;
typedef uLong uLongf;
typedef void *voidp;
typedef void *const voidpc;
#define Z_NULL 0
#define Z_NO_FLUSH tmz_NO_FLUSH
#define Z_PARTIAL_FLUSH tmz_PARTIAL_FLUSH
#define Z_SYNC_FLUSH tmz_SYNC_FLUSH
#define Z_FULL_FLUSH tmz_FULL_FLUSH
#define Z_FINISH tmz_FINISH
#define Z_BLOCK tmz_BLOCK
#define Z_OK tmz_OK
#define Z_STREAM_END tmz_STREAM_END
#define Z_NEED_DICT tmz_NEED_DICT
#define Z_ERRNO tmz_ERRNO
#define Z_STREAM_ERROR tmz_STREAM_ERROR
#define Z_DATA_ERROR tmz_DATA_ERROR
#define Z_MEM_ERROR tmz_MEM_ERROR
#define Z_BUF_ERROR tmz_BUF_ERROR
#define Z_VERSION_ERROR tmz_VERSION_ERROR
#define Z_PARAM_ERROR tmz_PARAM_ERROR
#define Z_NO_COMPRESSION tmz_NO_COMPRESSION
#define Z_BEST_SPEED tmz_BEST_SPEED
#define Z_BEST_COMPRESSION tmz_BEST_COMPRESSION
#define Z_DEFAULT_COMPRESSION tmz_DEFAULT_COMPRESSION
#define Z_DEFAULT_STRATEGY tmz_DEFAULT_STRATEGY
#define Z_FILTERED tmz_FILTERED
#define Z_HUFFMAN_ONLY tmz_HUFFMAN_ONLY
#define Z_RLE tmz_RLE
#define Z_FIXED tmz_FIXED
#define Z_DEFLATED tmz_DEFLATED
#define Z_DEFAULT_WINDOW_BITS tmz_DEFAULT_WINDOW_BITS
#define alloc_func tmz_alloc_func
#define free_func tmz_free_func
#define internal_state tmz_internal_state
#define z_stream tmz_stream
#define deflateInit tmz_deflateInit
#define deflateInit2 tmz_deflateInit2
#define deflateReset tmz_deflateReset
#define deflate tmz_deflate
#define deflateEnd tmz_deflateEnd
#define deflateBound tmz_deflateBound
#define compress tmz_compress
#define compress2 tmz_compress2
#define compressBound tmz_compressBound
#define inflateInit tmz_inflateInit
#define inflateInit2 tmz_inflateInit2
#define inflate tmz_inflate
#define inflateEnd tmz_inflateEnd
#define uncompress tmz_uncompress
#define crc32 tmz_crc32
#define adler32 tmz_adler32
#define MAX_WBITS 15
#define MAX_MEM_LEVEL 9
#define zError tmz_error
#define ZLIB_VERSION tmz_VERSION
#define ZLIB_VERNUM tmz_VERNUM
#define ZLIB_VER_MAJOR tmz_VER_MAJOR
#define ZLIB_VER_MINOR tmz_VER_MINOR
#define ZLIB_VER_REVISION tmz_VER_REVISION
#define ZLIB_VER_SUBREVISION tmz_VER_SUBREVISION
#define zlibVersion tmz_version
#define zlib_version tmz_version()
#endif /* #ifndef TMINIZ_NO_ZLIB_COMPATIBLE_NAMES */

#endif /* TMINIZ_NO_ZLIB_APIS */

#ifdef __cplusplus
}
#endif
#pragma once
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ------------------- Types and macros */
typedef unsigned char tmz_uint8;
typedef signed short tmz_int16;
typedef unsigned short tmz_uint16;
typedef unsigned int tmz_uint32;
typedef unsigned int tmz_uint;
typedef int64_t tmz_int64;
typedef uint64_t tmz_uint64;
typedef int tmz_bool;

#define tmz_FALSE (0)
#define tmz_TRUE (1)

/* Works around MSVC's spammy "warning C4127: conditional expression is constant" message. */
#ifdef _MSC_VER
#define tmz_MACRO_END while (0, 0)
#else
#define tmz_MACRO_END while (0)
#endif

#ifdef TMINIZ_NO_STDIO
#define tmz_FILE void *
#else
#include <stdio.h>
#define tmz_FILE FILE
#endif /* #ifdef TMINIZ_NO_STDIO */

#ifdef TMINIZ_NO_TIME
typedef struct tmz_dummy_time_t_tag
{
    int m_dummy;
} tmz_dummy_time_t;
#define tmz_TIME_T tmz_dummy_time_t
#else
#define tmz_TIME_T time_t
#endif

#define tmz_ASSERT(x) assert(x)

#ifdef TMINIZ_NO_MALLOC
#define tmz_MALLOC(x) NULL
#define tmz_FREE(x) (void) x, ((void)0)
#define tmz_REALLOC(p, x) NULL
#else
#define tmz_MALLOC(x) malloc(x)
#define tmz_FREE(x) free(x)
#define tmz_REALLOC(p, x) realloc(p, x)
#endif

#define tmz_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define tmz_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define tmz_CLEAR_OBJ(obj) memset(&(obj), 0, sizeof(obj))

#if TMINIZ_USE_UNALIGNED_LOADS_AND_STORES &&TMINIZ_LITTLE_ENDIAN
#define tmz_READ_LE16(p) *((const tmz_uint16 *)(p))
#define tmz_READ_LE32(p) *((const tmz_uint32 *)(p))
#else
#define tmz_READ_LE16(p) ((tmz_uint32)(((const tmz_uint8 *)(p))[0]) | ((tmz_uint32)(((const tmz_uint8 *)(p))[1]) << 8U))
#define tmz_READ_LE32(p) ((tmz_uint32)(((const tmz_uint8 *)(p))[0]) | ((tmz_uint32)(((const tmz_uint8 *)(p))[1]) << 8U) | ((tmz_uint32)(((const tmz_uint8 *)(p))[2]) << 16U) | ((tmz_uint32)(((const tmz_uint8 *)(p))[3]) << 24U))
#endif

#define tmz_READ_LE64(p) (((tmz_uint64)tmz_READ_LE32(p)) | (((tmz_uint64)tmz_READ_LE32((const tmz_uint8 *)(p) + sizeof(tmz_uint32))) << 32U))

#ifdef _MSC_VER
#define tmz_FORCEINLINE __forceinline
#elif defined(__GNUC__)
#define tmz_FORCEINLINE __inline__ __attribute__((__always_inline__))
#else
#define tmz_FORCEINLINE inline
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern void *tminiz_def_alloc_func(void *opaque, size_t items, size_t size);
extern void tminiz_def_free_func(void *opaque, void *address);
extern void *tminiz_def_realloc_func(void *opaque, void *address, size_t items, size_t size);

#define tmz_UINT16_MAX (0xFFFFU)
#define tmz_UINT32_MAX (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#pragma once


#ifdef __cplusplus
extern "C" {
#endif
/* ------------------- Low-level Compression API Definitions */

/* Set TDEFL_LESS_MEMORY to 1 to use less memory (compression will be slightly slower, and raw/dynamic blocks will be output more frequently). */
#define TDEFL_LESS_MEMORY 0

/* tdefl_init() compression flags logically OR'd together (low 12 bits contain the max. number of probes per dictionary search): */
/* TDEFL_DEFAULT_MAX_PROBES: The compressor defaults to 128 dictionary probes per dictionary search. 0=Huffman only, 1=Huffman+LZ (fastest/crap compression), 4095=Huffman+LZ (slowest/best compression). */
enum
{
    TDEFL_HUFFMAN_ONLY = 0,
    TDEFL_DEFAULT_MAX_PROBES = 128,
    TDEFL_MAX_PROBES_MASK = 0xFFF
};

/* TDEFL_WRITE_ZLIB_HEADER: If set, the compressor outputs a zlib header before the deflate data, and the Adler-32 of the source data at the end. Otherwise, you'll get raw deflate data. */
/* TDEFL_COMPUTE_ADLER32: Always compute the adler-32 of the input data (even when not writing zlib headers). */
/* TDEFL_GREEDY_PARSING_FLAG: Set to use faster greedy parsing, instead of more efficient lazy parsing. */
/* TDEFL_NONDETERMINISTIC_PARSING_FLAG: Enable to decrease the compressor's initialization time to the minimum, but the output may vary from run to run given the same input (depending on the contents of memory). */
/* TDEFL_RLE_MATCHES: Only look for RLE matches (matches with a distance of 1) */
/* TDEFL_FILTER_MATCHES: Discards matches <= 5 chars if enabled. */
/* TDEFL_FORCE_ALL_STATIC_BLOCKS: Disable usage of optimized Huffman tables. */
/* TDEFL_FORCE_ALL_RAW_BLOCKS: Only use raw (uncompressed) deflate blocks. */
/* The low 12 bits are reserved to control the max # of hash probes per dictionary lookup (see TDEFL_MAX_PROBES_MASK). */
enum
{
    TDEFL_WRITE_ZLIB_HEADER = 0x01000,
    TDEFL_COMPUTE_ADLER32 = 0x02000,
    TDEFL_GREEDY_PARSING_FLAG = 0x04000,
    TDEFL_NONDETERMINISTIC_PARSING_FLAG = 0x08000,
    TDEFL_RLE_MATCHES = 0x10000,
    TDEFL_FILTER_MATCHES = 0x20000,
    TDEFL_FORCE_ALL_STATIC_BLOCKS = 0x40000,
    TDEFL_FORCE_ALL_RAW_BLOCKS = 0x80000
};

/* High level compression functions: */
/* tdefl_compress_mem_to_heap() compresses a block in memory to a heap block allocated via malloc(). */
/* On entry: */
/*  pSrc_buf, src_buf_len: Pointer and size of source block to compress. */
/*  flags: The max match finder probes (default is 128) logically OR'd against the above flags. Higher probes are slower but improve compression. */
/* On return: */
/*  Function returns a pointer to the compressed data, or NULL on failure. */
/*  *pOut_len will be set to the compressed data's size, which could be larger than src_buf_len on uncompressible data. */
/*  The caller must free() the returned block when it's no longer needed. */
void *tdefl_compress_mem_to_heap(const void *pSrc_buf, size_t src_buf_len, size_t *pOut_len, int flags);

/* tdefl_compress_mem_to_mem() compresses a block in memory to another block in memory. */
/* Returns 0 on failure. */
size_t tdefl_compress_mem_to_mem(void *pOut_buf, size_t out_buf_len, const void *pSrc_buf, size_t src_buf_len, int flags);

/* Compresses an image to a compressed PNG file in memory. */
/* On entry: */
/*  pImage, w, h, and num_chans describe the image to compress. num_chans may be 1, 2, 3, or 4. */
/*  The image pitch in bytes per scanline will be w*num_chans. The leftmost pixel on the top scanline is stored first in memory. */
/*  level may range from [0,10], use tmz_NO_COMPRESSION, tmz_BEST_SPEED, tmz_BEST_COMPRESSION, etc. or a decent default is tmz_DEFAULT_LEVEL */
/*  If flip is true, the image will be flipped on the Y axis (useful for OpenGL apps). */
/* On return: */
/*  Function returns a pointer to the compressed data, or NULL on failure. */
/*  *pLen_out will be set to the size of the PNG image file. */
/*  The caller must tmz_free() the returned heap block (which will typically be larger than *pLen_out) when it's no longer needed. */
void *tdefl_write_image_to_png_file_in_memory_ex(const void *pImage, int w, int h, int num_chans, size_t *pLen_out, tmz_uint level, tmz_bool flip);
void *tdefl_write_image_to_png_file_in_memory(const void *pImage, int w, int h, int num_chans, size_t *pLen_out);

/* Output stream interface. The compressor uses this interface to write compressed data. It'll typically be called TDEFL_OUT_BUF_SIZE at a time. */
typedef tmz_bool (*tdefl_put_buf_func_ptr)(const void *pBuf, int len, void *pUser);

/* tdefl_compress_mem_to_output() compresses a block to an output stream. The above helpers use this function internally. */
tmz_bool tdefl_compress_mem_to_output(const void *pBuf, size_t buf_len, tdefl_put_buf_func_ptr pPut_buf_func, void *pPut_buf_user, int flags);

enum
{
    TDEFL_MAX_HUFF_TABLES = 3,
    TDEFL_MAX_HUFF_SYMBOLS_0 = 288,
    TDEFL_MAX_HUFF_SYMBOLS_1 = 32,
    TDEFL_MAX_HUFF_SYMBOLS_2 = 19,
    TDEFL_LZ_DICT_SIZE = 32768,
    TDEFL_LZ_DICT_SIZE_MASK = TDEFL_LZ_DICT_SIZE - 1,
    TDEFL_MIN_MATCH_LEN = 3,
    TDEFL_MAX_MATCH_LEN = 258
};

/* TDEFL_OUT_BUF_SIZE MUST be large enough to hold a single entire compressed output block (using static/fixed Huffman codes). */
#if TDEFL_LESS_MEMORY
enum
{
    TDEFL_LZ_CODE_BUF_SIZE = 24 * 1024,
    TDEFL_OUT_BUF_SIZE = (TDEFL_LZ_CODE_BUF_SIZE * 13) / 10,
    TDEFL_MAX_HUFF_SYMBOLS = 288,
    TDEFL_LZ_HASH_BITS = 12,
    TDEFL_LEVEL1_HASH_SIZE_MASK = 4095,
    TDEFL_LZ_HASH_SHIFT = (TDEFL_LZ_HASH_BITS + 2) / 3,
    TDEFL_LZ_HASH_SIZE = 1 << TDEFL_LZ_HASH_BITS
};
#else
enum
{
    TDEFL_LZ_CODE_BUF_SIZE = 64 * 1024,
    TDEFL_OUT_BUF_SIZE = (TDEFL_LZ_CODE_BUF_SIZE * 13) / 10,
    TDEFL_MAX_HUFF_SYMBOLS = 288,
    TDEFL_LZ_HASH_BITS = 15,
    TDEFL_LEVEL1_HASH_SIZE_MASK = 4095,
    TDEFL_LZ_HASH_SHIFT = (TDEFL_LZ_HASH_BITS + 2) / 3,
    TDEFL_LZ_HASH_SIZE = 1 << TDEFL_LZ_HASH_BITS
};
#endif

/* The low-level tdefl functions below may be used directly if the above helper functions aren't flexible enough. The low-level functions don't make any heap allocations, unlike the above helper functions. */
typedef enum
{
    TDEFL_STATUS_BAD_PARAM = -2,
    TDEFL_STATUS_PUT_BUF_FAILED = -1,
    TDEFL_STATUS_OKAY = 0,
    TDEFL_STATUS_DONE = 1
} tdefl_status;

/* Must map to tmz_NO_FLUSH, tmz_SYNC_FLUSH, etc. enums */
typedef enum
{
    TDEFL_NO_FLUSH = 0,
    TDEFL_SYNC_FLUSH = 2,
    TDEFL_FULL_FLUSH = 3,
    TDEFL_FINISH = 4
} tdefl_flush;

/* tdefl's compression state structure. */
typedef struct
{
    tdefl_put_buf_func_ptr m_pPut_buf_func;
    void *m_pPut_buf_user;
    tmz_uint m_flags, m_max_probes[2];
    int m_greedy_parsing;
    tmz_uint m_adler32, m_lookahead_pos, m_lookahead_size, m_dict_size;
    tmz_uint8 *m_pLZ_code_buf, *m_pLZ_flags, *m_pOutput_buf, *m_pOutput_buf_end;
    tmz_uint m_num_flags_left, m_total_lz_bytes, m_lz_code_buf_dict_pos, m_bits_in, m_bit_buffer;
    tmz_uint m_saved_match_dist, m_saved_match_len, m_saved_lit, m_output_flush_ofs, m_output_flush_remaining, m_finished, m_block_index, m_wants_to_finish;
    tdefl_status m_prev_return_status;
    const void *m_pIn_buf;
    void *m_pOut_buf;
    size_t *m_pIn_buf_size, *m_pOut_buf_size;
    tdefl_flush m_flush;
    const tmz_uint8 *m_pSrc;
    size_t m_src_buf_left, m_out_buf_ofs;
    tmz_uint8 m_dict[TDEFL_LZ_DICT_SIZE + TDEFL_MAX_MATCH_LEN - 1];
    tmz_uint16 m_huff_count[TDEFL_MAX_HUFF_TABLES][TDEFL_MAX_HUFF_SYMBOLS];
    tmz_uint16 m_huff_codes[TDEFL_MAX_HUFF_TABLES][TDEFL_MAX_HUFF_SYMBOLS];
    tmz_uint8 m_huff_code_sizes[TDEFL_MAX_HUFF_TABLES][TDEFL_MAX_HUFF_SYMBOLS];
    tmz_uint8 m_lz_code_buf[TDEFL_LZ_CODE_BUF_SIZE];
    tmz_uint16 m_next[TDEFL_LZ_DICT_SIZE];
    tmz_uint16 m_hash[TDEFL_LZ_HASH_SIZE];
    tmz_uint8 m_output_buf[TDEFL_OUT_BUF_SIZE];
} tdefl_compressor;

/* Initializes the compressor. */
/* There is no corresponding deinit() function because the tdefl API's do not dynamically allocate memory. */
/* pBut_buf_func: If NULL, output data will be supplied to the specified callback. In this case, the user should call the tdefl_compress_buffer() API for compression. */
/* If pBut_buf_func is NULL the user should always call the tdefl_compress() API. */
/* flags: See the above enums (TDEFL_HUFFMAN_ONLY, TDEFL_WRITE_ZLIB_HEADER, etc.) */
tdefl_status tdefl_init(tdefl_compressor *d, tdefl_put_buf_func_ptr pPut_buf_func, void *pPut_buf_user, int flags);

/* Compresses a block of data, consuming as much of the specified input buffer as possible, and writing as much compressed data to the specified output buffer as possible. */
tdefl_status tdefl_compress(tdefl_compressor *d, const void *pIn_buf, size_t *pIn_buf_size, void *pOut_buf, size_t *pOut_buf_size, tdefl_flush flush);

/* tdefl_compress_buffer() is only usable when the tdefl_init() is called with a non-NULL tdefl_put_buf_func_ptr. */
/* tdefl_compress_buffer() always consumes the entire input buffer. */
tdefl_status tdefl_compress_buffer(tdefl_compressor *d, const void *pIn_buf, size_t in_buf_size, tdefl_flush flush);

tdefl_status tdefl_get_prev_return_status(tdefl_compressor *d);
tmz_uint32 tdefl_get_adler32(tdefl_compressor *d);

/* Create tdefl_compress() flags given zlib-style compression parameters. */
/* level may range from [0,10] (where 10 is absolute max compression, but may be much slower on some files) */
/* window_bits may be -15 (raw deflate) or 15 (zlib) */
/* strategy may be either tmz_DEFAULT_STRATEGY, tmz_FILTERED, tmz_HUFFMAN_ONLY, tmz_RLE, or tmz_FIXED */
tmz_uint tdefl_create_comp_flags_from_zip_params(int level, int window_bits, int strategy);

/* Allocate the tdefl_compressor structure in C so that */
/* non-C language bindings to tdefl_ API don't need to worry about */
/* structure size and allocation mechanism. */
tdefl_compressor *tdefl_compressor_alloc();
void tdefl_compressor_free(tdefl_compressor *pComp);

#ifdef __cplusplus
}
#endif
#pragma once

/* ------------------- Low-level Decompression API Definitions */

#ifdef __cplusplus
extern "C" {
#endif
/* Decompression flags used by tinfl_decompress(). */
/* TINFL_FLAG_PARSE_ZLIB_HEADER: If set, the input has a valid zlib header and ends with an adler32 checksum (it's a valid zlib stream). Otherwise, the input is a raw deflate stream. */
/* TINFL_FLAG_HAS_MORE_INPUT: If set, there are more input bytes available beyond the end of the supplied input buffer. If clear, the input buffer contains all remaining input. */
/* TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF: If set, the output buffer is large enough to hold the entire decompressed stream. If clear, the output buffer is at least the size of the dictionary (typically 32KB). */
/* TINFL_FLAG_COMPUTE_ADLER32: Force adler-32 checksum computation of the decompressed bytes. */
enum
{
    TINFL_FLAG_PARSE_ZLIB_HEADER = 1,
    TINFL_FLAG_HAS_MORE_INPUT = 2,
    TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF = 4,
    TINFL_FLAG_COMPUTE_ADLER32 = 8
};

/* High level decompression functions: */
/* tinfl_decompress_mem_to_heap() decompresses a block in memory to a heap block allocated via malloc(). */
/* On entry: */
/*  pSrc_buf, src_buf_len: Pointer and size of the Deflate or zlib source data to decompress. */
/* On return: */
/*  Function returns a pointer to the decompressed data, or NULL on failure. */
/*  *pOut_len will be set to the decompressed data's size, which could be larger than src_buf_len on uncompressible data. */
/*  The caller must call tmz_free() on the returned block when it's no longer needed. */
void *tinfl_decompress_mem_to_heap(const void *pSrc_buf, size_t src_buf_len, size_t *pOut_len, int flags);

/* tinfl_decompress_mem_to_mem() decompresses a block in memory to another block in memory. */
/* Returns TINFL_DECOMPRESS_MEM_TO_MEM_FAILED on failure, or the number of bytes written on success. */
#define TINFL_DECOMPRESS_MEM_TO_MEM_FAILED ((size_t)(-1))
size_t tinfl_decompress_mem_to_mem(void *pOut_buf, size_t out_buf_len, const void *pSrc_buf, size_t src_buf_len, int flags);

/* tinfl_decompress_mem_to_callback() decompresses a block in memory to an internal 32KB buffer, and a user provided callback function will be called to flush the buffer. */
/* Returns 1 on success or 0 on failure. */
typedef int (*tinfl_put_buf_func_ptr)(const void *pBuf, int len, void *pUser);
int tinfl_decompress_mem_to_callback(const void *pIn_buf, size_t *pIn_buf_size, tinfl_put_buf_func_ptr pPut_buf_func, void *pPut_buf_user, int flags);

struct tinfl_decompressor_tag;
typedef struct tinfl_decompressor_tag tinfl_decompressor;

/* Allocate the tinfl_decompressor structure in C so that */
/* non-C language bindings to tinfl_ API don't need to worry about */
/* structure size and allocation mechanism. */

tinfl_decompressor *tinfl_decompressor_alloc();
void tinfl_decompressor_free(tinfl_decompressor *pDecomp);

/* Max size of LZ dictionary. */
#define TINFL_LZ_DICT_SIZE 32768

/* Return status. */
typedef enum
{
    /* This flags indicates the inflator needs 1 or more input bytes to make forward progress, but the caller is indicating that no more are available. The compressed data */
    /* is probably corrupted. If you call the inflator again with more bytes it'll try to continue processing the input but this is a BAD sign (either the data is corrupted or you called it incorrectly). */
    /* If you call it again with no input you'll just get TINFL_STATUS_FAILED_CANNOT_MAKE_PROGRESS again. */
    TINFL_STATUS_FAILED_CANNOT_MAKE_PROGRESS = -4,

    /* This flag indicates that one or more of the input parameters was obviously bogus. (You can try calling it again, but if you get this error the calling code is wrong.) */
    TINFL_STATUS_BAD_PARAM = -3,

    /* This flags indicate the inflator is finished but the adler32 check of the uncompressed data didn't match. If you call it again it'll return TINFL_STATUS_DONE. */
    TINFL_STATUS_ADLER32_MISMATCH = -2,

    /* This flags indicate the inflator has somehow failed (bad code, corrupted input, etc.). If you call it again without resetting via tinfl_init() it it'll just keep on returning the same status failure code. */
    TINFL_STATUS_FAILED = -1,

    /* Any status code less than TINFL_STATUS_DONE must indicate a failure. */

    /* This flag indicates the inflator has returned every byte of uncompressed data that it can, has consumed every byte that it needed, has successfully reached the end of the deflate stream, and */
    /* if zlib headers and adler32 checking enabled that it has successfully checked the uncompressed data's adler32. If you call it again you'll just get TINFL_STATUS_DONE over and over again. */
    TINFL_STATUS_DONE = 0,

    /* This flag indicates the inflator MUST have more input data (even 1 byte) before it can make any more forward progress, or you need to clear the TINFL_FLAG_HAS_MORE_INPUT */
    /* flag on the next call if you don't have any more source data. If the source data was somehow corrupted it's also possible (but unlikely) for the inflator to keep on demanding input to */
    /* proceed, so be sure to properly set the TINFL_FLAG_HAS_MORE_INPUT flag. */
    TINFL_STATUS_NEEDS_MORE_INPUT = 1,

    /* This flag indicates the inflator definitely has 1 or more bytes of uncompressed data available, but it cannot write this data into the output buffer. */
    /* Note if the source compressed data was corrupted it's possible for the inflator to return a lot of uncompressed data to the caller. I've been assuming you know how much uncompressed data to expect */
    /* (either exact or worst case) and will stop calling the inflator and fail after receiving too much. In pure streaming scenarios where you have no idea how many bytes to expect this may not be possible */
    /* so I may need to add some code to address this. */
    TINFL_STATUS_HAS_MORE_OUTPUT = 2
} tinfl_status;

/* Initializes the decompressor to its initial state. */
#define tinfl_init(r)     \
    do                    \
    {                     \
        (r)->m_state = 0; \
    }                     \
    tmz_MACRO_END
#define tinfl_get_adler32(r) (r)->m_check_adler32

/* Main low-level decompressor coroutine function. This is the only function actually needed for decompression. All the other functions are just high-level helpers for improved usability. */
/* This is a universal API, i.e. it can be used as a building block to build any desired higher level decompression API. In the limit case, it can be called once per every byte input or output. */
tinfl_status tinfl_decompress(tinfl_decompressor *r, const tmz_uint8 *pIn_buf_next, size_t *pIn_buf_size, tmz_uint8 *pOut_buf_start, tmz_uint8 *pOut_buf_next, size_t *pOut_buf_size, const tmz_uint32 decomp_flags);

/* Internal/private bits follow. */
enum
{
    TINFL_MAX_HUFF_TABLES = 3,
    TINFL_MAX_HUFF_SYMBOLS_0 = 288,
    TINFL_MAX_HUFF_SYMBOLS_1 = 32,
    TINFL_MAX_HUFF_SYMBOLS_2 = 19,
    TINFL_FAST_LOOKUP_BITS = 10,
    TINFL_FAST_LOOKUP_SIZE = 1 << TINFL_FAST_LOOKUP_BITS
};

typedef struct
{
    tmz_uint8 m_code_size[TINFL_MAX_HUFF_SYMBOLS_0];
    tmz_int16 m_look_up[TINFL_FAST_LOOKUP_SIZE], m_tree[TINFL_MAX_HUFF_SYMBOLS_0 * 2];
} tinfl_huff_table;

#if TMINIZ_HAS_64BIT_REGISTERS
#define TINFL_USE_64BIT_BITBUF 1
#endif

#if TINFL_USE_64BIT_BITBUF
typedef tmz_uint64 tinfl_bit_buf_t;
#define TINFL_BITBUF_SIZE (64)
#else
typedef tmz_uint32 tinfl_bit_buf_t;
#define TINFL_BITBUF_SIZE (32)
#endif

struct tinfl_decompressor_tag
{
    tmz_uint32 m_state, m_num_bits, m_zhdr0, m_zhdr1, m_z_adler32, m_final, m_type, m_check_adler32, m_dist, m_counter, m_num_extra, m_table_sizes[TINFL_MAX_HUFF_TABLES];
    tinfl_bit_buf_t m_bit_buf;
    size_t m_dist_from_out_buf_start;
    tinfl_huff_table m_tables[TINFL_MAX_HUFF_TABLES];
    tmz_uint8 m_raw_header[4], m_len_codes[TINFL_MAX_HUFF_SYMBOLS_0 + TINFL_MAX_HUFF_SYMBOLS_1 + 137];
};

#ifdef __cplusplus
}
#endif

#pragma once


/* ------------------- ZIP archive reading/writing */

#ifndef TMINIZ_NO_ARCHIVE_APIS

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    /* Note: These enums can be reduced as needed to save memory or stack space - they are pretty conservative. */
    tmz_ZIP_MAX_IO_BUF_SIZE = 64 * 1024,
    tmz_ZIP_MAX_ARCHIVE_FILENAME_SIZE = 512,
    tmz_ZIP_MAX_ARCHIVE_FILE_COMMENT_SIZE = 512
};

typedef struct
{
    /* Central directory file index. */
    tmz_uint32 m_file_index;

    /* Byte offset of this entry in the archive's central directory. Note we currently only support up to UINT_MAX or less bytes in the central dir. */
    tmz_uint64 m_central_dir_ofs;

    /* These fields are copied directly from the zip's central dir. */
    tmz_uint16 m_version_made_by;
    tmz_uint16 m_version_needed;
    tmz_uint16 m_bit_flag;
    tmz_uint16 m_method;

#ifndef TMINIZ_NO_TIME
    tmz_TIME_T m_time;
#endif

    /* CRC-32 of uncompressed data. */
    tmz_uint32 m_crc32;

    /* File's compressed size. */
    tmz_uint64 m_comp_size;

    /* File's uncompressed size. Note, I've seen some old archives where directory entries had 512 bytes for their uncompressed sizes, but when you try to unpack them you actually get 0 bytes. */
    tmz_uint64 m_uncomp_size;

    /* Zip internal and external file attributes. */
    tmz_uint16 m_internal_attr;
    tmz_uint32 m_external_attr;

    /* Entry's local header file offset in bytes. */
    tmz_uint64 m_local_header_ofs;

    /* Size of comment in bytes. */
    tmz_uint32 m_comment_size;

    /* tmz_TRUE if the entry appears to be a directory. */
    tmz_bool m_is_directory;

    /* tmz_TRUE if the entry uses encryption/strong encryption (which tminiz_zip doesn't support) */
    tmz_bool m_is_encrypted;

    /* tmz_TRUE if the file is not encrypted, a patch file, and if it uses a compression method we support. */
    tmz_bool m_is_supported;

    /* Filename. If string ends in '/' it's a subdirectory entry. */
    /* Guaranteed to be zero terminated, may be truncated to fit. */
    char m_filename[tmz_ZIP_MAX_ARCHIVE_FILENAME_SIZE];

    /* Comment field. */
    /* Guaranteed to be zero terminated, may be truncated to fit. */
    char m_comment[tmz_ZIP_MAX_ARCHIVE_FILE_COMMENT_SIZE];

} tmz_zip_archive_file_stat;

typedef size_t (*tmz_file_read_func)(void *pOpaque, tmz_uint64 file_ofs, void *pBuf, size_t n);
typedef size_t (*tmz_file_write_func)(void *pOpaque, tmz_uint64 file_ofs, const void *pBuf, size_t n);
typedef tmz_bool (*tmz_file_needs_keepalive)(void *pOpaque);

struct tmz_zip_internal_state_tag;
typedef struct tmz_zip_internal_state_tag tmz_zip_internal_state;

typedef enum
{
    tmz_ZIP_MODE_INVALID = 0,
    tmz_ZIP_MODE_READING = 1,
    tmz_ZIP_MODE_WRITING = 2,
    tmz_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED = 3
} tmz_zip_mode;

typedef enum
{
    tmz_ZIP_FLAG_CASE_SENSITIVE = 0x0100,
    tmz_ZIP_FLAG_IGNORE_PATH = 0x0200,
    tmz_ZIP_FLAG_COMPRESSED_DATA = 0x0400,
    tmz_ZIP_FLAG_DO_NOT_SORT_CENTRAL_DIRECTORY = 0x0800,
    tmz_ZIP_FLAG_VALIDATE_LOCATE_FILE_FLAG = 0x1000, /* if enabled, tmz_zip_reader_locate_file() will be called on each file as its validated to ensure the func finds the file in the central dir (intended for testing) */
    tmz_ZIP_FLAG_VALIDATE_HEADERS_ONLY = 0x2000,     /* validate the local headers, but don't decompress the entire file and check the crc32 */
    tmz_ZIP_FLAG_WRITE_ZIP64 = 0x4000,               /* use the zip64 file format, instead of the original zip file format */
    tmz_ZIP_FLAG_WRITE_ALLOW_READING = 0x8000,
    tmz_ZIP_FLAG_ASCII_FILENAME = 0x10000
} tmz_zip_flags;

typedef enum
{
    tmz_ZIP_TYPE_INVALID = 0,
    tmz_ZIP_TYPE_USER,
    tmz_ZIP_TYPE_MEMORY,
    tmz_ZIP_TYPE_HEAP,
    tmz_ZIP_TYPE_FILE,
    tmz_ZIP_TYPE_CFILE,
    tmz_ZIP_TOTAL_TYPES
} tmz_zip_type;

/* miniz error codes. Be sure to update tmz_zip_get_error_string() if you add or modify this enum. */
typedef enum
{
    tmz_ZIP_NO_ERROR = 0,
    tmz_ZIP_UNDEFINED_ERROR,
    tmz_ZIP_TOO_MANY_FILES,
    tmz_ZIP_FILE_TOO_LARGE,
    tmz_ZIP_UNSUPPORTED_METHOD,
    tmz_ZIP_UNSUPPORTED_ENCRYPTION,
    tmz_ZIP_UNSUPPORTED_FEATURE,
    tmz_ZIP_FAILED_FINDING_CENTRAL_DIR,
    tmz_ZIP_NOT_AN_ARCHIVE,
    tmz_ZIP_INVALID_HEADER_OR_CORRUPTED,
    tmz_ZIP_UNSUPPORTED_MULTIDISK,
    tmz_ZIP_DECOMPRESSION_FAILED,
    tmz_ZIP_COMPRESSION_FAILED,
    tmz_ZIP_UNEXPECTED_DECOMPRESSED_SIZE,
    tmz_ZIP_CRC_CHECK_FAILED,
    tmz_ZIP_UNSUPPORTED_CDIR_SIZE,
    tmz_ZIP_ALLOC_FAILED,
    tmz_ZIP_FILE_OPEN_FAILED,
    tmz_ZIP_FILE_CREATE_FAILED,
    tmz_ZIP_FILE_WRITE_FAILED,
    tmz_ZIP_FILE_READ_FAILED,
    tmz_ZIP_FILE_CLOSE_FAILED,
    tmz_ZIP_FILE_SEEK_FAILED,
    tmz_ZIP_FILE_STAT_FAILED,
    tmz_ZIP_INVALID_PARAMETER,
    tmz_ZIP_INVALID_FILENAME,
    tmz_ZIP_BUF_TOO_SMALL,
    tmz_ZIP_INTERNAL_ERROR,
    tmz_ZIP_FILE_NOT_FOUND,
    tmz_ZIP_ARCHIVE_TOO_LARGE,
    tmz_ZIP_VALIDATION_FAILED,
    tmz_ZIP_WRITE_CALLBACK_FAILED,
    tmz_ZIP_TOTAL_ERRORS
} tmz_zip_error;

typedef struct
{
    tmz_uint64 m_archive_size;
    tmz_uint64 m_central_directory_file_ofs;

    /* We only support up to UINT32_MAX files in zip64 mode. */
    tmz_uint32 m_total_files;
    tmz_zip_mode m_zip_mode;
    tmz_zip_type m_zip_type;
    tmz_zip_error m_last_error;

    tmz_uint64 m_file_offset_alignment;

    tmz_alloc_func m_pAlloc;
    tmz_free_func m_pFree;
    tmz_realloc_func m_pRealloc;
    void *m_pAlloc_opaque;

    tmz_file_read_func m_pRead;
    tmz_file_write_func m_pWrite;
	tmz_file_needs_keepalive m_pNeeds_keepalive;
    void *m_pIO_opaque;

    tmz_zip_internal_state *m_pState;

} tmz_zip_archive;

/* -------- ZIP reading */

/* Inits a ZIP archive reader. */
/* These functions read and validate the archive's central directory. */
tmz_bool tmz_zip_reader_init(tmz_zip_archive *pZip, tmz_uint64 size, tmz_uint flags);

tmz_bool tmz_zip_reader_init_mem(tmz_zip_archive *pZip, const void *pMem, size_t size, tmz_uint flags);

#ifndef TMINIZ_NO_STDIO
/* Read a archive from a disk file. */
/* file_start_ofs is the file offset where the archive actually begins, or 0. */
/* actual_archive_size is the true total size of the archive, which may be smaller than the file's actual size on disk. If zero the entire file is treated as the archive. */
tmz_bool tmz_zip_reader_init_file(tmz_zip_archive *pZip, const char *pFilename, tmz_uint32 flags);
tmz_bool tmz_zip_reader_init_file_v2(tmz_zip_archive *pZip, const char *pFilename, tmz_uint flags, tmz_uint64 file_start_ofs, tmz_uint64 archive_size);

/* Read an archive from an already opened FILE, beginning at the current file position. */
/* The archive is assumed to be archive_size bytes long. If archive_size is < 0, then the entire rest of the file is assumed to contain the archive. */
/* The FILE will NOT be closed when tmz_zip_reader_end() is called. */
tmz_bool tmz_zip_reader_init_cfile(tmz_zip_archive *pZip, tmz_FILE *pFile, tmz_uint64 archive_size, tmz_uint flags);
#endif

/* Ends archive reading, freeing all allocations, and closing the input archive file if tmz_zip_reader_init_file() was used. */
tmz_bool tmz_zip_reader_end(tmz_zip_archive *pZip);

/* -------- ZIP reading or writing */

/* Clears a tmz_zip_archive struct to all zeros. */
/* Important: This must be done before passing the struct to any tmz_zip functions. */
void tmz_zip_zero_struct(tmz_zip_archive *pZip);

tmz_zip_mode tmz_zip_get_mode(tmz_zip_archive *pZip);
tmz_zip_type tmz_zip_get_type(tmz_zip_archive *pZip);

/* Returns the total number of files in the archive. */
tmz_uint tmz_zip_reader_get_num_files(tmz_zip_archive *pZip);

tmz_uint64 tmz_zip_get_archive_size(tmz_zip_archive *pZip);
tmz_uint64 tmz_zip_get_archive_file_start_offset(tmz_zip_archive *pZip);
tmz_FILE *tmz_zip_get_cfile(tmz_zip_archive *pZip);

/* Reads n bytes of raw archive data, starting at file offset file_ofs, to pBuf. */
size_t tmz_zip_read_archive_data(tmz_zip_archive *pZip, tmz_uint64 file_ofs, void *pBuf, size_t n);

/* Attempts to locates a file in the archive's central directory. */
/* Valid flags: tmz_ZIP_FLAG_CASE_SENSITIVE, tmz_ZIP_FLAG_IGNORE_PATH */
/* Returns -1 if the file cannot be found. */
int tmz_zip_locate_file(tmz_zip_archive *pZip, const char *pName, const char *pComment, tmz_uint flags);
/* Returns tmz_FALSE if the file cannot be found. */
tmz_bool tmz_zip_locate_file_v2(tmz_zip_archive *pZip, const char *pName, const char *pComment, tmz_uint flags, tmz_uint32 *pIndex);

/* All tmz_zip funcs set the m_last_error field in the tmz_zip_archive struct. These functions retrieve/manipulate this field. */
/* Note that the m_last_error functionality is not thread safe. */
tmz_zip_error tmz_zip_set_last_error(tmz_zip_archive *pZip, tmz_zip_error err_num);
tmz_zip_error tmz_zip_peek_last_error(tmz_zip_archive *pZip);
tmz_zip_error tmz_zip_clear_last_error(tmz_zip_archive *pZip);
tmz_zip_error tmz_zip_get_last_error(tmz_zip_archive *pZip);
const char *tmz_zip_get_error_string(tmz_zip_error tmz_err);

/* tmz_TRUE if the archive file entry is a directory entry. */
tmz_bool tmz_zip_reader_is_file_a_directory(tmz_zip_archive *pZip, tmz_uint file_index);

/* tmz_TRUE if the file is encrypted/strong encrypted. */
tmz_bool tmz_zip_reader_is_file_encrypted(tmz_zip_archive *pZip, tmz_uint file_index);

/* tmz_TRUE if the compression method is supported, and the file is not encrypted, and the file is not a compressed patch file. */
tmz_bool tmz_zip_reader_is_file_supported(tmz_zip_archive *pZip, tmz_uint file_index);

/* Retrieves the filename of an archive file entry. */
/* Returns the number of bytes written to pFilename, or if filename_buf_size is 0 this function returns the number of bytes needed to fully store the filename. */
tmz_uint tmz_zip_reader_get_filename(tmz_zip_archive *pZip, tmz_uint file_index, char *pFilename, tmz_uint filename_buf_size);

/* Attempts to locates a file in the archive's central directory. */
/* Valid flags: tmz_ZIP_FLAG_CASE_SENSITIVE, tmz_ZIP_FLAG_IGNORE_PATH */
/* Returns -1 if the file cannot be found. */
int tmz_zip_reader_locate_file(tmz_zip_archive *pZip, const char *pName, const char *pComment, tmz_uint flags);
int tmz_zip_reader_locate_file_v2(tmz_zip_archive *pZip, const char *pName, const char *pComment, tmz_uint flags, tmz_uint32 *file_index);

/* Returns detailed information about an archive file entry. */
tmz_bool tmz_zip_reader_file_stat(tmz_zip_archive *pZip, tmz_uint file_index, tmz_zip_archive_file_stat *pStat);

/* tmz_TRUE if the file is in zip64 format. */
/* A file is considered zip64 if it contained a zip64 end of central directory marker, or if it contained any zip64 extended file information fields in the central directory. */
tmz_bool tmz_zip_is_zip64(tmz_zip_archive *pZip);

/* Returns the total central directory size in bytes. */
/* The current max supported size is <= tmz_UINT32_MAX. */
size_t tmz_zip_get_central_dir_size(tmz_zip_archive *pZip);

/* Extracts a archive file to a memory buffer using no memory allocation. */
/* There must be at least enough room on the stack to store the inflator's state (~34KB or so). */
tmz_bool tmz_zip_reader_extract_to_mem_no_alloc(tmz_zip_archive *pZip, tmz_uint file_index, void *pBuf, size_t buf_size, tmz_uint flags, void *pUser_read_buf, size_t user_read_buf_size);
tmz_bool tmz_zip_reader_extract_file_to_mem_no_alloc(tmz_zip_archive *pZip, const char *pFilename, void *pBuf, size_t buf_size, tmz_uint flags, void *pUser_read_buf, size_t user_read_buf_size);

/* Extracts a archive file to a memory buffer. */
tmz_bool tmz_zip_reader_extract_to_mem(tmz_zip_archive *pZip, tmz_uint file_index, void *pBuf, size_t buf_size, tmz_uint flags);
tmz_bool tmz_zip_reader_extract_file_to_mem(tmz_zip_archive *pZip, const char *pFilename, void *pBuf, size_t buf_size, tmz_uint flags);

/* Extracts a archive file to a dynamically allocated heap buffer. */
/* The memory will be allocated via the tmz_zip_archive's alloc/realloc functions. */
/* Returns NULL and sets the last error on failure. */
void *tmz_zip_reader_extract_to_heap(tmz_zip_archive *pZip, tmz_uint file_index, size_t *pSize, tmz_uint flags);
void *tmz_zip_reader_extract_file_to_heap(tmz_zip_archive *pZip, const char *pFilename, size_t *pSize, tmz_uint flags);

/* Extracts a archive file using a callback function to output the file's data. */
tmz_bool tmz_zip_reader_extract_to_callback(tmz_zip_archive *pZip, tmz_uint file_index, tmz_file_write_func pCallback, void *pOpaque, tmz_uint flags);
tmz_bool tmz_zip_reader_extract_file_to_callback(tmz_zip_archive *pZip, const char *pFilename, tmz_file_write_func pCallback, void *pOpaque, tmz_uint flags);

#ifndef TMINIZ_NO_STDIO
/* Extracts a archive file to a disk file and sets its last accessed and modified times. */
/* This function only extracts files, not archive directory records. */
tmz_bool tmz_zip_reader_extract_to_file(tmz_zip_archive *pZip, tmz_uint file_index, const char *pDst_filename, tmz_uint flags);
tmz_bool tmz_zip_reader_extract_file_to_file(tmz_zip_archive *pZip, const char *pArchive_filename, const char *pDst_filename, tmz_uint flags);

/* Extracts a archive file starting at the current position in the destination FILE stream. */
tmz_bool tmz_zip_reader_extract_to_cfile(tmz_zip_archive *pZip, tmz_uint file_index, tmz_FILE *File, tmz_uint flags);
tmz_bool tmz_zip_reader_extract_file_to_cfile(tmz_zip_archive *pZip, const char *pArchive_filename, tmz_FILE *pFile, tmz_uint flags);
#endif

#if 0
/* TODO */
	typedef void *tmz_zip_streaming_extract_state_ptr;
	tmz_zip_streaming_extract_state_ptr tmz_zip_streaming_extract_begin(tmz_zip_archive *pZip, tmz_uint file_index, tmz_uint flags);
	uint64_t tmz_zip_streaming_extract_get_size(tmz_zip_archive *pZip, tmz_zip_streaming_extract_state_ptr pState);
	uint64_t tmz_zip_streaming_extract_get_cur_ofs(tmz_zip_archive *pZip, tmz_zip_streaming_extract_state_ptr pState);
	tmz_bool tmz_zip_streaming_extract_seek(tmz_zip_archive *pZip, tmz_zip_streaming_extract_state_ptr pState, uint64_t new_ofs);
	size_t tmz_zip_streaming_extract_read(tmz_zip_archive *pZip, tmz_zip_streaming_extract_state_ptr pState, void *pBuf, size_t buf_size);
	tmz_bool tmz_zip_streaming_extract_end(tmz_zip_archive *pZip, tmz_zip_streaming_extract_state_ptr pState);
#endif

/* This function compares the archive's local headers, the optional local zip64 extended information block, and the optional descriptor following the compressed data vs. the data in the central directory. */
/* It also validates that each file can be successfully uncompressed unless the tmz_ZIP_FLAG_VALIDATE_HEADERS_ONLY is specified. */
tmz_bool tmz_zip_validate_file(tmz_zip_archive *pZip, tmz_uint file_index, tmz_uint flags);

/* Validates an entire archive by calling tmz_zip_validate_file() on each file. */
tmz_bool tmz_zip_validate_archive(tmz_zip_archive *pZip, tmz_uint flags);

/* Misc utils/helpers, valid for ZIP reading or writing */
tmz_bool tmz_zip_validate_mem_archive(const void *pMem, size_t size, tmz_uint flags, tmz_zip_error *pErr);
tmz_bool tmz_zip_validate_file_archive(const char *pFilename, tmz_uint flags, tmz_zip_error *pErr);

/* Universal end function - calls either tmz_zip_reader_end() or tmz_zip_writer_end(). */
tmz_bool tmz_zip_end(tmz_zip_archive *pZip);

/* -------- ZIP writing */

#ifndef TMINIZ_NO_ARCHIVE_WRITING_APIS

/* Inits a ZIP archive writer. */
tmz_bool tmz_zip_writer_init(tmz_zip_archive *pZip, tmz_uint64 existing_size);
tmz_bool tmz_zip_writer_init_v2(tmz_zip_archive *pZip, tmz_uint64 existing_size, tmz_uint flags);
tmz_bool tmz_zip_writer_init_heap(tmz_zip_archive *pZip, size_t size_to_reserve_at_beginning, size_t initial_allocation_size);
tmz_bool tmz_zip_writer_init_heap_v2(tmz_zip_archive *pZip, size_t size_to_reserve_at_beginning, size_t initial_allocation_size, tmz_uint flags);

#ifndef TMINIZ_NO_STDIO
tmz_bool tmz_zip_writer_init_file(tmz_zip_archive *pZip, const char *pFilename, tmz_uint64 size_to_reserve_at_beginning);
tmz_bool tmz_zip_writer_init_file_v2(tmz_zip_archive *pZip, const char *pFilename, tmz_uint64 size_to_reserve_at_beginning, tmz_uint flags);
tmz_bool tmz_zip_writer_init_cfile(tmz_zip_archive *pZip, tmz_FILE *pFile, tmz_uint flags);
#endif

/* Converts a ZIP archive reader object into a writer object, to allow efficient in-place file appends to occur on an existing archive. */
/* For archives opened using tmz_zip_reader_init_file, pFilename must be the archive's filename so it can be reopened for writing. If the file can't be reopened, tmz_zip_reader_end() will be called. */
/* For archives opened using tmz_zip_reader_init_mem, the memory block must be growable using the realloc callback (which defaults to realloc unless you've overridden it). */
/* Finally, for archives opened using tmz_zip_reader_init, the tmz_zip_archive's user provided m_pWrite function cannot be NULL. */
/* Note: In-place archive modification is not recommended unless you know what you're doing, because if execution stops or something goes wrong before */
/* the archive is finalized the file's central directory will be hosed. */
tmz_bool tmz_zip_writer_init_from_reader(tmz_zip_archive *pZip, const char *pFilename);
tmz_bool tmz_zip_writer_init_from_reader_v2(tmz_zip_archive *pZip, const char *pFilename, tmz_uint flags);

/* Adds the contents of a memory buffer to an archive. These functions record the current local time into the archive. */
/* To add a directory entry, call this method with an archive name ending in a forwardslash with an empty buffer. */
/* level_and_flags - compression level (0-10, see tmz_BEST_SPEED, tmz_BEST_COMPRESSION, etc.) logically OR'd with zero or more tmz_zip_flags, or just set to tmz_DEFAULT_COMPRESSION. */
tmz_bool tmz_zip_writer_add_mem(tmz_zip_archive *pZip, const char *pArchive_name, const void *pBuf, size_t buf_size, tmz_uint level_and_flags);

/* Like tmz_zip_writer_add_mem(), except you can specify a file comment field, and optionally supply the function with already compressed data. */
/* uncomp_size/uncomp_crc32 are only used if the tmz_ZIP_FLAG_COMPRESSED_DATA flag is specified. */
tmz_bool tmz_zip_writer_add_mem_ex(tmz_zip_archive *pZip, const char *pArchive_name, const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags,
                                 tmz_uint64 uncomp_size, tmz_uint32 uncomp_crc32);

tmz_bool tmz_zip_writer_add_mem_ex_v2(tmz_zip_archive *pZip, const char *pArchive_name, const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags,
                                    tmz_uint64 uncomp_size, tmz_uint32 uncomp_crc32, tmz_TIME_T *last_modified, const char *user_extra_data_local, tmz_uint user_extra_data_local_len,
                                    const char *user_extra_data_central, tmz_uint user_extra_data_central_len);

#ifndef TMINIZ_NO_STDIO
/* Adds the contents of a disk file to an archive. This function also records the disk file's modified time into the archive. */
/* level_and_flags - compression level (0-10, see tmz_BEST_SPEED, tmz_BEST_COMPRESSION, etc.) logically OR'd with zero or more tmz_zip_flags, or just set to tmz_DEFAULT_COMPRESSION. */
tmz_bool tmz_zip_writer_add_file(tmz_zip_archive *pZip, const char *pArchive_name, const char *pSrc_filename, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags);

/* Like tmz_zip_writer_add_file(), except the file data is read from the specified FILE stream. */
tmz_bool tmz_zip_writer_add_cfile(tmz_zip_archive *pZip, const char *pArchive_name, tmz_FILE *pSrc_file, tmz_uint64 size_to_add,
                                const tmz_TIME_T *pFile_time, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags, const char *user_extra_data_local, tmz_uint user_extra_data_local_len,
                                const char *user_extra_data_central, tmz_uint user_extra_data_central_len);
#endif

/* Adds a file to an archive by fully cloning the data from another archive. */
/* This function fully clones the source file's compressed data (no recompression), along with its full filename, extra data (it may add or modify the zip64 local header extra data field), and the optional descriptor following the compressed data. */
tmz_bool tmz_zip_writer_add_from_zip_reader(tmz_zip_archive *pZip, tmz_zip_archive *pSource_zip, tmz_uint src_file_index);

/* Finalizes the archive by writing the central directory records followed by the end of central directory record. */
/* After an archive is finalized, the only valid call on the tmz_zip_archive struct is tmz_zip_writer_end(). */
/* An archive must be manually finalized by calling this function for it to be valid. */
tmz_bool tmz_zip_writer_finalize_archive(tmz_zip_archive *pZip);

/* Finalizes a heap archive, returning a poiner to the heap block and its size. */
/* The heap block will be allocated using the tmz_zip_archive's alloc/realloc callbacks. */
tmz_bool tmz_zip_writer_finalize_heap_archive(tmz_zip_archive *pZip, void **ppBuf, size_t *pSize);

/* Ends archive writing, freeing all allocations, and closing the output file if tmz_zip_writer_init_file() was used. */
/* Note for the archive to be valid, it *must* have been finalized before ending (this function will not do it for you). */
tmz_bool tmz_zip_writer_end(tmz_zip_archive *pZip);

/* -------- Misc. high-level helper functions: */

/* tmz_zip_add_mem_to_archive_file_in_place() efficiently (but not atomically) appends a memory blob to a ZIP archive. */
/* Note this is NOT a fully safe operation. If it crashes or dies in some way your archive can be left in a screwed up state (without a central directory). */
/* level_and_flags - compression level (0-10, see tmz_BEST_SPEED, tmz_BEST_COMPRESSION, etc.) logically OR'd with zero or more tmz_zip_flags, or just set to tmz_DEFAULT_COMPRESSION. */
/* TODO: Perhaps add an option to leave the existing central dir in place in case the add dies? We could then truncate the file (so the old central dir would be at the end) if something goes wrong. */
tmz_bool tmz_zip_add_mem_to_archive_file_in_place(const char *pZip_filename, const char *pArchive_name, const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags);
tmz_bool tmz_zip_add_mem_to_archive_file_in_place_v2(const char *pZip_filename, const char *pArchive_name, const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags, tmz_zip_error *pErr);

/* Reads a single file from an archive into a heap block. */
/* If pComment is not NULL, only the file with the specified comment will be extracted. */
/* Returns NULL on failure. */
void *tmz_zip_extract_archive_file_to_heap(const char *pZip_filename, const char *pArchive_name, size_t *pSize, tmz_uint flags);
void *tmz_zip_extract_archive_file_to_heap_v2(const char *pZip_filename, const char *pArchive_name, const char *pComment, size_t *pSize, tmz_uint flags, tmz_zip_error *pErr);

#endif /* #ifndef TMINIZ_NO_ARCHIVE_WRITING_APIS */

#ifdef __cplusplus
}
#endif

#endif /* TMINIZ_NO_ARCHIVE_APIS */
