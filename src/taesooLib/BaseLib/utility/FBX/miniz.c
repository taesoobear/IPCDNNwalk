/**************************************************************************
 *
 * Copyright 2013-2014 RAD Game Tools and Valve Software
 * Copyright 2010-2014 Rich Geldreich and Tenacious Software LLC
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 **************************************************************************/

#include  "miniz.h"

typedef unsigned char tmz_validate_uint16[sizeof(tmz_uint16) == 2 ? 1 : -1];
typedef unsigned char tmz_validate_uint32[sizeof(tmz_uint32) == 4 ? 1 : -1];
typedef unsigned char tmz_validate_uint64[sizeof(tmz_uint64) == 8 ? 1 : -1];

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------- zlib-style API's */

tmz_ulong tmz_adler32(tmz_ulong adler, const unsigned char *ptr, size_t buf_len)
{
    tmz_uint32 i, s1 = (tmz_uint32)(adler & 0xffff), s2 = (tmz_uint32)(adler >> 16);
    size_t block_len = buf_len % 5552;
    if (!ptr)
        return tmz_ADLER32_INIT;
    while (buf_len)
    {
        for (i = 0; i + 7 < block_len; i += 8, ptr += 8)
        {
            s1 += ptr[0], s2 += s1;
            s1 += ptr[1], s2 += s1;
            s1 += ptr[2], s2 += s1;
            s1 += ptr[3], s2 += s1;
            s1 += ptr[4], s2 += s1;
            s1 += ptr[5], s2 += s1;
            s1 += ptr[6], s2 += s1;
            s1 += ptr[7], s2 += s1;
        }
        for (; i < block_len; ++i)
            s1 += *ptr++, s2 += s1;
        s1 %= 65521U, s2 %= 65521U;
        buf_len -= block_len;
        block_len = 5552;
    }
    return (s2 << 16) + s1;
}

/* Karl Malbrain's compact CRC-32. See "A compact CCITT crc16 and crc32 C implementation that balances processor cache usage against speed": http://www.geocities.com/malbrain/ */
#if 0
    tmz_ulong tmz_crc32(tmz_ulong crc, const tmz_uint8 *ptr, size_t buf_len)
    {
        static const tmz_uint32 s_crc32[16] = { 0, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
                                               0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c };
        tmz_uint32 crcu32 = (tmz_uint32)crc;
        if (!ptr)
            return tmz_CRC32_INIT;
        crcu32 = ~crcu32;
        while (buf_len--)
        {
            tmz_uint8 b = *ptr++;
            crcu32 = (crcu32 >> 4) ^ s_crc32[(crcu32 & 0xF) ^ (b & 0xF)];
            crcu32 = (crcu32 >> 4) ^ s_crc32[(crcu32 & 0xF) ^ (b >> 4)];
        }
        return ~crcu32;
    }
#else
/* Faster, but larger CPU cache footprint.
 */
tmz_ulong tmz_crc32(tmz_ulong crc, const tmz_uint8 *ptr, size_t buf_len)
{
    static const tmz_uint32 s_crc_table[256] =
        {
            0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535,
            0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD,
            0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D,
            0x6DDDE4EB, 0xF4D4B551, 0x83D385C7, 0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
            0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4,
            0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
            0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 0x26D930AC,
            0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
            0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB,
            0xB6662D3D, 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F,
            0x9FBFE4A5, 0xE8B8D433, 0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB,
            0x086D3D2D, 0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
            0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA,
            0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65, 0x4DB26158, 0x3AB551CE,
            0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A,
            0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
            0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409,
            0xCE61E49F, 0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
            0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739,
            0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
            0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1, 0xF00F9344, 0x8708A3D2, 0x1E01F268,
            0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0,
            0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8,
            0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
            0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF,
            0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703,
            0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7,
            0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D, 0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
            0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE,
            0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
            0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777, 0x88085AE6,
            0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
            0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D,
            0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5,
            0x47B2CF7F, 0x30B5FFE9, 0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605,
            0xCDD70693, 0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
            0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
        };

    tmz_uint32 crc32 = (tmz_uint32)crc ^ 0xFFFFFFFF;
    const tmz_uint8 *pByte_buf = (const tmz_uint8 *)ptr;

    while (buf_len >= 4)
    {
        crc32 = (crc32 >> 8) ^ s_crc_table[(crc32 ^ pByte_buf[0]) & 0xFF];
        crc32 = (crc32 >> 8) ^ s_crc_table[(crc32 ^ pByte_buf[1]) & 0xFF];
        crc32 = (crc32 >> 8) ^ s_crc_table[(crc32 ^ pByte_buf[2]) & 0xFF];
        crc32 = (crc32 >> 8) ^ s_crc_table[(crc32 ^ pByte_buf[3]) & 0xFF];
        pByte_buf += 4;
        buf_len -= 4;
    }

    while (buf_len)
    {
        crc32 = (crc32 >> 8) ^ s_crc_table[(crc32 ^ pByte_buf[0]) & 0xFF];
        ++pByte_buf;
        --buf_len;
    }

    return ~crc32;
}
#endif

void tmz_free(void *p)
{
    tmz_FREE(p);
}

void *tminiz_def_alloc_func(void *opaque, size_t items, size_t size)
{
    (void)opaque, (void)items, (void)size;
    return tmz_MALLOC(items * size);
}
void tminiz_def_free_func(void *opaque, void *address)
{
    (void)opaque, (void)address;
    tmz_FREE(address);
}
void *tminiz_def_realloc_func(void *opaque, void *address, size_t items, size_t size)
{
    (void)opaque, (void)address, (void)items, (void)size;
    return tmz_REALLOC(address, items * size);
}

const char *tmz_version(void)
{
    return tmz_VERSION;
}

#ifndef tminiz_NO_ZLIB_APIS

int tmz_deflateInit(tmz_streamp pStream, int level)
{
    return tmz_deflateInit2(pStream, level, tmz_DEFLATED, tmz_DEFAULT_WINDOW_BITS, 9, tmz_DEFAULT_STRATEGY);
}

int tmz_deflateInit2(tmz_streamp pStream, int level, int method, int window_bits, int mem_level, int strategy)
{
    tdefl_compressor *pComp;
    tmz_uint comp_flags = TDEFL_COMPUTE_ADLER32 | tdefl_create_comp_flags_from_zip_params(level, window_bits, strategy);

    if (!pStream)
        return tmz_STREAM_ERROR;
    if ((method != tmz_DEFLATED) || ((mem_level < 1) || (mem_level > 9)) || ((window_bits != tmz_DEFAULT_WINDOW_BITS) && (-window_bits != tmz_DEFAULT_WINDOW_BITS)))
        return tmz_PARAM_ERROR;

    pStream->data_type = 0;
    pStream->adler = tmz_ADLER32_INIT;
    pStream->msg = NULL;
    pStream->reserved = 0;
    pStream->total_in = 0;
    pStream->total_out = 0;
    if (!pStream->zalloc)
        pStream->zalloc = tminiz_def_alloc_func;
    if (!pStream->zfree)
        pStream->zfree = tminiz_def_free_func;

    pComp = (tdefl_compressor *)pStream->zalloc(pStream->opaque, 1, sizeof(tdefl_compressor));
    if (!pComp)
        return tmz_MEM_ERROR;

    pStream->state = (struct tmz_internal_state *)pComp;

    if (tdefl_init(pComp, NULL, NULL, comp_flags) != TDEFL_STATUS_OKAY)
    {
        tmz_deflateEnd(pStream);
        return tmz_PARAM_ERROR;
    }

    return tmz_OK;
}

int tmz_deflateReset(tmz_streamp pStream)
{
    if ((!pStream) || (!pStream->state) || (!pStream->zalloc) || (!pStream->zfree))
        return tmz_STREAM_ERROR;
    pStream->total_in = pStream->total_out = 0;
    tdefl_init((tdefl_compressor *)pStream->state, NULL, NULL, ((tdefl_compressor *)pStream->state)->m_flags);
    return tmz_OK;
}

int tmz_deflate(tmz_streamp pStream, int flush)
{
    size_t in_bytes, out_bytes;
    tmz_ulong orig_total_in, orig_total_out;
    int tmz_status = tmz_OK;

    if ((!pStream) || (!pStream->state) || (flush < 0) || (flush > tmz_FINISH) || (!pStream->next_out))
        return tmz_STREAM_ERROR;
    if (!pStream->avail_out)
        return tmz_BUF_ERROR;

    if (flush == tmz_PARTIAL_FLUSH)
        flush = tmz_SYNC_FLUSH;

    if (((tdefl_compressor *)pStream->state)->m_prev_return_status == TDEFL_STATUS_DONE)
        return (flush == tmz_FINISH) ? tmz_STREAM_END : tmz_BUF_ERROR;

    orig_total_in = pStream->total_in;
    orig_total_out = pStream->total_out;
    for (;;)
    {
        tdefl_status defl_status;
        in_bytes = pStream->avail_in;
        out_bytes = pStream->avail_out;

        defl_status = tdefl_compress((tdefl_compressor *)pStream->state, pStream->next_in, &in_bytes, pStream->next_out, &out_bytes, (tdefl_flush)flush);
        pStream->next_in += (tmz_uint)in_bytes;
        pStream->avail_in -= (tmz_uint)in_bytes;
        pStream->total_in += (tmz_uint)in_bytes;
        pStream->adler = tdefl_get_adler32((tdefl_compressor *)pStream->state);

        pStream->next_out += (tmz_uint)out_bytes;
        pStream->avail_out -= (tmz_uint)out_bytes;
        pStream->total_out += (tmz_uint)out_bytes;

        if (defl_status < 0)
        {
            tmz_status = tmz_STREAM_ERROR;
            break;
        }
        else if (defl_status == TDEFL_STATUS_DONE)
        {
            tmz_status = tmz_STREAM_END;
            break;
        }
        else if (!pStream->avail_out)
            break;
        else if ((!pStream->avail_in) && (flush != tmz_FINISH))
        {
            if ((flush) || (pStream->total_in != orig_total_in) || (pStream->total_out != orig_total_out))
                break;
            return tmz_BUF_ERROR; /* Can't make forward progress without some input.
 */
        }
    }
    return tmz_status;
}

int tmz_deflateEnd(tmz_streamp pStream)
{
    if (!pStream)
        return tmz_STREAM_ERROR;
    if (pStream->state)
    {
        pStream->zfree(pStream->opaque, pStream->state);
        pStream->state = NULL;
    }
    return tmz_OK;
}

tmz_ulong tmz_deflateBound(tmz_streamp pStream, tmz_ulong source_len)
{
    (void)pStream;
    /* This is really over conservative. (And lame, but it's actually pretty tricky to compute a true upper bound given the way tdefl's blocking works.) */
    return tmz_MAX(128 + (source_len * 110) / 100, 128 + source_len + ((source_len / (31 * 1024)) + 1) * 5);
}

int tmz_compress2(unsigned char *pDest, tmz_ulong *pDest_len, const unsigned char *pSource, tmz_ulong source_len, int level)
{
    int status;
    tmz_stream stream;
    memset(&stream, 0, sizeof(stream));

    /* In case tmz_ulong is 64-bits (argh I hate longs). */
    if ((source_len | *pDest_len) > 0xFFFFFFFFU)
        return tmz_PARAM_ERROR;

    stream.next_in = pSource;
    stream.avail_in = (tmz_uint32)source_len;
    stream.next_out = pDest;
    stream.avail_out = (tmz_uint32) * pDest_len;

    status = tmz_deflateInit(&stream, level);
    if (status != tmz_OK)
        return status;

    status = tmz_deflate(&stream, tmz_FINISH);
    if (status != tmz_STREAM_END)
    {
        tmz_deflateEnd(&stream);
        return (status == tmz_OK) ? tmz_BUF_ERROR : status;
    }

    *pDest_len = stream.total_out;
    return tmz_deflateEnd(&stream);
}

int tmz_compress(unsigned char *pDest, tmz_ulong *pDest_len, const unsigned char *pSource, tmz_ulong source_len)
{
    return tmz_compress2(pDest, pDest_len, pSource, source_len, tmz_DEFAULT_COMPRESSION);
}

tmz_ulong tmz_compressBound(tmz_ulong source_len)
{
    return tmz_deflateBound(NULL, source_len);
}

typedef struct
{
    tinfl_decompressor m_decomp;
    tmz_uint m_dict_ofs, m_dict_avail, m_first_call, m_has_flushed;
    int m_window_bits;
    tmz_uint8 m_dict[TINFL_LZ_DICT_SIZE];
    tinfl_status m_last_status;
} inflate_state;

int tmz_inflateInit2(tmz_streamp pStream, int window_bits)
{
    inflate_state *pDecomp;
    if (!pStream)
        return tmz_STREAM_ERROR;
    if ((window_bits != tmz_DEFAULT_WINDOW_BITS) && (-window_bits != tmz_DEFAULT_WINDOW_BITS))
        return tmz_PARAM_ERROR;

    pStream->data_type = 0;
    pStream->adler = 0;
    pStream->msg = NULL;
    pStream->total_in = 0;
    pStream->total_out = 0;
    pStream->reserved = 0;
    if (!pStream->zalloc)
        pStream->zalloc = tminiz_def_alloc_func;
    if (!pStream->zfree)
        pStream->zfree = tminiz_def_free_func;

    pDecomp = (inflate_state *)pStream->zalloc(pStream->opaque, 1, sizeof(inflate_state));
    if (!pDecomp)
        return tmz_MEM_ERROR;

    pStream->state = (struct tmz_internal_state *)pDecomp;

    tinfl_init(&pDecomp->m_decomp);
    pDecomp->m_dict_ofs = 0;
    pDecomp->m_dict_avail = 0;
    pDecomp->m_last_status = TINFL_STATUS_NEEDS_MORE_INPUT;
    pDecomp->m_first_call = 1;
    pDecomp->m_has_flushed = 0;
    pDecomp->m_window_bits = window_bits;

    return tmz_OK;
}

int tmz_inflateInit(tmz_streamp pStream)
{
    return tmz_inflateInit2(pStream, tmz_DEFAULT_WINDOW_BITS);
}

int tmz_inflate(tmz_streamp pStream, int flush)
{
    inflate_state *pState;
    tmz_uint n, first_call, decomp_flags = TINFL_FLAG_COMPUTE_ADLER32;
    size_t in_bytes, out_bytes, orig_avail_in;
    tinfl_status status;

    if ((!pStream) || (!pStream->state))
        return tmz_STREAM_ERROR;
    if (flush == tmz_PARTIAL_FLUSH)
        flush = tmz_SYNC_FLUSH;
    if ((flush) && (flush != tmz_SYNC_FLUSH) && (flush != tmz_FINISH))
        return tmz_STREAM_ERROR;

    pState = (inflate_state *)pStream->state;
    if (pState->m_window_bits > 0)
        decomp_flags |= TINFL_FLAG_PARSE_ZLIB_HEADER;
    orig_avail_in = pStream->avail_in;

    first_call = pState->m_first_call;
    pState->m_first_call = 0;
    if (pState->m_last_status < 0)
        return tmz_DATA_ERROR;

    if (pState->m_has_flushed && (flush != tmz_FINISH))
        return tmz_STREAM_ERROR;
    pState->m_has_flushed |= (flush == tmz_FINISH);

    if ((flush == tmz_FINISH) && (first_call))
    {
        /* tmz_FINISH on the first call implies that the input and output buffers are large enough to hold the entire compressed/decompressed file. */
        decomp_flags |= TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF;
        in_bytes = pStream->avail_in;
        out_bytes = pStream->avail_out;
        status = tinfl_decompress(&pState->m_decomp, pStream->next_in, &in_bytes, pStream->next_out, pStream->next_out, &out_bytes, decomp_flags);
        pState->m_last_status = status;
        pStream->next_in += (tmz_uint)in_bytes;
        pStream->avail_in -= (tmz_uint)in_bytes;
        pStream->total_in += (tmz_uint)in_bytes;
        pStream->adler = tinfl_get_adler32(&pState->m_decomp);
        pStream->next_out += (tmz_uint)out_bytes;
        pStream->avail_out -= (tmz_uint)out_bytes;
        pStream->total_out += (tmz_uint)out_bytes;

        if (status < 0)
            return tmz_DATA_ERROR;
        else if (status != TINFL_STATUS_DONE)
        {
            pState->m_last_status = TINFL_STATUS_FAILED;
            return tmz_BUF_ERROR;
        }
        return tmz_STREAM_END;
    }
    /* flush != tmz_FINISH then we must assume there's more input. */
    if (flush != tmz_FINISH)
        decomp_flags |= TINFL_FLAG_HAS_MORE_INPUT;

    if (pState->m_dict_avail)
    {
        n = tmz_MIN(pState->m_dict_avail, pStream->avail_out);
        memcpy(pStream->next_out, pState->m_dict + pState->m_dict_ofs, n);
        pStream->next_out += n;
        pStream->avail_out -= n;
        pStream->total_out += n;
        pState->m_dict_avail -= n;
        pState->m_dict_ofs = (pState->m_dict_ofs + n) & (TINFL_LZ_DICT_SIZE - 1);
        return ((pState->m_last_status == TINFL_STATUS_DONE) && (!pState->m_dict_avail)) ? tmz_STREAM_END : tmz_OK;
    }

    for (;;)
    {
        in_bytes = pStream->avail_in;
        out_bytes = TINFL_LZ_DICT_SIZE - pState->m_dict_ofs;

        status = tinfl_decompress(&pState->m_decomp, pStream->next_in, &in_bytes, pState->m_dict, pState->m_dict + pState->m_dict_ofs, &out_bytes, decomp_flags);
        pState->m_last_status = status;

        pStream->next_in += (tmz_uint)in_bytes;
        pStream->avail_in -= (tmz_uint)in_bytes;
        pStream->total_in += (tmz_uint)in_bytes;
        pStream->adler = tinfl_get_adler32(&pState->m_decomp);

        pState->m_dict_avail = (tmz_uint)out_bytes;

        n = tmz_MIN(pState->m_dict_avail, pStream->avail_out);
        memcpy(pStream->next_out, pState->m_dict + pState->m_dict_ofs, n);
        pStream->next_out += n;
        pStream->avail_out -= n;
        pStream->total_out += n;
        pState->m_dict_avail -= n;
        pState->m_dict_ofs = (pState->m_dict_ofs + n) & (TINFL_LZ_DICT_SIZE - 1);

        if (status < 0)
            return tmz_DATA_ERROR; /* Stream is corrupted (there could be some uncompressed data left in the output dictionary - oh well). */
        else if ((status == TINFL_STATUS_NEEDS_MORE_INPUT) && (!orig_avail_in))
            return tmz_BUF_ERROR; /* Signal caller that we can't make forward progress without supplying more input or by setting flush to tmz_FINISH. */
        else if (flush == tmz_FINISH)
        {
            /* The output buffer MUST be large to hold the remaining uncompressed data when flush==tmz_FINISH. */
            if (status == TINFL_STATUS_DONE)
                return pState->m_dict_avail ? tmz_BUF_ERROR : tmz_STREAM_END;
            /* status here must be TINFL_STATUS_HAS_MORE_OUTPUT, which means there's at least 1 more byte on the way. If there's no more room left in the output buffer then something is wrong. */
            else if (!pStream->avail_out)
                return tmz_BUF_ERROR;
        }
        else if ((status == TINFL_STATUS_DONE) || (!pStream->avail_in) || (!pStream->avail_out) || (pState->m_dict_avail))
            break;
    }

    return ((status == TINFL_STATUS_DONE) && (!pState->m_dict_avail)) ? tmz_STREAM_END : tmz_OK;
}

int tmz_inflateEnd(tmz_streamp pStream)
{
    if (!pStream)
        return tmz_STREAM_ERROR;
    if (pStream->state)
    {
        pStream->zfree(pStream->opaque, pStream->state);
        pStream->state = NULL;
    }
    return tmz_OK;
}

int tmz_uncompress(unsigned char *pDest, tmz_ulong *pDest_len, const unsigned char *pSource, tmz_ulong source_len)
{
    tmz_stream stream;
    int status;
    memset(&stream, 0, sizeof(stream));

    /* In case tmz_ulong is 64-bits (argh I hate longs). */
    if ((source_len | *pDest_len) > 0xFFFFFFFFU)
        return tmz_PARAM_ERROR;

    stream.next_in = pSource;
    stream.avail_in = (tmz_uint32)source_len;
    stream.next_out = pDest;
    stream.avail_out = (tmz_uint32) * pDest_len;

    status = tmz_inflateInit(&stream);
    if (status != tmz_OK)
        return status;

    status = tmz_inflate(&stream, tmz_FINISH);
    if (status != tmz_STREAM_END)
    {
        tmz_inflateEnd(&stream);
        return ((status == tmz_BUF_ERROR) && (!stream.avail_in)) ? tmz_DATA_ERROR : status;
    }
    *pDest_len = stream.total_out;

    return tmz_inflateEnd(&stream);
}

const char *tmz_error(int err)
{
    static struct
    {
        int m_err;
        const char *m_pDesc;
    } s_error_descs[] =
          {
              { tmz_OK, "" }, { tmz_STREAM_END, "stream end" }, { tmz_NEED_DICT, "need dictionary" }, { tmz_ERRNO, "file error" }, { tmz_STREAM_ERROR, "stream error" },
              { tmz_DATA_ERROR, "data error" }, { tmz_MEM_ERROR, "out of memory" }, { tmz_BUF_ERROR, "buf error" }, { tmz_VERSION_ERROR, "version error" }, { tmz_PARAM_ERROR, "parameter error" }
          };
    tmz_uint i;
    for (i = 0; i < sizeof(s_error_descs) / sizeof(s_error_descs[0]); ++i)
        if (s_error_descs[i].m_err == err)
            return s_error_descs[i].m_pDesc;
    return NULL;
}

#endif /*tminiz_NO_ZLIB_APIS */

#ifdef __cplusplus
}
#endif

/*
  This is free and unencumbered software released into the public domain.

  Anyone is free to copy, modify, publish, use, compile, sell, or
  distribute this software, either in source code form or as a compiled
  binary, for any purpose, commercial or non-commercial, and by any
  means.

  In jurisdictions that recognize copyright laws, the author or authors
  of this software dedicate any and all copyright interest in the
  software to the public domain. We make this dedication for the benefit
  of the public at large and to the detriment of our heirs and
  successors. We intend this dedication to be an overt act of
  relinquishment in perpetuity of all present and future rights to this
  software under copyright law.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
  OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.

  For more information, please refer to <http://unlicense.org/>
*/
/**************************************************************************
 *
 * Copyright 2013-2014 RAD Game Tools and Valve Software
 * Copyright 2010-2014 Rich Geldreich and Tenacious Software LLC
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 **************************************************************************/




#ifdef __cplusplus
extern "C" {
#endif

/* ------------------- Low-level Compression (independent from all decompression API's) */

/* Purposely making these tables static for faster init and thread safety. */
static const tmz_uint16 s_tdefl_len_sym[256] =
    {
        257, 258, 259, 260, 261, 262, 263, 264, 265, 265, 266, 266, 267, 267, 268, 268, 269, 269, 269, 269, 270, 270, 270, 270, 271, 271, 271, 271, 272, 272, 272, 272,
        273, 273, 273, 273, 273, 273, 273, 273, 274, 274, 274, 274, 274, 274, 274, 274, 275, 275, 275, 275, 275, 275, 275, 275, 276, 276, 276, 276, 276, 276, 276, 276,
        277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278,
        279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280,
        281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281,
        282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282,
        283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283,
        284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 285
    };

static const tmz_uint8 s_tdefl_len_extra[256] =
    {
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0
    };

static const tmz_uint8 s_tdefl_small_dist_sym[512] =
    {
        0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11,
        11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13,
        13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
        14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
        14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
        15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
        17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
        17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
        17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17
    };

static const tmz_uint8 s_tdefl_small_dist_extra[512] =
    {
        0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7
    };

static const tmz_uint8 s_tdefl_large_dist_sym[128] =
    {
        0, 0, 18, 19, 20, 20, 21, 21, 22, 22, 22, 22, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26,
        26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28,
        28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29
    };

static const tmz_uint8 s_tdefl_large_dist_extra[128] =
    {
        0, 0, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
        12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
        13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13
    };

/* Radix sorts tdefl_sym_freq[] array by 16-bit key m_key. Returns ptr to sorted values. */
typedef struct
{
    tmz_uint16 m_key, m_sym_index;
} tdefl_sym_freq;
static tdefl_sym_freq *tdefl_radix_sort_syms(tmz_uint num_syms, tdefl_sym_freq *pSyms0, tdefl_sym_freq *pSyms1)
{
    tmz_uint32 total_passes = 2, pass_shift, pass, i, hist[256 * 2];
    tdefl_sym_freq *pCur_syms = pSyms0, *pNew_syms = pSyms1;
    tmz_CLEAR_OBJ(hist);
    for (i = 0; i < num_syms; i++)
    {
        tmz_uint freq = pSyms0[i].m_key;
        hist[freq & 0xFF]++;
        hist[256 + ((freq >> 8) & 0xFF)]++;
    }
    while ((total_passes > 1) && (num_syms == hist[(total_passes - 1) * 256]))
        total_passes--;
    for (pass_shift = 0, pass = 0; pass < total_passes; pass++, pass_shift += 8)
    {
        const tmz_uint32 *pHist = &hist[pass << 8];
        tmz_uint offsets[256], cur_ofs = 0;
        for (i = 0; i < 256; i++)
        {
            offsets[i] = cur_ofs;
            cur_ofs += pHist[i];
        }
        for (i = 0; i < num_syms; i++)
            pNew_syms[offsets[(pCur_syms[i].m_key >> pass_shift) & 0xFF]++] = pCur_syms[i];
        {
            tdefl_sym_freq *t = pCur_syms;
            pCur_syms = pNew_syms;
            pNew_syms = t;
        }
    }
    return pCur_syms;
}

/* tdefl_calculate_minimum_redundancy() originally written by: Alistair Moffat, alistair@cs.mu.oz.au, Jyrki Katajainen, jyrki@diku.dk, November 1996. */
static void tdefl_calculate_minimum_redundancy(tdefl_sym_freq *A, int n)
{
    int root, leaf, next, avbl, used, dpth;
    if (n == 0)
        return;
    else if (n == 1)
    {
        A[0].m_key = 1;
        return;
    }
    A[0].m_key += A[1].m_key;
    root = 0;
    leaf = 2;
    for (next = 1; next < n - 1; next++)
    {
        if (leaf >= n || A[root].m_key < A[leaf].m_key)
        {
            A[next].m_key = A[root].m_key;
            A[root++].m_key = (tmz_uint16)next;
        }
        else
            A[next].m_key = A[leaf++].m_key;
        if (leaf >= n || (root < next && A[root].m_key < A[leaf].m_key))
        {
            A[next].m_key = (tmz_uint16)(A[next].m_key + A[root].m_key);
            A[root++].m_key = (tmz_uint16)next;
        }
        else
            A[next].m_key = (tmz_uint16)(A[next].m_key + A[leaf++].m_key);
    }
    A[n - 2].m_key = 0;
    for (next = n - 3; next >= 0; next--)
        A[next].m_key = A[A[next].m_key].m_key + 1;
    avbl = 1;
    used = dpth = 0;
    root = n - 2;
    next = n - 1;
    while (avbl > 0)
    {
        while (root >= 0 && (int)A[root].m_key == dpth)
        {
            used++;
            root--;
        }
        while (avbl > used)
        {
            A[next--].m_key = (tmz_uint16)(dpth);
            avbl--;
        }
        avbl = 2 * used;
        dpth++;
        used = 0;
    }
}

/* Limits canonical Huffman code table's max code size. */
enum
{
    TDEFL_MAX_SUPPORTED_HUFF_CODESIZE = 32
};
static void tdefl_huffman_enforce_max_code_size(int *pNum_codes, int code_list_len, int max_code_size)
{
    int i;
    tmz_uint32 total = 0;
    if (code_list_len <= 1)
        return;
    for (i = max_code_size + 1; i <= TDEFL_MAX_SUPPORTED_HUFF_CODESIZE; i++)
        pNum_codes[max_code_size] += pNum_codes[i];
    for (i = max_code_size; i > 0; i--)
        total += (((tmz_uint32)pNum_codes[i]) << (max_code_size - i));
    while (total != (1UL << max_code_size))
    {
        pNum_codes[max_code_size]--;
        for (i = max_code_size - 1; i > 0; i--)
            if (pNum_codes[i])
            {
                pNum_codes[i]--;
                pNum_codes[i + 1] += 2;
                break;
            }
        total--;
    }
}

static void tdefl_optimize_huffman_table(tdefl_compressor *d, int table_num, int table_len, int code_size_limit, int static_table)
{
    int i, j, l, num_codes[1 + TDEFL_MAX_SUPPORTED_HUFF_CODESIZE];
    tmz_uint next_code[TDEFL_MAX_SUPPORTED_HUFF_CODESIZE + 1];
    tmz_CLEAR_OBJ(num_codes);
    if (static_table)
    {
        for (i = 0; i < table_len; i++)
            num_codes[d->m_huff_code_sizes[table_num][i]]++;
    }
    else
    {
        tdefl_sym_freq syms0[TDEFL_MAX_HUFF_SYMBOLS], syms1[TDEFL_MAX_HUFF_SYMBOLS], *pSyms;
        int num_used_syms = 0;
        const tmz_uint16 *pSym_count = &d->m_huff_count[table_num][0];
        for (i = 0; i < table_len; i++)
            if (pSym_count[i])
            {
                syms0[num_used_syms].m_key = (tmz_uint16)pSym_count[i];
                syms0[num_used_syms++].m_sym_index = (tmz_uint16)i;
            }

        pSyms = tdefl_radix_sort_syms(num_used_syms, syms0, syms1);
        tdefl_calculate_minimum_redundancy(pSyms, num_used_syms);

        for (i = 0; i < num_used_syms; i++)
            num_codes[pSyms[i].m_key]++;

        tdefl_huffman_enforce_max_code_size(num_codes, num_used_syms, code_size_limit);

        tmz_CLEAR_OBJ(d->m_huff_code_sizes[table_num]);
        tmz_CLEAR_OBJ(d->m_huff_codes[table_num]);
        for (i = 1, j = num_used_syms; i <= code_size_limit; i++)
            for (l = num_codes[i]; l > 0; l--)
                d->m_huff_code_sizes[table_num][pSyms[--j].m_sym_index] = (tmz_uint8)(i);
    }

    next_code[1] = 0;
    for (j = 0, i = 2; i <= code_size_limit; i++)
        next_code[i] = j = ((j + num_codes[i - 1]) << 1);

    for (i = 0; i < table_len; i++)
    {
        tmz_uint rev_code = 0, code, code_size;
        if ((code_size = d->m_huff_code_sizes[table_num][i]) == 0)
            continue;
        code = next_code[code_size]++;
        for (l = code_size; l > 0; l--, code >>= 1)
            rev_code = (rev_code << 1) | (code & 1);
        d->m_huff_codes[table_num][i] = (tmz_uint16)rev_code;
    }
}

#define TDEFL_PUT_BITS(b, l)                                       \
    do                                                             \
    {                                                              \
        tmz_uint bits = b;                                          \
        tmz_uint len = l;                                           \
        tmz_ASSERT(bits <= ((1U << len) - 1U));                     \
        d->m_bit_buffer |= (bits << d->m_bits_in);                 \
        d->m_bits_in += len;                                       \
        while (d->m_bits_in >= 8)                                  \
        {                                                          \
            if (d->m_pOutput_buf < d->m_pOutput_buf_end)           \
                *d->m_pOutput_buf++ = (tmz_uint8)(d->m_bit_buffer); \
            d->m_bit_buffer >>= 8;                                 \
            d->m_bits_in -= 8;                                     \
        }                                                          \
    }                                                              \
    tmz_MACRO_END

#define TDEFL_RLE_PREV_CODE_SIZE()                                                                                       \
    {                                                                                                                    \
        if (rle_repeat_count)                                                                                            \
        {                                                                                                                \
            if (rle_repeat_count < 3)                                                                                    \
            {                                                                                                            \
                d->m_huff_count[2][prev_code_size] = (tmz_uint16)(d->m_huff_count[2][prev_code_size] + rle_repeat_count); \
                while (rle_repeat_count--)                                                                               \
                    packed_code_sizes[num_packed_code_sizes++] = prev_code_size;                                         \
            }                                                                                                            \
            else                                                                                                         \
            {                                                                                                            \
                d->m_huff_count[2][16] = (tmz_uint16)(d->m_huff_count[2][16] + 1);                                        \
                packed_code_sizes[num_packed_code_sizes++] = 16;                                                         \
                packed_code_sizes[num_packed_code_sizes++] = (tmz_uint8)(rle_repeat_count - 3);                           \
            }                                                                                                            \
            rle_repeat_count = 0;                                                                                        \
        }                                                                                                                \
    }

#define TDEFL_RLE_ZERO_CODE_SIZE()                                                         \
    {                                                                                      \
        if (rle_z_count)                                                                   \
        {                                                                                  \
            if (rle_z_count < 3)                                                           \
            {                                                                              \
                d->m_huff_count[2][0] = (tmz_uint16)(d->m_huff_count[2][0] + rle_z_count);  \
                while (rle_z_count--)                                                      \
                    packed_code_sizes[num_packed_code_sizes++] = 0;                        \
            }                                                                              \
            else if (rle_z_count <= 10)                                                    \
            {                                                                              \
                d->m_huff_count[2][17] = (tmz_uint16)(d->m_huff_count[2][17] + 1);          \
                packed_code_sizes[num_packed_code_sizes++] = 17;                           \
                packed_code_sizes[num_packed_code_sizes++] = (tmz_uint8)(rle_z_count - 3);  \
            }                                                                              \
            else                                                                           \
            {                                                                              \
                d->m_huff_count[2][18] = (tmz_uint16)(d->m_huff_count[2][18] + 1);          \
                packed_code_sizes[num_packed_code_sizes++] = 18;                           \
                packed_code_sizes[num_packed_code_sizes++] = (tmz_uint8)(rle_z_count - 11); \
            }                                                                              \
            rle_z_count = 0;                                                               \
        }                                                                                  \
    }

static tmz_uint8 s_tdefl_packed_code_size_syms_swizzle[] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };

static void tdefl_start_dynamic_block(tdefl_compressor *d)
{
    int num_lit_codes, num_dist_codes, num_bit_lengths;
    tmz_uint i, total_code_sizes_to_pack, num_packed_code_sizes, rle_z_count, rle_repeat_count, packed_code_sizes_index;
    tmz_uint8 code_sizes_to_pack[TDEFL_MAX_HUFF_SYMBOLS_0 + TDEFL_MAX_HUFF_SYMBOLS_1], packed_code_sizes[TDEFL_MAX_HUFF_SYMBOLS_0 + TDEFL_MAX_HUFF_SYMBOLS_1], prev_code_size = 0xFF;

    d->m_huff_count[0][256] = 1;

    tdefl_optimize_huffman_table(d, 0, TDEFL_MAX_HUFF_SYMBOLS_0, 15, tmz_FALSE);
    tdefl_optimize_huffman_table(d, 1, TDEFL_MAX_HUFF_SYMBOLS_1, 15, tmz_FALSE);

    for (num_lit_codes = 286; num_lit_codes > 257; num_lit_codes--)
        if (d->m_huff_code_sizes[0][num_lit_codes - 1])
            break;
    for (num_dist_codes = 30; num_dist_codes > 1; num_dist_codes--)
        if (d->m_huff_code_sizes[1][num_dist_codes - 1])
            break;

    memcpy(code_sizes_to_pack, &d->m_huff_code_sizes[0][0], num_lit_codes);
    memcpy(code_sizes_to_pack + num_lit_codes, &d->m_huff_code_sizes[1][0], num_dist_codes);
    total_code_sizes_to_pack = num_lit_codes + num_dist_codes;
    num_packed_code_sizes = 0;
    rle_z_count = 0;
    rle_repeat_count = 0;

    memset(&d->m_huff_count[2][0], 0, sizeof(d->m_huff_count[2][0]) * TDEFL_MAX_HUFF_SYMBOLS_2);
    for (i = 0; i < total_code_sizes_to_pack; i++)
    {
        tmz_uint8 code_size = code_sizes_to_pack[i];
        if (!code_size)
        {
            TDEFL_RLE_PREV_CODE_SIZE();
            if (++rle_z_count == 138)
            {
                TDEFL_RLE_ZERO_CODE_SIZE();
            }
        }
        else
        {
            TDEFL_RLE_ZERO_CODE_SIZE();
            if (code_size != prev_code_size)
            {
                TDEFL_RLE_PREV_CODE_SIZE();
                d->m_huff_count[2][code_size] = (tmz_uint16)(d->m_huff_count[2][code_size] + 1);
                packed_code_sizes[num_packed_code_sizes++] = code_size;
            }
            else if (++rle_repeat_count == 6)
            {
                TDEFL_RLE_PREV_CODE_SIZE();
            }
        }
        prev_code_size = code_size;
    }
    if (rle_repeat_count)
    {
        TDEFL_RLE_PREV_CODE_SIZE();
    }
    else
    {
        TDEFL_RLE_ZERO_CODE_SIZE();
    }

    tdefl_optimize_huffman_table(d, 2, TDEFL_MAX_HUFF_SYMBOLS_2, 7, tmz_FALSE);

    TDEFL_PUT_BITS(2, 2);

    TDEFL_PUT_BITS(num_lit_codes - 257, 5);
    TDEFL_PUT_BITS(num_dist_codes - 1, 5);

    for (num_bit_lengths = 18; num_bit_lengths >= 0; num_bit_lengths--)
        if (d->m_huff_code_sizes[2][s_tdefl_packed_code_size_syms_swizzle[num_bit_lengths]])
            break;
    num_bit_lengths = tmz_MAX(4, (num_bit_lengths + 1));
    TDEFL_PUT_BITS(num_bit_lengths - 4, 4);
    for (i = 0; (int)i < num_bit_lengths; i++)
        TDEFL_PUT_BITS(d->m_huff_code_sizes[2][s_tdefl_packed_code_size_syms_swizzle[i]], 3);

    for (packed_code_sizes_index = 0; packed_code_sizes_index < num_packed_code_sizes;)
    {
        tmz_uint code = packed_code_sizes[packed_code_sizes_index++];
        tmz_ASSERT(code < TDEFL_MAX_HUFF_SYMBOLS_2);
        TDEFL_PUT_BITS(d->m_huff_codes[2][code], d->m_huff_code_sizes[2][code]);
        if (code >= 16)
            TDEFL_PUT_BITS(packed_code_sizes[packed_code_sizes_index++], "\02\03\07"[code - 16]);
    }
}

static void tdefl_start_static_block(tdefl_compressor *d)
{
    tmz_uint i;
    tmz_uint8 *p = &d->m_huff_code_sizes[0][0];

    for (i = 0; i <= 143; ++i)
        *p++ = 8;
    for (; i <= 255; ++i)
        *p++ = 9;
    for (; i <= 279; ++i)
        *p++ = 7;
    for (; i <= 287; ++i)
        *p++ = 8;

    memset(d->m_huff_code_sizes[1], 5, 32);

    tdefl_optimize_huffman_table(d, 0, 288, 15, tmz_TRUE);
    tdefl_optimize_huffman_table(d, 1, 32, 15, tmz_TRUE);

    TDEFL_PUT_BITS(1, 2);
}

static const tmz_uint tmz_bitmasks[17] = { 0x0000, 0x0001, 0x0003, 0x0007, 0x000F, 0x001F, 0x003F, 0x007F, 0x00FF, 0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF };

#if tminiz_USE_UNALIGNED_LOADS_AND_STORES &&tminiz_LITTLE_ENDIAN &&tminiz_HAS_64BIT_REGISTERS
static tmz_bool tdefl_compress_lz_codes(tdefl_compressor *d)
{
    tmz_uint flags;
    tmz_uint8 *pLZ_codes;
    tmz_uint8 *pOutput_buf = d->m_pOutput_buf;
    tmz_uint8 *pLZ_code_buf_end = d->m_pLZ_code_buf;
    tmz_uint64 bit_buffer = d->m_bit_buffer;
    tmz_uint bits_in = d->m_bits_in;

#define TDEFL_PUT_BITS_FAST(b, l)                    \
    {                                                \
        bit_buffer |= (((tmz_uint64)(b)) << bits_in); \
        bits_in += (l);                              \
    }

    flags = 1;
    for (pLZ_codes = d->m_lz_code_buf; pLZ_codes < pLZ_code_buf_end; flags >>= 1)
    {
        if (flags == 1)
            flags = *pLZ_codes++ | 0x100;

        if (flags & 1)
        {
            tmz_uint s0, s1, n0, n1, sym, num_extra_bits;
            tmz_uint match_len = pLZ_codes[0], match_dist = *(const tmz_uint16 *)(pLZ_codes + 1);
            pLZ_codes += 3;

            tmz_ASSERT(d->m_huff_code_sizes[0][s_tdefl_len_sym[match_len]]);
            TDEFL_PUT_BITS_FAST(d->m_huff_codes[0][s_tdefl_len_sym[match_len]], d->m_huff_code_sizes[0][s_tdefl_len_sym[match_len]]);
            TDEFL_PUT_BITS_FAST(match_len & tmz_bitmasks[s_tdefl_len_extra[match_len]], s_tdefl_len_extra[match_len]);

            /* This sequence coaxes MSVC into using cmov's vs. jmp's. */
            s0 = s_tdefl_small_dist_sym[match_dist & 511];
            n0 = s_tdefl_small_dist_extra[match_dist & 511];
            s1 = s_tdefl_large_dist_sym[match_dist >> 8];
            n1 = s_tdefl_large_dist_extra[match_dist >> 8];
            sym = (match_dist < 512) ? s0 : s1;
            num_extra_bits = (match_dist < 512) ? n0 : n1;

            tmz_ASSERT(d->m_huff_code_sizes[1][sym]);
            TDEFL_PUT_BITS_FAST(d->m_huff_codes[1][sym], d->m_huff_code_sizes[1][sym]);
            TDEFL_PUT_BITS_FAST(match_dist & tmz_bitmasks[num_extra_bits], num_extra_bits);
        }
        else
        {
            tmz_uint lit = *pLZ_codes++;
            tmz_ASSERT(d->m_huff_code_sizes[0][lit]);
            TDEFL_PUT_BITS_FAST(d->m_huff_codes[0][lit], d->m_huff_code_sizes[0][lit]);

            if (((flags & 2) == 0) && (pLZ_codes < pLZ_code_buf_end))
            {
                flags >>= 1;
                lit = *pLZ_codes++;
                tmz_ASSERT(d->m_huff_code_sizes[0][lit]);
                TDEFL_PUT_BITS_FAST(d->m_huff_codes[0][lit], d->m_huff_code_sizes[0][lit]);

                if (((flags & 2) == 0) && (pLZ_codes < pLZ_code_buf_end))
                {
                    flags >>= 1;
                    lit = *pLZ_codes++;
                    tmz_ASSERT(d->m_huff_code_sizes[0][lit]);
                    TDEFL_PUT_BITS_FAST(d->m_huff_codes[0][lit], d->m_huff_code_sizes[0][lit]);
                }
            }
        }

        if (pOutput_buf >= d->m_pOutput_buf_end)
            return tmz_FALSE;

        *(tmz_uint64 *)pOutput_buf = bit_buffer;
        pOutput_buf += (bits_in >> 3);
        bit_buffer >>= (bits_in & ~7);
        bits_in &= 7;
    }

#undef TDEFL_PUT_BITS_FAST

    d->m_pOutput_buf = pOutput_buf;
    d->m_bits_in = 0;
    d->m_bit_buffer = 0;

    while (bits_in)
    {
        tmz_uint32 n = tmz_MIN(bits_in, 16);
        TDEFL_PUT_BITS((tmz_uint)bit_buffer & tmz_bitmasks[n], n);
        bit_buffer >>= n;
        bits_in -= n;
    }

    TDEFL_PUT_BITS(d->m_huff_codes[0][256], d->m_huff_code_sizes[0][256]);

    return (d->m_pOutput_buf < d->m_pOutput_buf_end);
}
#else
static tmz_bool tdefl_compress_lz_codes(tdefl_compressor *d)
{
    tmz_uint flags;
    tmz_uint8 *pLZ_codes;

    flags = 1;
    for (pLZ_codes = d->m_lz_code_buf; pLZ_codes < d->m_pLZ_code_buf; flags >>= 1)
    {
        if (flags == 1)
            flags = *pLZ_codes++ | 0x100;
        if (flags & 1)
        {
            tmz_uint sym, num_extra_bits;
            tmz_uint match_len = pLZ_codes[0], match_dist = (pLZ_codes[1] | (pLZ_codes[2] << 8));
            pLZ_codes += 3;

            tmz_ASSERT(d->m_huff_code_sizes[0][s_tdefl_len_sym[match_len]]);
            TDEFL_PUT_BITS(d->m_huff_codes[0][s_tdefl_len_sym[match_len]], d->m_huff_code_sizes[0][s_tdefl_len_sym[match_len]]);
            TDEFL_PUT_BITS(match_len & tmz_bitmasks[s_tdefl_len_extra[match_len]], s_tdefl_len_extra[match_len]);

            if (match_dist < 512)
            {
                sym = s_tdefl_small_dist_sym[match_dist];
                num_extra_bits = s_tdefl_small_dist_extra[match_dist];
            }
            else
            {
                sym = s_tdefl_large_dist_sym[match_dist >> 8];
                num_extra_bits = s_tdefl_large_dist_extra[match_dist >> 8];
            }
            tmz_ASSERT(d->m_huff_code_sizes[1][sym]);
            TDEFL_PUT_BITS(d->m_huff_codes[1][sym], d->m_huff_code_sizes[1][sym]);
            TDEFL_PUT_BITS(match_dist & tmz_bitmasks[num_extra_bits], num_extra_bits);
        }
        else
        {
            tmz_uint lit = *pLZ_codes++;
            tmz_ASSERT(d->m_huff_code_sizes[0][lit]);
            TDEFL_PUT_BITS(d->m_huff_codes[0][lit], d->m_huff_code_sizes[0][lit]);
        }
    }

    TDEFL_PUT_BITS(d->m_huff_codes[0][256], d->m_huff_code_sizes[0][256]);

    return (d->m_pOutput_buf < d->m_pOutput_buf_end);
}
#endif /* tminiz_USE_UNALIGNED_LOADS_AND_STORES && tminiz_LITTLE_ENDIAN && tminiz_HAS_64BIT_REGISTERS */

static tmz_bool tdefl_compress_block(tdefl_compressor *d, tmz_bool static_block)
{
    if (static_block)
        tdefl_start_static_block(d);
    else
        tdefl_start_dynamic_block(d);
    return tdefl_compress_lz_codes(d);
}

static int tdefl_flush_block(tdefl_compressor *d, int flush)
{
    tmz_uint saved_bit_buf, saved_bits_in;
    tmz_uint8 *pSaved_output_buf;
    tmz_bool comp_block_succeeded = tmz_FALSE;
    int n, use_raw_block = ((d->m_flags & TDEFL_FORCE_ALL_RAW_BLOCKS) != 0) && (d->m_lookahead_pos - d->m_lz_code_buf_dict_pos) <= d->m_dict_size;
    tmz_uint8 *pOutput_buf_start = ((d->m_pPut_buf_func == NULL) && ((*d->m_pOut_buf_size - d->m_out_buf_ofs) >= TDEFL_OUT_BUF_SIZE)) ? ((tmz_uint8 *)d->m_pOut_buf + d->m_out_buf_ofs) : d->m_output_buf;

    d->m_pOutput_buf = pOutput_buf_start;
    d->m_pOutput_buf_end = d->m_pOutput_buf + TDEFL_OUT_BUF_SIZE - 16;

    tmz_ASSERT(!d->m_output_flush_remaining);
    d->m_output_flush_ofs = 0;
    d->m_output_flush_remaining = 0;

    *d->m_pLZ_flags = (tmz_uint8)(*d->m_pLZ_flags >> d->m_num_flags_left);
    d->m_pLZ_code_buf -= (d->m_num_flags_left == 8);

    if ((d->m_flags & TDEFL_WRITE_ZLIB_HEADER) && (!d->m_block_index))
    {
        TDEFL_PUT_BITS(0x78, 8);
        TDEFL_PUT_BITS(0x01, 8);
    }

    TDEFL_PUT_BITS(flush == TDEFL_FINISH, 1);

    pSaved_output_buf = d->m_pOutput_buf;
    saved_bit_buf = d->m_bit_buffer;
    saved_bits_in = d->m_bits_in;

    if (!use_raw_block)
        comp_block_succeeded = tdefl_compress_block(d, (d->m_flags & TDEFL_FORCE_ALL_STATIC_BLOCKS) || (d->m_total_lz_bytes < 48));

    /* If the block gets expanded, forget the current contents of the output buffer and send a raw block instead. */
    if (((use_raw_block) || ((d->m_total_lz_bytes) && ((d->m_pOutput_buf - pSaved_output_buf + 1U) >= d->m_total_lz_bytes))) &&
        ((d->m_lookahead_pos - d->m_lz_code_buf_dict_pos) <= d->m_dict_size))
    {
        tmz_uint i;
        d->m_pOutput_buf = pSaved_output_buf;
        d->m_bit_buffer = saved_bit_buf, d->m_bits_in = saved_bits_in;
        TDEFL_PUT_BITS(0, 2);
        if (d->m_bits_in)
        {
            TDEFL_PUT_BITS(0, 8 - d->m_bits_in);
        }
        for (i = 2; i; --i, d->m_total_lz_bytes ^= 0xFFFF)
        {
            TDEFL_PUT_BITS(d->m_total_lz_bytes & 0xFFFF, 16);
        }
        for (i = 0; i < d->m_total_lz_bytes; ++i)
        {
            TDEFL_PUT_BITS(d->m_dict[(d->m_lz_code_buf_dict_pos + i) & TDEFL_LZ_DICT_SIZE_MASK], 8);
        }
    }
    /* Check for the extremely unlikely (if not impossible) case of the compressed block not fitting into the output buffer when using dynamic codes. */
    else if (!comp_block_succeeded)
    {
        d->m_pOutput_buf = pSaved_output_buf;
        d->m_bit_buffer = saved_bit_buf, d->m_bits_in = saved_bits_in;
        tdefl_compress_block(d, tmz_TRUE);
    }

    if (flush)
    {
        if (flush == TDEFL_FINISH)
        {
            if (d->m_bits_in)
            {
                TDEFL_PUT_BITS(0, 8 - d->m_bits_in);
            }
            if (d->m_flags & TDEFL_WRITE_ZLIB_HEADER)
            {
                tmz_uint i, a = d->m_adler32;
                for (i = 0; i < 4; i++)
                {
                    TDEFL_PUT_BITS((a >> 24) & 0xFF, 8);
                    a <<= 8;
                }
            }
        }
        else
        {
            tmz_uint i, z = 0;
            TDEFL_PUT_BITS(0, 3);
            if (d->m_bits_in)
            {
                TDEFL_PUT_BITS(0, 8 - d->m_bits_in);
            }
            for (i = 2; i; --i, z ^= 0xFFFF)
            {
                TDEFL_PUT_BITS(z & 0xFFFF, 16);
            }
        }
    }

    tmz_ASSERT(d->m_pOutput_buf < d->m_pOutput_buf_end);

    memset(&d->m_huff_count[0][0], 0, sizeof(d->m_huff_count[0][0]) * TDEFL_MAX_HUFF_SYMBOLS_0);
    memset(&d->m_huff_count[1][0], 0, sizeof(d->m_huff_count[1][0]) * TDEFL_MAX_HUFF_SYMBOLS_1);

    d->m_pLZ_code_buf = d->m_lz_code_buf + 1;
    d->m_pLZ_flags = d->m_lz_code_buf;
    d->m_num_flags_left = 8;
    d->m_lz_code_buf_dict_pos += d->m_total_lz_bytes;
    d->m_total_lz_bytes = 0;
    d->m_block_index++;

    if ((n = (int)(d->m_pOutput_buf - pOutput_buf_start)) != 0)
    {
        if (d->m_pPut_buf_func)
        {
            *d->m_pIn_buf_size = d->m_pSrc - (const tmz_uint8 *)d->m_pIn_buf;
            if (!(*d->m_pPut_buf_func)(d->m_output_buf, n, d->m_pPut_buf_user))
                return (d->m_prev_return_status = TDEFL_STATUS_PUT_BUF_FAILED);
        }
        else if (pOutput_buf_start == d->m_output_buf)
        {
            int bytes_to_copy = (int)tmz_MIN((size_t)n, (size_t)(*d->m_pOut_buf_size - d->m_out_buf_ofs));
            memcpy((tmz_uint8 *)d->m_pOut_buf + d->m_out_buf_ofs, d->m_output_buf, bytes_to_copy);
            d->m_out_buf_ofs += bytes_to_copy;
            if ((n -= bytes_to_copy) != 0)
            {
                d->m_output_flush_ofs = bytes_to_copy;
                d->m_output_flush_remaining = n;
            }
        }
        else
        {
            d->m_out_buf_ofs += n;
        }
    }

    return d->m_output_flush_remaining;
}

#if tminiz_USE_UNALIGNED_LOADS_AND_STORES
#define TDEFL_READ_UNALIGNED_WORD(p) *(const tmz_uint16 *)(p)
static tmz_FORCEINLINE void tdefl_find_match(tdefl_compressor *d, tmz_uint lookahead_pos, tmz_uint max_dist, tmz_uint max_match_len, tmz_uint *pMatch_dist, tmz_uint *pMatch_len)
{
    tmz_uint dist, pos = lookahead_pos & TDEFL_LZ_DICT_SIZE_MASK, match_len = *pMatch_len, probe_pos = pos, next_probe_pos, probe_len;
    tmz_uint num_probes_left = d->m_max_probes[match_len >= 32];
    const tmz_uint16 *s = (const tmz_uint16 *)(d->m_dict + pos), *p, *q;
    tmz_uint16 c01 = TDEFL_READ_UNALIGNED_WORD(&d->m_dict[pos + match_len - 1]), s01 = TDEFL_READ_UNALIGNED_WORD(s);
    tmz_ASSERT(max_match_len <= TDEFL_MAX_MATCH_LEN);
    if (max_match_len <= match_len)
        return;
    for (;;)
    {
        for (;;)
        {
            if (--num_probes_left == 0)
                return;
#define TDEFL_PROBE                                                                             \
    next_probe_pos = d->m_next[probe_pos];                                                      \
    if ((!next_probe_pos) || ((dist = (tmz_uint16)(lookahead_pos - next_probe_pos)) > max_dist)) \
        return;                                                                                 \
    probe_pos = next_probe_pos & TDEFL_LZ_DICT_SIZE_MASK;                                       \
    if (TDEFL_READ_UNALIGNED_WORD(&d->m_dict[probe_pos + match_len - 1]) == c01)                \
        break;
            TDEFL_PROBE;
            TDEFL_PROBE;
            TDEFL_PROBE;
        }
        if (!dist)
            break;
        q = (const tmz_uint16 *)(d->m_dict + probe_pos);
        if (TDEFL_READ_UNALIGNED_WORD(q) != s01)
            continue;
        p = s;
        probe_len = 32;
        do
        {
        } while ((TDEFL_READ_UNALIGNED_WORD(++p) == TDEFL_READ_UNALIGNED_WORD(++q)) && (TDEFL_READ_UNALIGNED_WORD(++p) == TDEFL_READ_UNALIGNED_WORD(++q)) &&
                 (TDEFL_READ_UNALIGNED_WORD(++p) == TDEFL_READ_UNALIGNED_WORD(++q)) && (TDEFL_READ_UNALIGNED_WORD(++p) == TDEFL_READ_UNALIGNED_WORD(++q)) && (--probe_len > 0));
        if (!probe_len)
        {
            *pMatch_dist = dist;
            *pMatch_len = tmz_MIN(max_match_len, (tmz_uint)TDEFL_MAX_MATCH_LEN);
            break;
        }
        else if ((probe_len = ((tmz_uint)(p - s) * 2) + (tmz_uint)(*(const tmz_uint8 *)p == *(const tmz_uint8 *)q)) > match_len)
        {
            *pMatch_dist = dist;
            if ((*pMatch_len = match_len = tmz_MIN(max_match_len, probe_len)) == max_match_len)
                break;
            c01 = TDEFL_READ_UNALIGNED_WORD(&d->m_dict[pos + match_len - 1]);
        }
    }
}
#else
static tmz_FORCEINLINE void tdefl_find_match(tdefl_compressor *d, tmz_uint lookahead_pos, tmz_uint max_dist, tmz_uint max_match_len, tmz_uint *pMatch_dist, tmz_uint *pMatch_len)
{
    tmz_uint dist, pos = lookahead_pos & TDEFL_LZ_DICT_SIZE_MASK, match_len = *pMatch_len, probe_pos = pos, next_probe_pos, probe_len;
    tmz_uint num_probes_left = d->m_max_probes[match_len >= 32];
    const tmz_uint8 *s = d->m_dict + pos, *p, *q;
    tmz_uint8 c0 = d->m_dict[pos + match_len], c1 = d->m_dict[pos + match_len - 1];
    tmz_ASSERT(max_match_len <= TDEFL_MAX_MATCH_LEN);
    if (max_match_len <= match_len)
        return;
    for (;;)
    {
        for (;;)
        {
            if (--num_probes_left == 0)
                return;
#define TDEFL_PROBE                                                                               \
    next_probe_pos = d->m_next[probe_pos];                                                        \
    if ((!next_probe_pos) || ((dist = (tmz_uint16)(lookahead_pos - next_probe_pos)) > max_dist))   \
        return;                                                                                   \
    probe_pos = next_probe_pos & TDEFL_LZ_DICT_SIZE_MASK;                                         \
    if ((d->m_dict[probe_pos + match_len] == c0) && (d->m_dict[probe_pos + match_len - 1] == c1)) \
        break;
            TDEFL_PROBE;
            TDEFL_PROBE;
            TDEFL_PROBE;
        }
        if (!dist)
            break;
        p = s;
        q = d->m_dict + probe_pos;
        for (probe_len = 0; probe_len < max_match_len; probe_len++)
            if (*p++ != *q++)
                break;
        if (probe_len > match_len)
        {
            *pMatch_dist = dist;
            if ((*pMatch_len = match_len = probe_len) == max_match_len)
                return;
            c0 = d->m_dict[pos + match_len];
            c1 = d->m_dict[pos + match_len - 1];
        }
    }
}
#endif /* #if tminiz_USE_UNALIGNED_LOADS_AND_STORES */

#if tminiz_USE_UNALIGNED_LOADS_AND_STORES &&tminiz_LITTLE_ENDIAN
static tmz_bool tdefl_compress_fast(tdefl_compressor *d)
{
    /* Faster, minimally featured LZRW1-style match+parse loop with better register utilization. Intended for applications where raw throughput is valued more highly than ratio. */
    tmz_uint lookahead_pos = d->m_lookahead_pos, lookahead_size = d->m_lookahead_size, dict_size = d->m_dict_size, total_lz_bytes = d->m_total_lz_bytes, num_flags_left = d->m_num_flags_left;
    tmz_uint8 *pLZ_code_buf = d->m_pLZ_code_buf, *pLZ_flags = d->m_pLZ_flags;
    tmz_uint cur_pos = lookahead_pos & TDEFL_LZ_DICT_SIZE_MASK;

    while ((d->m_src_buf_left) || ((d->m_flush) && (lookahead_size)))
    {
        const tmz_uint TDEFL_COMP_FAST_LOOKAHEAD_SIZE = 4096;
        tmz_uint dst_pos = (lookahead_pos + lookahead_size) & TDEFL_LZ_DICT_SIZE_MASK;
        tmz_uint num_bytes_to_process = (tmz_uint)tmz_MIN(d->m_src_buf_left, TDEFL_COMP_FAST_LOOKAHEAD_SIZE - lookahead_size);
        d->m_src_buf_left -= num_bytes_to_process;
        lookahead_size += num_bytes_to_process;

        while (num_bytes_to_process)
        {
            tmz_uint32 n = tmz_MIN(TDEFL_LZ_DICT_SIZE - dst_pos, num_bytes_to_process);
            memcpy(d->m_dict + dst_pos, d->m_pSrc, n);
            if (dst_pos < (TDEFL_MAX_MATCH_LEN - 1))
                memcpy(d->m_dict + TDEFL_LZ_DICT_SIZE + dst_pos, d->m_pSrc, tmz_MIN(n, (TDEFL_MAX_MATCH_LEN - 1) - dst_pos));
            d->m_pSrc += n;
            dst_pos = (dst_pos + n) & TDEFL_LZ_DICT_SIZE_MASK;
            num_bytes_to_process -= n;
        }

        dict_size = tmz_MIN(TDEFL_LZ_DICT_SIZE - lookahead_size, dict_size);
        if ((!d->m_flush) && (lookahead_size < TDEFL_COMP_FAST_LOOKAHEAD_SIZE))
            break;

        while (lookahead_size >= 4)
        {
            tmz_uint cur_match_dist, cur_match_len = 1;
            tmz_uint8 *pCur_dict = d->m_dict + cur_pos;
            tmz_uint first_trigram = (*(const tmz_uint32 *)pCur_dict) & 0xFFFFFF;
            tmz_uint hash = (first_trigram ^ (first_trigram >> (24 - (TDEFL_LZ_HASH_BITS - 8)))) & TDEFL_LEVEL1_HASH_SIZE_MASK;
            tmz_uint probe_pos = d->m_hash[hash];
            d->m_hash[hash] = (tmz_uint16)lookahead_pos;

            if (((cur_match_dist = (tmz_uint16)(lookahead_pos - probe_pos)) <= dict_size) && ((*(const tmz_uint32 *)(d->m_dict + (probe_pos &= TDEFL_LZ_DICT_SIZE_MASK)) & 0xFFFFFF) == first_trigram))
            {
                const tmz_uint16 *p = (const tmz_uint16 *)pCur_dict;
                const tmz_uint16 *q = (const tmz_uint16 *)(d->m_dict + probe_pos);
                tmz_uint32 probe_len = 32;
                do
                {
                } while ((TDEFL_READ_UNALIGNED_WORD(++p) == TDEFL_READ_UNALIGNED_WORD(++q)) && (TDEFL_READ_UNALIGNED_WORD(++p) == TDEFL_READ_UNALIGNED_WORD(++q)) &&
                         (TDEFL_READ_UNALIGNED_WORD(++p) == TDEFL_READ_UNALIGNED_WORD(++q)) && (TDEFL_READ_UNALIGNED_WORD(++p) == TDEFL_READ_UNALIGNED_WORD(++q)) && (--probe_len > 0));
                cur_match_len = ((tmz_uint)(p - (const tmz_uint16 *)pCur_dict) * 2) + (tmz_uint)(*(const tmz_uint8 *)p == *(const tmz_uint8 *)q);
                if (!probe_len)
                    cur_match_len = cur_match_dist ? TDEFL_MAX_MATCH_LEN : 0;

                if ((cur_match_len < TDEFL_MIN_MATCH_LEN) || ((cur_match_len == TDEFL_MIN_MATCH_LEN) && (cur_match_dist >= 8U * 1024U)))
                {
                    cur_match_len = 1;
                    *pLZ_code_buf++ = (tmz_uint8)first_trigram;
                    *pLZ_flags = (tmz_uint8)(*pLZ_flags >> 1);
                    d->m_huff_count[0][(tmz_uint8)first_trigram]++;
                }
                else
                {
                    tmz_uint32 s0, s1;
                    cur_match_len = tmz_MIN(cur_match_len, lookahead_size);

                    tmz_ASSERT((cur_match_len >= TDEFL_MIN_MATCH_LEN) && (cur_match_dist >= 1) && (cur_match_dist <= TDEFL_LZ_DICT_SIZE));

                    cur_match_dist--;

                    pLZ_code_buf[0] = (tmz_uint8)(cur_match_len - TDEFL_MIN_MATCH_LEN);
                    *(tmz_uint16 *)(&pLZ_code_buf[1]) = (tmz_uint16)cur_match_dist;
                    pLZ_code_buf += 3;
                    *pLZ_flags = (tmz_uint8)((*pLZ_flags >> 1) | 0x80);

                    s0 = s_tdefl_small_dist_sym[cur_match_dist & 511];
                    s1 = s_tdefl_large_dist_sym[cur_match_dist >> 8];
                    d->m_huff_count[1][(cur_match_dist < 512) ? s0 : s1]++;

                    d->m_huff_count[0][s_tdefl_len_sym[cur_match_len - TDEFL_MIN_MATCH_LEN]]++;
                }
            }
            else
            {
                *pLZ_code_buf++ = (tmz_uint8)first_trigram;
                *pLZ_flags = (tmz_uint8)(*pLZ_flags >> 1);
                d->m_huff_count[0][(tmz_uint8)first_trigram]++;
            }

            if (--num_flags_left == 0)
            {
                num_flags_left = 8;
                pLZ_flags = pLZ_code_buf++;
            }

            total_lz_bytes += cur_match_len;
            lookahead_pos += cur_match_len;
            dict_size = tmz_MIN(dict_size + cur_match_len, (tmz_uint)TDEFL_LZ_DICT_SIZE);
            cur_pos = (cur_pos + cur_match_len) & TDEFL_LZ_DICT_SIZE_MASK;
            tmz_ASSERT(lookahead_size >= cur_match_len);
            lookahead_size -= cur_match_len;

            if (pLZ_code_buf > &d->m_lz_code_buf[TDEFL_LZ_CODE_BUF_SIZE - 8])
            {
                int n;
                d->m_lookahead_pos = lookahead_pos;
                d->m_lookahead_size = lookahead_size;
                d->m_dict_size = dict_size;
                d->m_total_lz_bytes = total_lz_bytes;
                d->m_pLZ_code_buf = pLZ_code_buf;
                d->m_pLZ_flags = pLZ_flags;
                d->m_num_flags_left = num_flags_left;
                if ((n = tdefl_flush_block(d, 0)) != 0)
                    return (n < 0) ? tmz_FALSE : tmz_TRUE;
                total_lz_bytes = d->m_total_lz_bytes;
                pLZ_code_buf = d->m_pLZ_code_buf;
                pLZ_flags = d->m_pLZ_flags;
                num_flags_left = d->m_num_flags_left;
            }
        }

        while (lookahead_size)
        {
            tmz_uint8 lit = d->m_dict[cur_pos];

            total_lz_bytes++;
            *pLZ_code_buf++ = lit;
            *pLZ_flags = (tmz_uint8)(*pLZ_flags >> 1);
            if (--num_flags_left == 0)
            {
                num_flags_left = 8;
                pLZ_flags = pLZ_code_buf++;
            }

            d->m_huff_count[0][lit]++;

            lookahead_pos++;
            dict_size = tmz_MIN(dict_size + 1, (tmz_uint)TDEFL_LZ_DICT_SIZE);
            cur_pos = (cur_pos + 1) & TDEFL_LZ_DICT_SIZE_MASK;
            lookahead_size--;

            if (pLZ_code_buf > &d->m_lz_code_buf[TDEFL_LZ_CODE_BUF_SIZE - 8])
            {
                int n;
                d->m_lookahead_pos = lookahead_pos;
                d->m_lookahead_size = lookahead_size;
                d->m_dict_size = dict_size;
                d->m_total_lz_bytes = total_lz_bytes;
                d->m_pLZ_code_buf = pLZ_code_buf;
                d->m_pLZ_flags = pLZ_flags;
                d->m_num_flags_left = num_flags_left;
                if ((n = tdefl_flush_block(d, 0)) != 0)
                    return (n < 0) ? tmz_FALSE : tmz_TRUE;
                total_lz_bytes = d->m_total_lz_bytes;
                pLZ_code_buf = d->m_pLZ_code_buf;
                pLZ_flags = d->m_pLZ_flags;
                num_flags_left = d->m_num_flags_left;
            }
        }
    }

    d->m_lookahead_pos = lookahead_pos;
    d->m_lookahead_size = lookahead_size;
    d->m_dict_size = dict_size;
    d->m_total_lz_bytes = total_lz_bytes;
    d->m_pLZ_code_buf = pLZ_code_buf;
    d->m_pLZ_flags = pLZ_flags;
    d->m_num_flags_left = num_flags_left;
    return tmz_TRUE;
}
#endif /* tminiz_USE_UNALIGNED_LOADS_AND_STORES && tminiz_LITTLE_ENDIAN */

static tmz_FORCEINLINE void tdefl_record_literal(tdefl_compressor *d, tmz_uint8 lit)
{
    d->m_total_lz_bytes++;
    *d->m_pLZ_code_buf++ = lit;
    *d->m_pLZ_flags = (tmz_uint8)(*d->m_pLZ_flags >> 1);
    if (--d->m_num_flags_left == 0)
    {
        d->m_num_flags_left = 8;
        d->m_pLZ_flags = d->m_pLZ_code_buf++;
    }
    d->m_huff_count[0][lit]++;
}

static tmz_FORCEINLINE void tdefl_record_match(tdefl_compressor *d, tmz_uint match_len, tmz_uint match_dist)
{
    tmz_uint32 s0, s1;

    tmz_ASSERT((match_len >= TDEFL_MIN_MATCH_LEN) && (match_dist >= 1) && (match_dist <= TDEFL_LZ_DICT_SIZE));

    d->m_total_lz_bytes += match_len;

    d->m_pLZ_code_buf[0] = (tmz_uint8)(match_len - TDEFL_MIN_MATCH_LEN);

    match_dist -= 1;
    d->m_pLZ_code_buf[1] = (tmz_uint8)(match_dist & 0xFF);
    d->m_pLZ_code_buf[2] = (tmz_uint8)(match_dist >> 8);
    d->m_pLZ_code_buf += 3;

    *d->m_pLZ_flags = (tmz_uint8)((*d->m_pLZ_flags >> 1) | 0x80);
    if (--d->m_num_flags_left == 0)
    {
        d->m_num_flags_left = 8;
        d->m_pLZ_flags = d->m_pLZ_code_buf++;
    }

    s0 = s_tdefl_small_dist_sym[match_dist & 511];
    s1 = s_tdefl_large_dist_sym[(match_dist >> 8) & 127];
    d->m_huff_count[1][(match_dist < 512) ? s0 : s1]++;

    if (match_len >= TDEFL_MIN_MATCH_LEN)
        d->m_huff_count[0][s_tdefl_len_sym[match_len - TDEFL_MIN_MATCH_LEN]]++;
}

static tmz_bool tdefl_compress_normal(tdefl_compressor *d)
{
    const tmz_uint8 *pSrc = d->m_pSrc;
    size_t src_buf_left = d->m_src_buf_left;
    tdefl_flush flush = d->m_flush;

    while ((src_buf_left) || ((flush) && (d->m_lookahead_size)))
    {
        tmz_uint len_to_move, cur_match_dist, cur_match_len, cur_pos;
        /* Update dictionary and hash chains. Keeps the lookahead size equal to TDEFL_MAX_MATCH_LEN. */
        if ((d->m_lookahead_size + d->m_dict_size) >= (TDEFL_MIN_MATCH_LEN - 1))
        {
            tmz_uint dst_pos = (d->m_lookahead_pos + d->m_lookahead_size) & TDEFL_LZ_DICT_SIZE_MASK, ins_pos = d->m_lookahead_pos + d->m_lookahead_size - 2;
            tmz_uint hash = (d->m_dict[ins_pos & TDEFL_LZ_DICT_SIZE_MASK] << TDEFL_LZ_HASH_SHIFT) ^ d->m_dict[(ins_pos + 1) & TDEFL_LZ_DICT_SIZE_MASK];
            tmz_uint num_bytes_to_process = (tmz_uint)tmz_MIN(src_buf_left, TDEFL_MAX_MATCH_LEN - d->m_lookahead_size);
            const tmz_uint8 *pSrc_end = pSrc + num_bytes_to_process;
            src_buf_left -= num_bytes_to_process;
            d->m_lookahead_size += num_bytes_to_process;
            while (pSrc != pSrc_end)
            {
                tmz_uint8 c = *pSrc++;
                d->m_dict[dst_pos] = c;
                if (dst_pos < (TDEFL_MAX_MATCH_LEN - 1))
                    d->m_dict[TDEFL_LZ_DICT_SIZE + dst_pos] = c;
                hash = ((hash << TDEFL_LZ_HASH_SHIFT) ^ c) & (TDEFL_LZ_HASH_SIZE - 1);
                d->m_next[ins_pos & TDEFL_LZ_DICT_SIZE_MASK] = d->m_hash[hash];
                d->m_hash[hash] = (tmz_uint16)(ins_pos);
                dst_pos = (dst_pos + 1) & TDEFL_LZ_DICT_SIZE_MASK;
                ins_pos++;
            }
        }
        else
        {
            while ((src_buf_left) && (d->m_lookahead_size < TDEFL_MAX_MATCH_LEN))
            {
                tmz_uint8 c = *pSrc++;
                tmz_uint dst_pos = (d->m_lookahead_pos + d->m_lookahead_size) & TDEFL_LZ_DICT_SIZE_MASK;
                src_buf_left--;
                d->m_dict[dst_pos] = c;
                if (dst_pos < (TDEFL_MAX_MATCH_LEN - 1))
                    d->m_dict[TDEFL_LZ_DICT_SIZE + dst_pos] = c;
                if ((++d->m_lookahead_size + d->m_dict_size) >= TDEFL_MIN_MATCH_LEN)
                {
                    tmz_uint ins_pos = d->m_lookahead_pos + (d->m_lookahead_size - 1) - 2;
                    tmz_uint hash = ((d->m_dict[ins_pos & TDEFL_LZ_DICT_SIZE_MASK] << (TDEFL_LZ_HASH_SHIFT * 2)) ^ (d->m_dict[(ins_pos + 1) & TDEFL_LZ_DICT_SIZE_MASK] << TDEFL_LZ_HASH_SHIFT) ^ c) & (TDEFL_LZ_HASH_SIZE - 1);
                    d->m_next[ins_pos & TDEFL_LZ_DICT_SIZE_MASK] = d->m_hash[hash];
                    d->m_hash[hash] = (tmz_uint16)(ins_pos);
                }
            }
        }
        d->m_dict_size = tmz_MIN(TDEFL_LZ_DICT_SIZE - d->m_lookahead_size, d->m_dict_size);
        if ((!flush) && (d->m_lookahead_size < TDEFL_MAX_MATCH_LEN))
            break;

        /* Simple lazy/greedy parsing state machine. */
        len_to_move = 1;
        cur_match_dist = 0;
        cur_match_len = d->m_saved_match_len ? d->m_saved_match_len : (TDEFL_MIN_MATCH_LEN - 1);
        cur_pos = d->m_lookahead_pos & TDEFL_LZ_DICT_SIZE_MASK;
        if (d->m_flags & (TDEFL_RLE_MATCHES | TDEFL_FORCE_ALL_RAW_BLOCKS))
        {
            if ((d->m_dict_size) && (!(d->m_flags & TDEFL_FORCE_ALL_RAW_BLOCKS)))
            {
                tmz_uint8 c = d->m_dict[(cur_pos - 1) & TDEFL_LZ_DICT_SIZE_MASK];
                cur_match_len = 0;
                while (cur_match_len < d->m_lookahead_size)
                {
                    if (d->m_dict[cur_pos + cur_match_len] != c)
                        break;
                    cur_match_len++;
                }
                if (cur_match_len < TDEFL_MIN_MATCH_LEN)
                    cur_match_len = 0;
                else
                    cur_match_dist = 1;
            }
        }
        else
        {
            tdefl_find_match(d, d->m_lookahead_pos, d->m_dict_size, d->m_lookahead_size, &cur_match_dist, &cur_match_len);
        }
        if (((cur_match_len == TDEFL_MIN_MATCH_LEN) && (cur_match_dist >= 8U * 1024U)) || (cur_pos == cur_match_dist) || ((d->m_flags & TDEFL_FILTER_MATCHES) && (cur_match_len <= 5)))
        {
            cur_match_dist = cur_match_len = 0;
        }
        if (d->m_saved_match_len)
        {
            if (cur_match_len > d->m_saved_match_len)
            {
                tdefl_record_literal(d, (tmz_uint8)d->m_saved_lit);
                if (cur_match_len >= 128)
                {
                    tdefl_record_match(d, cur_match_len, cur_match_dist);
                    d->m_saved_match_len = 0;
                    len_to_move = cur_match_len;
                }
                else
                {
                    d->m_saved_lit = d->m_dict[cur_pos];
                    d->m_saved_match_dist = cur_match_dist;
                    d->m_saved_match_len = cur_match_len;
                }
            }
            else
            {
                tdefl_record_match(d, d->m_saved_match_len, d->m_saved_match_dist);
                len_to_move = d->m_saved_match_len - 1;
                d->m_saved_match_len = 0;
            }
        }
        else if (!cur_match_dist)
            tdefl_record_literal(d, d->m_dict[tmz_MIN(cur_pos, sizeof(d->m_dict) - 1)]);
        else if ((d->m_greedy_parsing) || (d->m_flags & TDEFL_RLE_MATCHES) || (cur_match_len >= 128))
        {
            tdefl_record_match(d, cur_match_len, cur_match_dist);
            len_to_move = cur_match_len;
        }
        else
        {
            d->m_saved_lit = d->m_dict[tmz_MIN(cur_pos, sizeof(d->m_dict) - 1)];
            d->m_saved_match_dist = cur_match_dist;
            d->m_saved_match_len = cur_match_len;
        }
        /* Move the lookahead forward by len_to_move bytes. */
        d->m_lookahead_pos += len_to_move;
        tmz_ASSERT(d->m_lookahead_size >= len_to_move);
        d->m_lookahead_size -= len_to_move;
        d->m_dict_size = tmz_MIN(d->m_dict_size + len_to_move, (tmz_uint)TDEFL_LZ_DICT_SIZE);
        /* Check if it's time to flush the current LZ codes to the internal output buffer. */
        if ((d->m_pLZ_code_buf > &d->m_lz_code_buf[TDEFL_LZ_CODE_BUF_SIZE - 8]) ||
            ((d->m_total_lz_bytes > 31 * 1024) && (((((tmz_uint)(d->m_pLZ_code_buf - d->m_lz_code_buf) * 115) >> 7) >= d->m_total_lz_bytes) || (d->m_flags & TDEFL_FORCE_ALL_RAW_BLOCKS))))
        {
            int n;
            d->m_pSrc = pSrc;
            d->m_src_buf_left = src_buf_left;
            if ((n = tdefl_flush_block(d, 0)) != 0)
                return (n < 0) ? tmz_FALSE : tmz_TRUE;
        }
    }

    d->m_pSrc = pSrc;
    d->m_src_buf_left = src_buf_left;
    return tmz_TRUE;
}

static tdefl_status tdefl_flush_output_buffer(tdefl_compressor *d)
{
    if (d->m_pIn_buf_size)
    {
        *d->m_pIn_buf_size = d->m_pSrc - (const tmz_uint8 *)d->m_pIn_buf;
    }

    if (d->m_pOut_buf_size)
    {
        size_t n = tmz_MIN(*d->m_pOut_buf_size - d->m_out_buf_ofs, d->m_output_flush_remaining);
        memcpy((tmz_uint8 *)d->m_pOut_buf + d->m_out_buf_ofs, d->m_output_buf + d->m_output_flush_ofs, n);
        d->m_output_flush_ofs += (tmz_uint)n;
        d->m_output_flush_remaining -= (tmz_uint)n;
        d->m_out_buf_ofs += n;

        *d->m_pOut_buf_size = d->m_out_buf_ofs;
    }

    return (d->m_finished && !d->m_output_flush_remaining) ? TDEFL_STATUS_DONE : TDEFL_STATUS_OKAY;
}

tdefl_status tdefl_compress(tdefl_compressor *d, const void *pIn_buf, size_t *pIn_buf_size, void *pOut_buf, size_t *pOut_buf_size, tdefl_flush flush)
{
    if (!d)
    {
        if (pIn_buf_size)
            *pIn_buf_size = 0;
        if (pOut_buf_size)
            *pOut_buf_size = 0;
        return TDEFL_STATUS_BAD_PARAM;
    }

    d->m_pIn_buf = pIn_buf;
    d->m_pIn_buf_size = pIn_buf_size;
    d->m_pOut_buf = pOut_buf;
    d->m_pOut_buf_size = pOut_buf_size;
    d->m_pSrc = (const tmz_uint8 *)(pIn_buf);
    d->m_src_buf_left = pIn_buf_size ? *pIn_buf_size : 0;
    d->m_out_buf_ofs = 0;
    d->m_flush = flush;

    if (((d->m_pPut_buf_func != NULL) == ((pOut_buf != NULL) || (pOut_buf_size != NULL))) || (d->m_prev_return_status != TDEFL_STATUS_OKAY) ||
        (d->m_wants_to_finish && (flush != TDEFL_FINISH)) || (pIn_buf_size && *pIn_buf_size && !pIn_buf) || (pOut_buf_size && *pOut_buf_size && !pOut_buf))
    {
        if (pIn_buf_size)
            *pIn_buf_size = 0;
        if (pOut_buf_size)
            *pOut_buf_size = 0;
        return (d->m_prev_return_status = TDEFL_STATUS_BAD_PARAM);
    }
    d->m_wants_to_finish |= (flush == TDEFL_FINISH);

    if ((d->m_output_flush_remaining) || (d->m_finished))
        return (d->m_prev_return_status = tdefl_flush_output_buffer(d));

#if tminiz_USE_UNALIGNED_LOADS_AND_STORES &&tminiz_LITTLE_ENDIAN
    if (((d->m_flags & TDEFL_MAX_PROBES_MASK) == 1) &&
        ((d->m_flags & TDEFL_GREEDY_PARSING_FLAG) != 0) &&
        ((d->m_flags & (TDEFL_FILTER_MATCHES | TDEFL_FORCE_ALL_RAW_BLOCKS | TDEFL_RLE_MATCHES)) == 0))
    {
        if (!tdefl_compress_fast(d))
            return d->m_prev_return_status;
    }
    else
#endif /* #if tminiz_USE_UNALIGNED_LOADS_AND_STORES && tminiz_LITTLE_ENDIAN */
    {
        if (!tdefl_compress_normal(d))
            return d->m_prev_return_status;
    }

    if ((d->m_flags & (TDEFL_WRITE_ZLIB_HEADER | TDEFL_COMPUTE_ADLER32)) && (pIn_buf))
        d->m_adler32 = (tmz_uint32)tmz_adler32(d->m_adler32, (const tmz_uint8 *)pIn_buf, d->m_pSrc - (const tmz_uint8 *)pIn_buf);

    if ((flush) && (!d->m_lookahead_size) && (!d->m_src_buf_left) && (!d->m_output_flush_remaining))
    {
        if (tdefl_flush_block(d, flush) < 0)
            return d->m_prev_return_status;
        d->m_finished = (flush == TDEFL_FINISH);
        if (flush == TDEFL_FULL_FLUSH)
        {
            tmz_CLEAR_OBJ(d->m_hash);
            tmz_CLEAR_OBJ(d->m_next);
            d->m_dict_size = 0;
        }
    }

    return (d->m_prev_return_status = tdefl_flush_output_buffer(d));
}

tdefl_status tdefl_compress_buffer(tdefl_compressor *d, const void *pIn_buf, size_t in_buf_size, tdefl_flush flush)
{
    tmz_ASSERT(d->m_pPut_buf_func);
    return tdefl_compress(d, pIn_buf, &in_buf_size, NULL, NULL, flush);
}

tdefl_status tdefl_init(tdefl_compressor *d, tdefl_put_buf_func_ptr pPut_buf_func, void *pPut_buf_user, int flags)
{
    d->m_pPut_buf_func = pPut_buf_func;
    d->m_pPut_buf_user = pPut_buf_user;
    d->m_flags = (tmz_uint)(flags);
    d->m_max_probes[0] = 1 + ((flags & 0xFFF) + 2) / 3;
    d->m_greedy_parsing = (flags & TDEFL_GREEDY_PARSING_FLAG) != 0;
    d->m_max_probes[1] = 1 + (((flags & 0xFFF) >> 2) + 2) / 3;
    if (!(flags & TDEFL_NONDETERMINISTIC_PARSING_FLAG))
        tmz_CLEAR_OBJ(d->m_hash);
    d->m_lookahead_pos = d->m_lookahead_size = d->m_dict_size = d->m_total_lz_bytes = d->m_lz_code_buf_dict_pos = d->m_bits_in = 0;
    d->m_output_flush_ofs = d->m_output_flush_remaining = d->m_finished = d->m_block_index = d->m_bit_buffer = d->m_wants_to_finish = 0;
    d->m_pLZ_code_buf = d->m_lz_code_buf + 1;
    d->m_pLZ_flags = d->m_lz_code_buf;
    d->m_num_flags_left = 8;
    d->m_pOutput_buf = d->m_output_buf;
    d->m_pOutput_buf_end = d->m_output_buf;
    d->m_prev_return_status = TDEFL_STATUS_OKAY;
    d->m_saved_match_dist = d->m_saved_match_len = d->m_saved_lit = 0;
    d->m_adler32 = 1;
    d->m_pIn_buf = NULL;
    d->m_pOut_buf = NULL;
    d->m_pIn_buf_size = NULL;
    d->m_pOut_buf_size = NULL;
    d->m_flush = TDEFL_NO_FLUSH;
    d->m_pSrc = NULL;
    d->m_src_buf_left = 0;
    d->m_out_buf_ofs = 0;
    memset(&d->m_huff_count[0][0], 0, sizeof(d->m_huff_count[0][0]) * TDEFL_MAX_HUFF_SYMBOLS_0);
    memset(&d->m_huff_count[1][0], 0, sizeof(d->m_huff_count[1][0]) * TDEFL_MAX_HUFF_SYMBOLS_1);
    return TDEFL_STATUS_OKAY;
}

tdefl_status tdefl_get_prev_return_status(tdefl_compressor *d)
{
    return d->m_prev_return_status;
}

tmz_uint32 tdefl_get_adler32(tdefl_compressor *d)
{
    return d->m_adler32;
}

tmz_bool tdefl_compress_mem_to_output(const void *pBuf, size_t buf_len, tdefl_put_buf_func_ptr pPut_buf_func, void *pPut_buf_user, int flags)
{
    tdefl_compressor *pComp;
    tmz_bool succeeded;
    if (((buf_len) && (!pBuf)) || (!pPut_buf_func))
        return tmz_FALSE;
    pComp = (tdefl_compressor *)tmz_MALLOC(sizeof(tdefl_compressor));
    if (!pComp)
        return tmz_FALSE;
    succeeded = (tdefl_init(pComp, pPut_buf_func, pPut_buf_user, flags) == TDEFL_STATUS_OKAY);
    succeeded = succeeded && (tdefl_compress_buffer(pComp, pBuf, buf_len, TDEFL_FINISH) == TDEFL_STATUS_DONE);
    tmz_FREE(pComp);
    return succeeded;
}

typedef struct
{
    size_t m_size, m_capacity;
    tmz_uint8 *m_pBuf;
    tmz_bool m_expandable;
} tdefl_output_buffer;

static tmz_bool tdefl_output_buffer_putter(const void *pBuf, int len, void *pUser)
{
    tdefl_output_buffer *p = (tdefl_output_buffer *)pUser;
    size_t new_size = p->m_size + len;
    if (new_size > p->m_capacity)
    {
        size_t new_capacity = p->m_capacity;
        tmz_uint8 *pNew_buf;
        if (!p->m_expandable)
            return tmz_FALSE;
        do
        {
            new_capacity = tmz_MAX(128U, new_capacity << 1U);
        } while (new_size > new_capacity);
        pNew_buf = (tmz_uint8 *)tmz_REALLOC(p->m_pBuf, new_capacity);
        if (!pNew_buf)
            return tmz_FALSE;
        p->m_pBuf = pNew_buf;
        p->m_capacity = new_capacity;
    }
    memcpy((tmz_uint8 *)p->m_pBuf + p->m_size, pBuf, len);
    p->m_size = new_size;
    return tmz_TRUE;
}

void *tdefl_compress_mem_to_heap(const void *pSrc_buf, size_t src_buf_len, size_t *pOut_len, int flags)
{
    tdefl_output_buffer out_buf;
    tmz_CLEAR_OBJ(out_buf);
    if (!pOut_len)
        return tmz_FALSE;
    else
        *pOut_len = 0;
    out_buf.m_expandable = tmz_TRUE;
    if (!tdefl_compress_mem_to_output(pSrc_buf, src_buf_len, tdefl_output_buffer_putter, &out_buf, flags))
        return NULL;
    *pOut_len = out_buf.m_size;
    return out_buf.m_pBuf;
}

size_t tdefl_compress_mem_to_mem(void *pOut_buf, size_t out_buf_len, const void *pSrc_buf, size_t src_buf_len, int flags)
{
    tdefl_output_buffer out_buf;
    tmz_CLEAR_OBJ(out_buf);
    if (!pOut_buf)
        return 0;
    out_buf.m_pBuf = (tmz_uint8 *)pOut_buf;
    out_buf.m_capacity = out_buf_len;
    if (!tdefl_compress_mem_to_output(pSrc_buf, src_buf_len, tdefl_output_buffer_putter, &out_buf, flags))
        return 0;
    return out_buf.m_size;
}

static const tmz_uint s_tdefl_num_probes[11] = { 0, 1, 6, 32, 16, 32, 128, 256, 512, 768, 1500 };

/* level may actually range from [0,10] (10 is a "hidden" max level, where we want a bit more compression and it's fine if throughput to fall off a cliff on some files). */
tmz_uint tdefl_create_comp_flags_from_zip_params(int level, int window_bits, int strategy)
{
    tmz_uint comp_flags = s_tdefl_num_probes[(level >= 0) ? tmz_MIN(10, level) : tmz_DEFAULT_LEVEL] | ((level <= 3) ? TDEFL_GREEDY_PARSING_FLAG : 0);
    if (window_bits > 0)
        comp_flags |= TDEFL_WRITE_ZLIB_HEADER;

    if (!level)
        comp_flags |= TDEFL_FORCE_ALL_RAW_BLOCKS;
    else if (strategy == tmz_FILTERED)
        comp_flags |= TDEFL_FILTER_MATCHES;
    else if (strategy == tmz_HUFFMAN_ONLY)
        comp_flags &= ~TDEFL_MAX_PROBES_MASK;
    else if (strategy == tmz_FIXED)
        comp_flags |= TDEFL_FORCE_ALL_STATIC_BLOCKS;
    else if (strategy == tmz_RLE)
        comp_flags |= TDEFL_RLE_MATCHES;

    return comp_flags;
}

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4204) /* nonstandard extension used : non-constant aggregate initializer (also supported by GNU C and C99, so no big deal) */
#endif

/* Simple PNG writer function by Alex Evans, 2011. Released into the public domain: https://gist.github.com/908299, more context at
 http://altdevblogaday.org/2011/04/06/a-smaller-jpg-encoder/.
 This is actually a modification of Alex's original code so PNG files generated by this function pass pngcheck. */
void *tdefl_write_image_to_png_file_in_memory_ex(const void *pImage, int w, int h, int num_chans, size_t *pLen_out, tmz_uint level, tmz_bool flip)
{
    /* Using a local copy of this array here in case tminiz_NO_ZLIB_APIS was defined. */
    static const tmz_uint s_tdefl_png_num_probes[11] = { 0, 1, 6, 32, 16, 32, 128, 256, 512, 768, 1500 };
    tdefl_compressor *pComp = (tdefl_compressor *)tmz_MALLOC(sizeof(tdefl_compressor));
    tdefl_output_buffer out_buf;
    int i, bpl = w * num_chans, y, z;
    tmz_uint32 c;
    *pLen_out = 0;
    if (!pComp)
        return NULL;
    tmz_CLEAR_OBJ(out_buf);
    out_buf.m_expandable = tmz_TRUE;
    out_buf.m_capacity = 57 + tmz_MAX(64, (1 + bpl) * h);
    if (NULL == (out_buf.m_pBuf = (tmz_uint8 *)tmz_MALLOC(out_buf.m_capacity)))
    {
        tmz_FREE(pComp);
        return NULL;
    }
    /* write dummy header */
    for (z = 41; z; --z)
        tdefl_output_buffer_putter(&z, 1, &out_buf);
    /* compress image data */
    tdefl_init(pComp, tdefl_output_buffer_putter, &out_buf, s_tdefl_png_num_probes[tmz_MIN(10, level)] | TDEFL_WRITE_ZLIB_HEADER);
    for (y = 0; y < h; ++y)
    {
        tdefl_compress_buffer(pComp, &z, 1, TDEFL_NO_FLUSH);
        tdefl_compress_buffer(pComp, (tmz_uint8 *)pImage + (flip ? (h - 1 - y) : y) * bpl, bpl, TDEFL_NO_FLUSH);
    }
    if (tdefl_compress_buffer(pComp, NULL, 0, TDEFL_FINISH) != TDEFL_STATUS_DONE)
    {
        tmz_FREE(pComp);
        tmz_FREE(out_buf.m_pBuf);
        return NULL;
    }
    /* write real header */
    *pLen_out = out_buf.m_size - 41;
    {
        static const tmz_uint8 chans[] = { 0x00, 0x00, 0x04, 0x02, 0x06 };
        tmz_uint8 pnghdr[41] = { 0x89, 0x50, 0x4e, 0x47, 0x0d, 
								0x0a, 0x1a, 0x0a, 0x00, 0x00, 
								0x00, 0x0d, 0x49, 0x48, 0x44, 
								0x52, 0x00, 0x00, 0x00, 0x00, 
								0x00, 0x00, 0x00, 0x00, 0x08, 
								0x00, 0x00, 0x00, 0x00, 0x00, 
								0x00, 0x00, 0x00, 0x00, 0x00, 
								0x00, 0x00, 0x49, 0x44, 0x41, 
								0x54 };
		pnghdr[18] = (tmz_uint8)(w >> 8);
		pnghdr[19] = (tmz_uint8)w;
		pnghdr[22] = (tmz_uint8)(h >> 8);
		pnghdr[22] = (tmz_uint8)h;
		pnghdr[25] = chans[num_chans];
		pnghdr[33] = (tmz_uint8)(*pLen_out >> 24);
		pnghdr[34] = (tmz_uint8)(*pLen_out >> 16);
		pnghdr[35] = (tmz_uint8)(*pLen_out >> 8);
		pnghdr[36] = (tmz_uint8)*pLen_out;
        c = (tmz_uint32)tmz_crc32(tmz_CRC32_INIT, pnghdr + 12, 17);
        for (i = 0; i < 4; ++i, c <<= 8)
            ((tmz_uint8 *)(pnghdr + 29))[i] = (tmz_uint8)(c >> 24);
        memcpy(out_buf.m_pBuf, pnghdr, 41);
    }
    /* write footer (IDAT CRC-32, followed by IEND chunk) */
    if (!tdefl_output_buffer_putter("\0\0\0\0\0\0\0\0\x49\x45\x4e\x44\xae\x42\x60\x82", 16, &out_buf))
    {
        *pLen_out = 0;
        tmz_FREE(pComp);
        tmz_FREE(out_buf.m_pBuf);
        return NULL;
    }
    c = (tmz_uint32)tmz_crc32(tmz_CRC32_INIT, out_buf.m_pBuf + 41 - 4, *pLen_out + 4);
    for (i = 0; i < 4; ++i, c <<= 8)
        (out_buf.m_pBuf + out_buf.m_size - 16)[i] = (tmz_uint8)(c >> 24);
    /* compute final size of file, grab compressed data buffer and return */
    *pLen_out += 57;
    tmz_FREE(pComp);
    return out_buf.m_pBuf;
}
void *tdefl_write_image_to_png_file_in_memory(const void *pImage, int w, int h, int num_chans, size_t *pLen_out)
{
    /* Level 6 corresponds to TDEFL_DEFAULT_MAX_PROBES or tmz_DEFAULT_LEVEL (but we can't depend on tmz_DEFAULT_LEVEL being available in case the zlib API's where #defined out) */
    return tdefl_write_image_to_png_file_in_memory_ex(pImage, w, h, num_chans, pLen_out, 6, tmz_FALSE);
}

/* Allocate the tdefl_compressor and tinfl_decompressor structures in C so that */
/* non-C language bindings to tdefL_ and tinfl_ API don't need to worry about */
/* structure size and allocation mechanism. */
tdefl_compressor *tdefl_compressor_alloc()
{
    return (tdefl_compressor *)tmz_MALLOC(sizeof(tdefl_compressor));
}

void tdefl_compressor_free(tdefl_compressor *pComp)
{
    tmz_FREE(pComp);
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#ifdef __cplusplus
}
#endif
/**************************************************************************
 *
 * Copyright 2013-2014 RAD Game Tools and Valve Software
 * Copyright 2010-2014 Rich Geldreich and Tenacious Software LLC
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 **************************************************************************/



#ifdef __cplusplus
extern "C" {
#endif

/* ------------------- Low-level Decompression (completely independent from all compression API's) */

#define TINFL_MEMCPY(d, s, l) memcpy(d, s, l)
#define TINFL_MEMSET(p, c, l) memset(p, c, l)

#define TINFL_CR_BEGIN  \
    switch (r->m_state) \
    {                   \
        case 0:
#define TINFL_CR_RETURN(state_index, result) \
    do                                       \
    {                                        \
        status = result;                     \
        r->m_state = state_index;            \
        goto common_exit;                    \
        case state_index:                    \
            ;                                \
    }                                        \
    tmz_MACRO_END
#define TINFL_CR_RETURN_FOREVER(state_index, result) \
    do                                               \
    {                                                \
        for (;;)                                     \
        {                                            \
            TINFL_CR_RETURN(state_index, result);    \
        }                                            \
    }                                                \
    tmz_MACRO_END
#define TINFL_CR_FINISH }

#define TINFL_GET_BYTE(state_index, c)                                                                                                                          \
    do                                                                                                                                                          \
    {                                                                                                                                                           \
        while (pIn_buf_cur >= pIn_buf_end)                                                                                                                      \
        {                                                                                                                                                       \
            TINFL_CR_RETURN(state_index, (decomp_flags &TINFL_FLAG_HAS_MORE_INPUT) ? TINFL_STATUS_NEEDS_MORE_INPUT : TINFL_STATUS_FAILED_CANNOT_MAKE_PROGRESS); \
        }                                                                                                                                                       \
        c = *pIn_buf_cur++;                                                                                                                                     \
    }                                                                                                                                                           \
    tmz_MACRO_END

#define TINFL_NEED_BITS(state_index, n)                \
    do                                                 \
    {                                                  \
        tmz_uint c;                                     \
        TINFL_GET_BYTE(state_index, c);                \
        bit_buf |= (((tinfl_bit_buf_t)c) << num_bits); \
        num_bits += 8;                                 \
    } while (num_bits < (tmz_uint)(n))
#define TINFL_SKIP_BITS(state_index, n)      \
    do                                       \
    {                                        \
        if (num_bits < (tmz_uint)(n))         \
        {                                    \
            TINFL_NEED_BITS(state_index, n); \
        }                                    \
        bit_buf >>= (n);                     \
        num_bits -= (n);                     \
    }                                        \
    tmz_MACRO_END
#define TINFL_GET_BITS(state_index, b, n)    \
    do                                       \
    {                                        \
        if (num_bits < (tmz_uint)(n))         \
        {                                    \
            TINFL_NEED_BITS(state_index, n); \
        }                                    \
        b = bit_buf & ((1 << (n)) - 1);      \
        bit_buf >>= (n);                     \
        num_bits -= (n);                     \
    }                                        \
    tmz_MACRO_END

/* TINFL_HUFF_BITBUF_FILL() is only used rarely, when the number of bytes remaining in the input buffer falls below 2. */
/* It reads just enough bytes from the input stream that are needed to decode the next Huffman code (and absolutely no more). It works by trying to fully decode a */
/* Huffman code by using whatever bits are currently present in the bit buffer. If this fails, it reads another byte, and tries again until it succeeds or until the */
/* bit buffer contains >=15 bits (deflate's max. Huffman code size). */
#define TINFL_HUFF_BITBUF_FILL(state_index, pHuff)                             \
    do                                                                         \
    {                                                                          \
        temp = (pHuff)->m_look_up[bit_buf & (TINFL_FAST_LOOKUP_SIZE - 1)];     \
        if (temp >= 0)                                                         \
        {                                                                      \
            code_len = temp >> 9;                                              \
            if ((code_len) && (num_bits >= code_len))                          \
                break;                                                         \
        }                                                                      \
        else if (num_bits > TINFL_FAST_LOOKUP_BITS)                            \
        {                                                                      \
            code_len = TINFL_FAST_LOOKUP_BITS;                                 \
            do                                                                 \
            {                                                                  \
                temp = (pHuff)->m_tree[~temp + ((bit_buf >> code_len++) & 1)]; \
            } while ((temp < 0) && (num_bits >= (code_len + 1)));              \
            if (temp >= 0)                                                     \
                break;                                                         \
        }                                                                      \
        TINFL_GET_BYTE(state_index, c);                                        \
        bit_buf |= (((tinfl_bit_buf_t)c) << num_bits);                         \
        num_bits += 8;                                                         \
    } while (num_bits < 15);

/* TINFL_HUFF_DECODE() decodes the next Huffman coded symbol. It's more complex than you would initially expect because the zlib API expects the decompressor to never read */
/* beyond the final byte of the deflate stream. (In other words, when this macro wants to read another byte from the input, it REALLY needs another byte in order to fully */
/* decode the next Huffman code.) Handling this properly is particularly important on raw deflate (non-zlib) streams, which aren't followed by a byte aligned adler-32. */
/* The slow path is only executed at the very end of the input buffer. */
/* v1.16: The original macro handled the case at the very end of the passed-in input buffer, but we also need to handle the case where the user passes in 1+zillion bytes */
/* following the deflate data and our non-conservative read-ahead path won't kick in here on this code. This is much trickier. */
#define TINFL_HUFF_DECODE(state_index, sym, pHuff)                                                                                  \
    do                                                                                                                              \
    {                                                                                                                               \
        int temp;                                                                                                                   \
        tmz_uint code_len, c;                                                                                                        \
        if (num_bits < 15)                                                                                                          \
        {                                                                                                                           \
            if ((pIn_buf_end - pIn_buf_cur) < 2)                                                                                    \
            {                                                                                                                       \
                TINFL_HUFF_BITBUF_FILL(state_index, pHuff);                                                                         \
            }                                                                                                                       \
            else                                                                                                                    \
            {                                                                                                                       \
                bit_buf |= (((tinfl_bit_buf_t)pIn_buf_cur[0]) << num_bits) | (((tinfl_bit_buf_t)pIn_buf_cur[1]) << (num_bits + 8)); \
                pIn_buf_cur += 2;                                                                                                   \
                num_bits += 16;                                                                                                     \
            }                                                                                                                       \
        }                                                                                                                           \
        if ((temp = (pHuff)->m_look_up[bit_buf & (TINFL_FAST_LOOKUP_SIZE - 1)]) >= 0)                                               \
            code_len = temp >> 9, temp &= 511;                                                                                      \
        else                                                                                                                        \
        {                                                                                                                           \
            code_len = TINFL_FAST_LOOKUP_BITS;                                                                                      \
            do                                                                                                                      \
            {                                                                                                                       \
                temp = (pHuff)->m_tree[~temp + ((bit_buf >> code_len++) & 1)];                                                      \
            } while (temp < 0);                                                                                                     \
        }                                                                                                                           \
        sym = temp;                                                                                                                 \
        bit_buf >>= code_len;                                                                                                       \
        num_bits -= code_len;                                                                                                       \
    }                                                                                                                               \
    tmz_MACRO_END

tinfl_status tinfl_decompress(tinfl_decompressor *r, const tmz_uint8 *pIn_buf_next, size_t *pIn_buf_size, tmz_uint8 *pOut_buf_start, tmz_uint8 *pOut_buf_next, size_t *pOut_buf_size, const tmz_uint32 decomp_flags)
{
    static const int s_length_base[31] = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258, 0, 0 };
    static const int s_length_extra[31] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0, 0, 0 };
    static const int s_dist_base[32] = { 1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577, 0, 0 };
    static const int s_dist_extra[32] = { 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13 };
    static const tmz_uint8 s_length_dezigzag[19] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
    static const int s_min_table_sizes[3] = { 257, 1, 4 };

    tinfl_status status = TINFL_STATUS_FAILED;
    tmz_uint32 num_bits, dist, counter, num_extra;
    tinfl_bit_buf_t bit_buf;
    const tmz_uint8 *pIn_buf_cur = pIn_buf_next, *const pIn_buf_end = pIn_buf_next + *pIn_buf_size;
    tmz_uint8 *pOut_buf_cur = pOut_buf_next, *const pOut_buf_end = pOut_buf_next + *pOut_buf_size;
    size_t out_buf_size_mask = (decomp_flags & TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF) ? (size_t) - 1 : ((pOut_buf_next - pOut_buf_start) + *pOut_buf_size) - 1, dist_from_out_buf_start;

    /* Ensure the output buffer's size is a power of 2, unless the output buffer is large enough to hold the entire output file (in which case it doesn't matter). */
    if (((out_buf_size_mask + 1) & out_buf_size_mask) || (pOut_buf_next < pOut_buf_start))
    {
        *pIn_buf_size = *pOut_buf_size = 0;
        return TINFL_STATUS_BAD_PARAM;
    }

    num_bits = r->m_num_bits;
    bit_buf = r->m_bit_buf;
    dist = r->m_dist;
    counter = r->m_counter;
    num_extra = r->m_num_extra;
    dist_from_out_buf_start = r->m_dist_from_out_buf_start;
    TINFL_CR_BEGIN

    bit_buf = num_bits = dist = counter = num_extra = r->m_zhdr0 = r->m_zhdr1 = 0;
    r->m_z_adler32 = r->m_check_adler32 = 1;
    if (decomp_flags & TINFL_FLAG_PARSE_ZLIB_HEADER)
    {
        TINFL_GET_BYTE(1, r->m_zhdr0);
        TINFL_GET_BYTE(2, r->m_zhdr1);
        counter = (((r->m_zhdr0 * 256 + r->m_zhdr1) % 31 != 0) || (r->m_zhdr1 & 32) || ((r->m_zhdr0 & 15) != 8));
        if (!(decomp_flags & TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF))
            counter |= (((1U << (8U + (r->m_zhdr0 >> 4))) > 32768U) || ((out_buf_size_mask + 1) < (size_t)(1U << (8U + (r->m_zhdr0 >> 4)))));
        if (counter)
        {
            TINFL_CR_RETURN_FOREVER(36, TINFL_STATUS_FAILED);
        }
    }

    do
    {
        TINFL_GET_BITS(3, r->m_final, 3);
        r->m_type = r->m_final >> 1;
        if (r->m_type == 0)
        {
            TINFL_SKIP_BITS(5, num_bits & 7);
            for (counter = 0; counter < 4; ++counter)
            {
                if (num_bits)
                    TINFL_GET_BITS(6, r->m_raw_header[counter], 8);
                else
                    TINFL_GET_BYTE(7, r->m_raw_header[counter]);
            }
            if ((counter = (r->m_raw_header[0] | (r->m_raw_header[1] << 8))) != (tmz_uint)(0xFFFF ^ (r->m_raw_header[2] | (r->m_raw_header[3] << 8))))
            {
                TINFL_CR_RETURN_FOREVER(39, TINFL_STATUS_FAILED);
            }
            while ((counter) && (num_bits))
            {
                TINFL_GET_BITS(51, dist, 8);
                while (pOut_buf_cur >= pOut_buf_end)
                {
                    TINFL_CR_RETURN(52, TINFL_STATUS_HAS_MORE_OUTPUT);
                }
                *pOut_buf_cur++ = (tmz_uint8)dist;
                counter--;
            }
            while (counter)
            {
                size_t n;
                while (pOut_buf_cur >= pOut_buf_end)
                {
                    TINFL_CR_RETURN(9, TINFL_STATUS_HAS_MORE_OUTPUT);
                }
                while (pIn_buf_cur >= pIn_buf_end)
                {
                    TINFL_CR_RETURN(38, (decomp_flags & TINFL_FLAG_HAS_MORE_INPUT) ? TINFL_STATUS_NEEDS_MORE_INPUT : TINFL_STATUS_FAILED_CANNOT_MAKE_PROGRESS);
                }
                n = tmz_MIN(tmz_MIN((size_t)(pOut_buf_end - pOut_buf_cur), (size_t)(pIn_buf_end - pIn_buf_cur)), counter);
                TINFL_MEMCPY(pOut_buf_cur, pIn_buf_cur, n);
                pIn_buf_cur += n;
                pOut_buf_cur += n;
                counter -= (tmz_uint)n;
            }
        }
        else if (r->m_type == 3)
        {
            TINFL_CR_RETURN_FOREVER(10, TINFL_STATUS_FAILED);
        }
        else
        {
            if (r->m_type == 1)
            {
                tmz_uint8 *p = r->m_tables[0].m_code_size;
                tmz_uint i;
                r->m_table_sizes[0] = 288;
                r->m_table_sizes[1] = 32;
                TINFL_MEMSET(r->m_tables[1].m_code_size, 5, 32);
                for (i = 0; i <= 143; ++i)
                    *p++ = 8;
                for (; i <= 255; ++i)
                    *p++ = 9;
                for (; i <= 279; ++i)
                    *p++ = 7;
                for (; i <= 287; ++i)
                    *p++ = 8;
            }
            else
            {
                for (counter = 0; counter < 3; counter++)
                {
                    TINFL_GET_BITS(11, r->m_table_sizes[counter], "\05\05\04"[counter]);
                    r->m_table_sizes[counter] += s_min_table_sizes[counter];
                }
                tmz_CLEAR_OBJ(r->m_tables[2].m_code_size);
                for (counter = 0; counter < r->m_table_sizes[2]; counter++)
                {
                    tmz_uint s;
                    TINFL_GET_BITS(14, s, 3);
                    r->m_tables[2].m_code_size[s_length_dezigzag[counter]] = (tmz_uint8)s;
                }
                r->m_table_sizes[2] = 19;
            }
            for (; (int)r->m_type >= 0; r->m_type--)
            {
                int tree_next, tree_cur;
                tinfl_huff_table *pTable;
                tmz_uint i, j, used_syms, total, sym_index, next_code[17], total_syms[16];
                pTable = &r->m_tables[r->m_type];
                tmz_CLEAR_OBJ(total_syms);
                tmz_CLEAR_OBJ(pTable->m_look_up);
                tmz_CLEAR_OBJ(pTable->m_tree);
                for (i = 0; i < r->m_table_sizes[r->m_type]; ++i)
                    total_syms[pTable->m_code_size[i]]++;
                used_syms = 0, total = 0;
                next_code[0] = next_code[1] = 0;
                for (i = 1; i <= 15; ++i)
                {
                    used_syms += total_syms[i];
                    next_code[i + 1] = (total = ((total + total_syms[i]) << 1));
                }
                if ((65536 != total) && (used_syms > 1))
                {
                    TINFL_CR_RETURN_FOREVER(35, TINFL_STATUS_FAILED);
                }
                for (tree_next = -1, sym_index = 0; sym_index < r->m_table_sizes[r->m_type]; ++sym_index)
                {
                    tmz_uint rev_code = 0, l, cur_code, code_size = pTable->m_code_size[sym_index];
                    if (!code_size)
                        continue;
                    cur_code = next_code[code_size]++;
                    for (l = code_size; l > 0; l--, cur_code >>= 1)
                        rev_code = (rev_code << 1) | (cur_code & 1);
                    if (code_size <= TINFL_FAST_LOOKUP_BITS)
                    {
                        tmz_int16 k = (tmz_int16)((code_size << 9) | sym_index);
                        while (rev_code < TINFL_FAST_LOOKUP_SIZE)
                        {
                            pTable->m_look_up[rev_code] = k;
                            rev_code += (1 << code_size);
                        }
                        continue;
                    }
                    if (0 == (tree_cur = pTable->m_look_up[rev_code & (TINFL_FAST_LOOKUP_SIZE - 1)]))
                    {
                        pTable->m_look_up[rev_code & (TINFL_FAST_LOOKUP_SIZE - 1)] = (tmz_int16)tree_next;
                        tree_cur = tree_next;
                        tree_next -= 2;
                    }
                    rev_code >>= (TINFL_FAST_LOOKUP_BITS - 1);
                    for (j = code_size; j > (TINFL_FAST_LOOKUP_BITS + 1); j--)
                    {
                        tree_cur -= ((rev_code >>= 1) & 1);
                        if (!pTable->m_tree[-tree_cur - 1])
                        {
                            pTable->m_tree[-tree_cur - 1] = (tmz_int16)tree_next;
                            tree_cur = tree_next;
                            tree_next -= 2;
                        }
                        else
                            tree_cur = pTable->m_tree[-tree_cur - 1];
                    }
                    tree_cur -= ((rev_code >>= 1) & 1);
                    pTable->m_tree[-tree_cur - 1] = (tmz_int16)sym_index;
                }
                if (r->m_type == 2)
                {
                    for (counter = 0; counter < (r->m_table_sizes[0] + r->m_table_sizes[1]);)
                    {
                        tmz_uint s;
                        TINFL_HUFF_DECODE(16, dist, &r->m_tables[2]);
                        if (dist < 16)
                        {
                            r->m_len_codes[counter++] = (tmz_uint8)dist;
                            continue;
                        }
                        if ((dist == 16) && (!counter))
                        {
                            TINFL_CR_RETURN_FOREVER(17, TINFL_STATUS_FAILED);
                        }
                        num_extra = "\02\03\07"[dist - 16];
                        TINFL_GET_BITS(18, s, num_extra);
                        s += "\03\03\013"[dist - 16];
                        TINFL_MEMSET(r->m_len_codes + counter, (dist == 16) ? r->m_len_codes[counter - 1] : 0, s);
                        counter += s;
                    }
                    if ((r->m_table_sizes[0] + r->m_table_sizes[1]) != counter)
                    {
                        TINFL_CR_RETURN_FOREVER(21, TINFL_STATUS_FAILED);
                    }
                    TINFL_MEMCPY(r->m_tables[0].m_code_size, r->m_len_codes, r->m_table_sizes[0]);
                    TINFL_MEMCPY(r->m_tables[1].m_code_size, r->m_len_codes + r->m_table_sizes[0], r->m_table_sizes[1]);
                }
            }
            for (;;)
            {
                tmz_uint8 *pSrc;
                for (;;)
                {
                    if (((pIn_buf_end - pIn_buf_cur) < 4) || ((pOut_buf_end - pOut_buf_cur) < 2))
                    {
                        TINFL_HUFF_DECODE(23, counter, &r->m_tables[0]);
                        if (counter >= 256)
                            break;
                        while (pOut_buf_cur >= pOut_buf_end)
                        {
                            TINFL_CR_RETURN(24, TINFL_STATUS_HAS_MORE_OUTPUT);
                        }
                        *pOut_buf_cur++ = (tmz_uint8)counter;
                    }
                    else
                    {
                        int sym2;
                        tmz_uint code_len;
#if TINFL_USE_64BIT_BITBUF
                        if (num_bits < 30)
                        {
                            bit_buf |= (((tinfl_bit_buf_t)tmz_READ_LE32(pIn_buf_cur)) << num_bits);
                            pIn_buf_cur += 4;
                            num_bits += 32;
                        }
#else
                        if (num_bits < 15)
                        {
                            bit_buf |= (((tinfl_bit_buf_t)tmz_READ_LE16(pIn_buf_cur)) << num_bits);
                            pIn_buf_cur += 2;
                            num_bits += 16;
                        }
#endif
                        if ((sym2 = r->m_tables[0].m_look_up[bit_buf & (TINFL_FAST_LOOKUP_SIZE - 1)]) >= 0)
                            code_len = sym2 >> 9;
                        else
                        {
                            code_len = TINFL_FAST_LOOKUP_BITS;
                            do
                            {
                                sym2 = r->m_tables[0].m_tree[~sym2 + ((bit_buf >> code_len++) & 1)];
                            } while (sym2 < 0);
                        }
                        counter = sym2;
                        bit_buf >>= code_len;
                        num_bits -= code_len;
                        if (counter & 256)
                            break;

#if !TINFL_USE_64BIT_BITBUF
                        if (num_bits < 15)
                        {
                            bit_buf |= (((tinfl_bit_buf_t)tmz_READ_LE16(pIn_buf_cur)) << num_bits);
                            pIn_buf_cur += 2;
                            num_bits += 16;
                        }
#endif
                        if ((sym2 = r->m_tables[0].m_look_up[bit_buf & (TINFL_FAST_LOOKUP_SIZE - 1)]) >= 0)
                            code_len = sym2 >> 9;
                        else
                        {
                            code_len = TINFL_FAST_LOOKUP_BITS;
                            do
                            {
                                sym2 = r->m_tables[0].m_tree[~sym2 + ((bit_buf >> code_len++) & 1)];
                            } while (sym2 < 0);
                        }
                        bit_buf >>= code_len;
                        num_bits -= code_len;

                        pOut_buf_cur[0] = (tmz_uint8)counter;
                        if (sym2 & 256)
                        {
                            pOut_buf_cur++;
                            counter = sym2;
                            break;
                        }
                        pOut_buf_cur[1] = (tmz_uint8)sym2;
                        pOut_buf_cur += 2;
                    }
                }
                if ((counter &= 511) == 256)
                    break;

                num_extra = s_length_extra[counter - 257];
                counter = s_length_base[counter - 257];
                if (num_extra)
                {
                    tmz_uint extra_bits;
                    TINFL_GET_BITS(25, extra_bits, num_extra);
                    counter += extra_bits;
                }

                TINFL_HUFF_DECODE(26, dist, &r->m_tables[1]);
                num_extra = s_dist_extra[dist];
                dist = s_dist_base[dist];
                if (num_extra)
                {
                    tmz_uint extra_bits;
                    TINFL_GET_BITS(27, extra_bits, num_extra);
                    dist += extra_bits;
                }

                dist_from_out_buf_start = pOut_buf_cur - pOut_buf_start;
                if ((dist > dist_from_out_buf_start) && (decomp_flags & TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF))
                {
                    TINFL_CR_RETURN_FOREVER(37, TINFL_STATUS_FAILED);
                }

                pSrc = pOut_buf_start + ((dist_from_out_buf_start - dist) & out_buf_size_mask);

                if ((tmz_MAX(pOut_buf_cur, pSrc) + counter) > pOut_buf_end)
                {
                    while (counter--)
                    {
                        while (pOut_buf_cur >= pOut_buf_end)
                        {
                            TINFL_CR_RETURN(53, TINFL_STATUS_HAS_MORE_OUTPUT);
                        }
                        *pOut_buf_cur++ = pOut_buf_start[(dist_from_out_buf_start++ - dist) & out_buf_size_mask];
                    }
                    continue;
                }
#if tminiz_USE_UNALIGNED_LOADS_AND_STORES
                else if ((counter >= 9) && (counter <= dist))
                {
                    const tmz_uint8 *pSrc_end = pSrc + (counter & ~7);
                    do
                    {
                        ((tmz_uint32 *)pOut_buf_cur)[0] = ((const tmz_uint32 *)pSrc)[0];
                        ((tmz_uint32 *)pOut_buf_cur)[1] = ((const tmz_uint32 *)pSrc)[1];
                        pOut_buf_cur += 8;
                    } while ((pSrc += 8) < pSrc_end);
                    if ((counter &= 7) < 3)
                    {
                        if (counter)
                        {
                            pOut_buf_cur[0] = pSrc[0];
                            if (counter > 1)
                                pOut_buf_cur[1] = pSrc[1];
                            pOut_buf_cur += counter;
                        }
                        continue;
                    }
                }
#endif
                do
                {
                    pOut_buf_cur[0] = pSrc[0];
                    pOut_buf_cur[1] = pSrc[1];
                    pOut_buf_cur[2] = pSrc[2];
                    pOut_buf_cur += 3;
                    pSrc += 3;
                } while ((int)(counter -= 3) > 2);
                if ((int)counter > 0)
                {
                    pOut_buf_cur[0] = pSrc[0];
                    if ((int)counter > 1)
                        pOut_buf_cur[1] = pSrc[1];
                    pOut_buf_cur += counter;
                }
            }
        }
    } while (!(r->m_final & 1));

    /* Ensure byte alignment and put back any bytes from the bitbuf if we've looked ahead too far on gzip, or other Deflate streams followed by arbitrary data. */
    /* I'm being super conservative here. A number of simplifications can be made to the byte alignment part, and the Adler32 check shouldn't ever need to worry about reading from the bitbuf now. */
    TINFL_SKIP_BITS(32, num_bits & 7);
    while ((pIn_buf_cur > pIn_buf_next) && (num_bits >= 8))
    {
        --pIn_buf_cur;
        num_bits -= 8;
    }
    bit_buf &= (tinfl_bit_buf_t)(( ((tmz_uint64)1) << num_bits) - (tmz_uint64)1);
    tmz_ASSERT(!num_bits); /* if this assert fires then we've read beyond the end of non-deflate/zlib streams with following data (such as gzip streams). */

    if (decomp_flags & TINFL_FLAG_PARSE_ZLIB_HEADER)
    {
        for (counter = 0; counter < 4; ++counter)
        {
            tmz_uint s;
            if (num_bits)
                TINFL_GET_BITS(41, s, 8);
            else
                TINFL_GET_BYTE(42, s);
            r->m_z_adler32 = (r->m_z_adler32 << 8) | s;
        }
    }
    TINFL_CR_RETURN_FOREVER(34, TINFL_STATUS_DONE);

    TINFL_CR_FINISH

common_exit:
    /* As long as we aren't telling the caller that we NEED more input to make forward progress: */
    /* Put back any bytes from the bitbuf in case we've looked ahead too far on gzip, or other Deflate streams followed by arbitrary data. */
    /* We need to be very careful here to NOT push back any bytes we definitely know we need to make forward progress, though, or we'll lock the caller up into an inf loop. */
    if ((status != TINFL_STATUS_NEEDS_MORE_INPUT) && (status != TINFL_STATUS_FAILED_CANNOT_MAKE_PROGRESS))
    {
        while ((pIn_buf_cur > pIn_buf_next) && (num_bits >= 8))
        {
            --pIn_buf_cur;
            num_bits -= 8;
        }
    }
    r->m_num_bits = num_bits;
    r->m_bit_buf = bit_buf & (tinfl_bit_buf_t)(( ((tmz_uint64)1) << num_bits) - (tmz_uint64)1);
    r->m_dist = dist;
    r->m_counter = counter;
    r->m_num_extra = num_extra;
    r->m_dist_from_out_buf_start = dist_from_out_buf_start;
    *pIn_buf_size = pIn_buf_cur - pIn_buf_next;
    *pOut_buf_size = pOut_buf_cur - pOut_buf_next;
    if ((decomp_flags & (TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_COMPUTE_ADLER32)) && (status >= 0))
    {
        const tmz_uint8 *ptr = pOut_buf_next;
        size_t buf_len = *pOut_buf_size;
        tmz_uint32 i, s1 = r->m_check_adler32 & 0xffff, s2 = r->m_check_adler32 >> 16;
        size_t block_len = buf_len % 5552;
        while (buf_len)
        {
            for (i = 0; i + 7 < block_len; i += 8, ptr += 8)
            {
                s1 += ptr[0], s2 += s1;
                s1 += ptr[1], s2 += s1;
                s1 += ptr[2], s2 += s1;
                s1 += ptr[3], s2 += s1;
                s1 += ptr[4], s2 += s1;
                s1 += ptr[5], s2 += s1;
                s1 += ptr[6], s2 += s1;
                s1 += ptr[7], s2 += s1;
            }
            for (; i < block_len; ++i)
                s1 += *ptr++, s2 += s1;
            s1 %= 65521U, s2 %= 65521U;
            buf_len -= block_len;
            block_len = 5552;
        }
        r->m_check_adler32 = (s2 << 16) + s1;
        if ((status == TINFL_STATUS_DONE) && (decomp_flags & TINFL_FLAG_PARSE_ZLIB_HEADER) && (r->m_check_adler32 != r->m_z_adler32))
            status = TINFL_STATUS_ADLER32_MISMATCH;
    }
    return status;
}

/* Higher level helper functions. */
void *tinfl_decompress_mem_to_heap(const void *pSrc_buf, size_t src_buf_len, size_t *pOut_len, int flags)
{
    tinfl_decompressor decomp;
    void *pBuf = NULL, *pNew_buf;
    size_t src_buf_ofs = 0, out_buf_capacity = 0;
    *pOut_len = 0;
    tinfl_init(&decomp);
    for (;;)
    {
        size_t src_buf_size = src_buf_len - src_buf_ofs, dst_buf_size = out_buf_capacity - *pOut_len, new_out_buf_capacity;
        tinfl_status status = tinfl_decompress(&decomp, (const tmz_uint8 *)pSrc_buf + src_buf_ofs, &src_buf_size, (tmz_uint8 *)pBuf, pBuf ? (tmz_uint8 *)pBuf + *pOut_len : NULL, &dst_buf_size,
                                               (flags & ~TINFL_FLAG_HAS_MORE_INPUT) | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
        if ((status < 0) || (status == TINFL_STATUS_NEEDS_MORE_INPUT))
        {
            tmz_FREE(pBuf);
            *pOut_len = 0;
            return NULL;
        }
        src_buf_ofs += src_buf_size;
        *pOut_len += dst_buf_size;
        if (status == TINFL_STATUS_DONE)
            break;
        new_out_buf_capacity = out_buf_capacity * 2;
        if (new_out_buf_capacity < 128)
            new_out_buf_capacity = 128;
        pNew_buf = tmz_REALLOC(pBuf, new_out_buf_capacity);
        if (!pNew_buf)
        {
            tmz_FREE(pBuf);
            *pOut_len = 0;
            return NULL;
        }
        pBuf = pNew_buf;
        out_buf_capacity = new_out_buf_capacity;
    }
    return pBuf;
}

size_t tinfl_decompress_mem_to_mem(void *pOut_buf, size_t out_buf_len, const void *pSrc_buf, size_t src_buf_len, int flags)
{
    tinfl_decompressor decomp;
    tinfl_status status;
    tinfl_init(&decomp);
    status = tinfl_decompress(&decomp, (const tmz_uint8 *)pSrc_buf, &src_buf_len, (tmz_uint8 *)pOut_buf, (tmz_uint8 *)pOut_buf, &out_buf_len, (flags & ~TINFL_FLAG_HAS_MORE_INPUT) | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    return (status != TINFL_STATUS_DONE) ? TINFL_DECOMPRESS_MEM_TO_MEM_FAILED : out_buf_len;
}

int tinfl_decompress_mem_to_callback(const void *pIn_buf, size_t *pIn_buf_size, tinfl_put_buf_func_ptr pPut_buf_func, void *pPut_buf_user, int flags)
{
    int result = 0;
    tinfl_decompressor decomp;
    tmz_uint8 *pDict = (tmz_uint8 *)tmz_MALLOC(TINFL_LZ_DICT_SIZE);
    size_t in_buf_ofs = 0, dict_ofs = 0;
    if (!pDict)
        return TINFL_STATUS_FAILED;
    tinfl_init(&decomp);
    for (;;)
    {
        size_t in_buf_size = *pIn_buf_size - in_buf_ofs, dst_buf_size = TINFL_LZ_DICT_SIZE - dict_ofs;
        tinfl_status status = tinfl_decompress(&decomp, (const tmz_uint8 *)pIn_buf + in_buf_ofs, &in_buf_size, pDict, pDict + dict_ofs, &dst_buf_size,
                                               (flags & ~(TINFL_FLAG_HAS_MORE_INPUT | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF)));
        in_buf_ofs += in_buf_size;
        if ((dst_buf_size) && (!(*pPut_buf_func)(pDict + dict_ofs, (int)dst_buf_size, pPut_buf_user)))
            break;
        if (status != TINFL_STATUS_HAS_MORE_OUTPUT)
        {
            result = (status == TINFL_STATUS_DONE);
            break;
        }
        dict_ofs = (dict_ofs + dst_buf_size) & (TINFL_LZ_DICT_SIZE - 1);
    }
    tmz_FREE(pDict);
    *pIn_buf_size = in_buf_ofs;
    return result;
}

tinfl_decompressor *tinfl_decompressor_alloc()
{
    tinfl_decompressor *pDecomp = (tinfl_decompressor *)tmz_MALLOC(sizeof(tinfl_decompressor));
    if (pDecomp)
        tinfl_init(pDecomp);
    return pDecomp;
}

void tinfl_decompressor_free(tinfl_decompressor *pDecomp)
{
    tmz_FREE(pDecomp);
}

#ifdef __cplusplus
}
#endif
/**************************************************************************
 *
 * Copyright 2013-2014 RAD Game Tools and Valve Software
 * Copyright 2010-2014 Rich Geldreich and Tenacious Software LLC
 * Copyright 2016 Martin Raiber
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 **************************************************************************/


#ifndef tminiz_NO_ARCHIVE_APIS

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------- .ZIP archive reading */

#ifdef tminiz_NO_STDIO
#define tmz_FILE void *
#else
#include <sys/stat.h>

#if defined(_MSC_VER) || defined(__MINGW64__)
static FILE *tmz_fopen(const char *pFilename, const char *pMode)
{
    FILE *pFile = NULL;
    fopen_s(&pFile, pFilename, pMode);
    return pFile;
}
static FILE *tmz_freopen(const char *pPath, const char *pMode, FILE *pStream)
{
    FILE *pFile = NULL;
    if (freopen_s(&pFile, pPath, pMode, pStream))
        return NULL;
    return pFile;
}
#ifndef tminiz_NO_TIME
#include <sys/utime.h>
#endif
#define tmz_FOPEN tmz_fopen
#define tmz_FCLOSE fclose
#define tmz_FREAD fread
#define tmz_FWRITE fwrite
#define tmz_FTELL64 _ftelli64
#define tmz_FSEEK64 _fseeki64
#define tmz_FILE_STAT_STRUCT _stat
#define tmz_FILE_STAT _stat
#define tmz_FFLUSH fflush
#define tmz_FREOPEN tmz_freopen
#define tmz_DELETE_FILE remove
#elif defined(__MINGW32__)
#ifndef tminiz_NO_TIME
#include <sys/utime.h>
#endif
#define tmz_FOPEN(f, m) fopen(f, m)
#define tmz_FCLOSE fclose
#define tmz_FREAD fread
#define tmz_FWRITE fwrite
#define tmz_FTELL64 ftello64
#define tmz_FSEEK64 fseeko64
#define tmz_FILE_STAT_STRUCT _stat
#define tmz_FILE_STAT _stat
#define tmz_FFLUSH fflush
#define tmz_FREOPEN(f, m, s) freopen(f, m, s)
#define tmz_DELETE_FILE remove
#elif defined(__TINYC__)
#ifndef tminiz_NO_TIME
#include <sys/utime.h>
#endif
#define tmz_FOPEN(f, m) fopen(f, m)
#define tmz_FCLOSE fclose
#define tmz_FREAD fread
#define tmz_FWRITE fwrite
#define tmz_FTELL64 ftell
#define tmz_FSEEK64 fseek
#define tmz_FILE_STAT_STRUCT stat
#define tmz_FILE_STAT stat
#define tmz_FFLUSH fflush
#define tmz_FREOPEN(f, m, s) freopen(f, m, s)
#define tmz_DELETE_FILE remove
#elif defined(__GNUC__) && _LARGEFILE64_SOURCE
#ifndef tminiz_NO_TIME
#include <utime.h>
#endif
#define tmz_FOPEN(f, m) fopen64(f, m)
#define tmz_FCLOSE fclose
#define tmz_FREAD fread
#define tmz_FWRITE fwrite
#define tmz_FTELL64 ftello64
#define tmz_FSEEK64 fseeko64
#define tmz_FILE_STAT_STRUCT stat64
#define tmz_FILE_STAT stat64
#define tmz_FFLUSH fflush
#define tmz_FREOPEN(p, m, s) freopen64(p, m, s)
#define tmz_DELETE_FILE remove
#elif defined(__APPLE__) && _LARGEFILE64_SOURCE
#ifndef tminiz_NO_TIME
#include <utime.h>
#endif
#define tmz_FOPEN(f, m) fopen(f, m)
#define tmz_FCLOSE fclose
#define tmz_FREAD fread
#define tmz_FWRITE fwrite
#define tmz_FTELL64 ftello
#define tmz_FSEEK64 fseeko
#define tmz_FILE_STAT_STRUCT stat
#define tmz_FILE_STAT stat
#define tmz_FFLUSH fflush
#define tmz_FREOPEN(p, m, s) freopen(p, m, s)
#define tmz_DELETE_FILE remove

#else
#pragma message("Using fopen, ftello, fseeko, stat() etc. path for file I/O - this path may not support large files.")
#ifndef tminiz_NO_TIME
#include <utime.h>
#endif
#define tmz_FOPEN(f, m) fopen(f, m)
#define tmz_FCLOSE fclose
#define tmz_FREAD fread
#define tmz_FWRITE fwrite
#ifdef __STRICT_ANSI__
#define tmz_FTELL64 ftell
#define tmz_FSEEK64 fseek
#else
#define tmz_FTELL64 ftello
#define tmz_FSEEK64 fseeko
#endif
#define tmz_FILE_STAT_STRUCT stat
#define tmz_FILE_STAT stat
#define tmz_FFLUSH fflush
#define tmz_FREOPEN(f, m, s) freopen(f, m, s)
#define tmz_DELETE_FILE remove
#endif /* #ifdef _MSC_VER */
#endif /* #ifdef tminiz_NO_STDIO */

#define tmz_TOLOWER(c) ((((c) >= 'A') && ((c) <= 'Z')) ? ((c) - 'A' + 'a') : (c))

/* Various ZIP archive enums. To completely avoid cross platform compiler alignment and platform endian issues, miniz.c doesn't use structs for any of this stuff. */
enum
{
    /* ZIP archive identifiers and record sizes */
    tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIG = 0x06054b50,
    tmz_ZIP_CENTRAL_DIR_HEADER_SIG = 0x02014b50,
    tmz_ZIP_LOCAL_DIR_HEADER_SIG = 0x04034b50,
    tmz_ZIP_LOCAL_DIR_HEADER_SIZE = 30,
    tmz_ZIP_CENTRAL_DIR_HEADER_SIZE = 46,
    tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE = 22,

    /* ZIP64 archive identifier and record sizes */
    tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIG = 0x06064b50,
    tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIG = 0x07064b50,
    tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE = 56,
    tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE = 20,
    tmz_ZIP64_EXTENDED_INFORMATION_FIELD_HEADER_ID = 0x0001,
    tmz_ZIP_DATA_DESCRIPTOR_ID = 0x08074b50,
    tmz_ZIP_DATA_DESCRIPTER_SIZE64 = 24,
    tmz_ZIP_DATA_DESCRIPTER_SIZE32 = 16,

    /* Central directory header record offsets */
    tmz_ZIP_CDH_SIG_OFS = 0,
    tmz_ZIP_CDH_VERSION_MADE_BY_OFS = 4,
    tmz_ZIP_CDH_VERSION_NEEDED_OFS = 6,
    tmz_ZIP_CDH_BIT_FLAG_OFS = 8,
    tmz_ZIP_CDH_METHOD_OFS = 10,
    tmz_ZIP_CDH_FILE_TIME_OFS = 12,
    tmz_ZIP_CDH_FILE_DATE_OFS = 14,
    tmz_ZIP_CDH_CRC32_OFS = 16,
    tmz_ZIP_CDH_COMPRESSED_SIZE_OFS = 20,
    tmz_ZIP_CDH_DECOMPRESSED_SIZE_OFS = 24,
    tmz_ZIP_CDH_FILENAME_LEN_OFS = 28,
    tmz_ZIP_CDH_EXTRA_LEN_OFS = 30,
    tmz_ZIP_CDH_COMMENT_LEN_OFS = 32,
    tmz_ZIP_CDH_DISK_START_OFS = 34,
    tmz_ZIP_CDH_INTERNAL_ATTR_OFS = 36,
    tmz_ZIP_CDH_EXTERNAL_ATTR_OFS = 38,
    tmz_ZIP_CDH_LOCAL_HEADER_OFS = 42,

    /* Local directory header offsets */
    tmz_ZIP_LDH_SIG_OFS = 0,
    tmz_ZIP_LDH_VERSION_NEEDED_OFS = 4,
    tmz_ZIP_LDH_BIT_FLAG_OFS = 6,
    tmz_ZIP_LDH_METHOD_OFS = 8,
    tmz_ZIP_LDH_FILE_TIME_OFS = 10,
    tmz_ZIP_LDH_FILE_DATE_OFS = 12,
    tmz_ZIP_LDH_CRC32_OFS = 14,
    tmz_ZIP_LDH_COMPRESSED_SIZE_OFS = 18,
    tmz_ZIP_LDH_DECOMPRESSED_SIZE_OFS = 22,
    tmz_ZIP_LDH_FILENAME_LEN_OFS = 26,
    tmz_ZIP_LDH_EXTRA_LEN_OFS = 28,
    tmz_ZIP_LDH_BIT_FLAG_HAS_LOCATOR = 1 << 3,

    /* End of central directory offsets */
    tmz_ZIP_ECDH_SIG_OFS = 0,
    tmz_ZIP_ECDH_NUM_THIS_DISK_OFS = 4,
    tmz_ZIP_ECDH_NUM_DISK_CDIR_OFS = 6,
    tmz_ZIP_ECDH_CDIR_NUM_ENTRIES_ON_DISK_OFS = 8,
    tmz_ZIP_ECDH_CDIR_TOTAL_ENTRIES_OFS = 10,
    tmz_ZIP_ECDH_CDIR_SIZE_OFS = 12,
    tmz_ZIP_ECDH_CDIR_OFS_OFS = 16,
    tmz_ZIP_ECDH_COMMENT_SIZE_OFS = 20,

    /* ZIP64 End of central directory locator offsets */
    tmz_ZIP64_ECDL_SIG_OFS = 0,                    /* 4 bytes */
    tmz_ZIP64_ECDL_NUM_DISK_CDIR_OFS = 4,          /* 4 bytes */
    tmz_ZIP64_ECDL_REL_OFS_TO_ZIP64_ECDR_OFS = 8,  /* 8 bytes */
    tmz_ZIP64_ECDL_TOTAL_NUMBER_OF_DISKS_OFS = 16, /* 4 bytes */

    /* ZIP64 End of central directory header offsets */
    tmz_ZIP64_ECDH_SIG_OFS = 0,                       /* 4 bytes */
    tmz_ZIP64_ECDH_SIZE_OF_RECORD_OFS = 4,            /* 8 bytes */
    tmz_ZIP64_ECDH_VERSION_MADE_BY_OFS = 12,          /* 2 bytes */
    tmz_ZIP64_ECDH_VERSION_NEEDED_OFS = 14,           /* 2 bytes */
    tmz_ZIP64_ECDH_NUM_THIS_DISK_OFS = 16,            /* 4 bytes */
    tmz_ZIP64_ECDH_NUM_DISK_CDIR_OFS = 20,            /* 4 bytes */
    tmz_ZIP64_ECDH_CDIR_NUM_ENTRIES_ON_DISK_OFS = 24, /* 8 bytes */
    tmz_ZIP64_ECDH_CDIR_TOTAL_ENTRIES_OFS = 32,       /* 8 bytes */
    tmz_ZIP64_ECDH_CDIR_SIZE_OFS = 40,                /* 8 bytes */
    tmz_ZIP64_ECDH_CDIR_OFS_OFS = 48,                 /* 8 bytes */
    tmz_ZIP_VERSION_MADE_BY_DOS_FILESYSTEM_ID = 0,
    tmz_ZIP_DOS_DIR_ATTRIBUTE_BITFLAG = 0x10,
    tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_IS_ENCRYPTED = 1,
    tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_COMPRESSED_PATCH_FLAG = 32,
    tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_USES_STRONG_ENCRYPTION = 64,
    tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_LOCAL_DIR_IS_MASKED = 8192,
    tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_UTF8 = 1 << 11
};

typedef struct
{
    void *m_p;
    size_t m_size, m_capacity;
    tmz_uint m_element_size;
} tmz_zip_array;

struct tmz_zip_internal_state_tag
{
    tmz_zip_array m_central_dir;
    tmz_zip_array m_central_dir_offsets;
    tmz_zip_array m_sorted_central_dir_offsets;

    /* The flags passed in when the archive is initially opened. */
    uint32_t m_init_flags;

    /* tmz_TRUE if the archive has a zip64 end of central directory headers, etc. */
    tmz_bool m_zip64;

    /* tmz_TRUE if we found zip64 extended info in the central directory (m_zip64 will also be slammed to true too, even if we didn't find a zip64 end of central dir header, etc.) */
    tmz_bool m_zip64_has_extended_info_fields;

    /* These fields are used by the file, FILE, memory, and memory/heap read/write helpers. */
    tmz_FILE *m_pFile;
    tmz_uint64 m_file_archive_start_ofs;

    void *m_pMem;
    size_t m_mem_size;
    size_t m_mem_capacity;
};

#define tmz_ZIP_ARRAY_SET_ELEMENT_SIZE(array_ptr, element_size) (array_ptr)->m_element_size = element_size

#if defined(DEBUG) || defined(_DEBUG) || defined(NDEBUG)
static tmz_FORCEINLINE tmz_uint tmz_zip_array_range_check(const tmz_zip_array *pArray, tmz_uint index)
{
    tmz_ASSERT(index < pArray->m_size);
    return index;
}
#define tmz_ZIP_ARRAY_ELEMENT(array_ptr, element_type, index) ((element_type *)((array_ptr)->m_p))[tmz_zip_array_range_check(array_ptr, index)]
#else
#define tmz_ZIP_ARRAY_ELEMENT(array_ptr, element_type, index) ((element_type *)((array_ptr)->m_p))[index]
#endif

static tmz_FORCEINLINE void tmz_zip_array_init(tmz_zip_array *pArray, tmz_uint32 element_size)
{
    memset(pArray, 0, sizeof(tmz_zip_array));
    pArray->m_element_size = element_size;
}

static tmz_FORCEINLINE void tmz_zip_array_clear(tmz_zip_archive *pZip, tmz_zip_array *pArray)
{
    pZip->m_pFree(pZip->m_pAlloc_opaque, pArray->m_p);
    memset(pArray, 0, sizeof(tmz_zip_array));
}

static tmz_bool tmz_zip_array_ensure_capacity(tmz_zip_archive *pZip, tmz_zip_array *pArray, size_t min_new_capacity, tmz_uint growing)
{
    void *pNew_p;
    size_t new_capacity = min_new_capacity;
    tmz_ASSERT(pArray->m_element_size);
    if (pArray->m_capacity >= min_new_capacity)
        return tmz_TRUE;
    if (growing)
    {
        new_capacity = tmz_MAX(1, pArray->m_capacity);
        while (new_capacity < min_new_capacity)
            new_capacity *= 2;
    }
    if (NULL == (pNew_p = pZip->m_pRealloc(pZip->m_pAlloc_opaque, pArray->m_p, pArray->m_element_size, new_capacity)))
        return tmz_FALSE;
    pArray->m_p = pNew_p;
    pArray->m_capacity = new_capacity;
    return tmz_TRUE;
}

static tmz_FORCEINLINE tmz_bool tmz_zip_array_reserve(tmz_zip_archive *pZip, tmz_zip_array *pArray, size_t new_capacity, tmz_uint growing)
{
    if (new_capacity > pArray->m_capacity)
    {
        if (!tmz_zip_array_ensure_capacity(pZip, pArray, new_capacity, growing))
            return tmz_FALSE;
    }
    return tmz_TRUE;
}

static tmz_FORCEINLINE tmz_bool tmz_zip_array_resize(tmz_zip_archive *pZip, tmz_zip_array *pArray, size_t new_size, tmz_uint growing)
{
    if (new_size > pArray->m_capacity)
    {
        if (!tmz_zip_array_ensure_capacity(pZip, pArray, new_size, growing))
            return tmz_FALSE;
    }
    pArray->m_size = new_size;
    return tmz_TRUE;
}

static tmz_FORCEINLINE tmz_bool tmz_zip_array_ensure_room(tmz_zip_archive *pZip, tmz_zip_array *pArray, size_t n)
{
    return tmz_zip_array_reserve(pZip, pArray, pArray->m_size + n, tmz_TRUE);
}

static tmz_FORCEINLINE tmz_bool tmz_zip_array_push_back(tmz_zip_archive *pZip, tmz_zip_array *pArray, const void *pElements, size_t n)
{
    size_t orig_size = pArray->m_size;
    if (!tmz_zip_array_resize(pZip, pArray, orig_size + n, tmz_TRUE))
        return tmz_FALSE;
    memcpy((tmz_uint8 *)pArray->m_p + orig_size * pArray->m_element_size, pElements, n * pArray->m_element_size);
    return tmz_TRUE;
}

#ifndef tminiz_NO_TIME
static tmz_TIME_T tmz_zip_dos_to_time_t(int dos_time, int dos_date)
{
    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    tm.tm_isdst = -1;
    tm.tm_year = ((dos_date >> 9) & 127) + 1980 - 1900;
    tm.tm_mon = ((dos_date >> 5) & 15) - 1;
    tm.tm_mday = dos_date & 31;
    tm.tm_hour = (dos_time >> 11) & 31;
    tm.tm_min = (dos_time >> 5) & 63;
    tm.tm_sec = (dos_time << 1) & 62;
    return mktime(&tm);
}

#ifndef tminiz_NO_ARCHIVE_WRITING_APIS
static void tmz_zip_time_t_to_dos_time(tmz_TIME_T time, tmz_uint16 *pDOS_time, tmz_uint16 *pDOS_date)
{
#ifdef _MSC_VER
    struct tm tm_struct;
    struct tm *tm = &tm_struct;
    errno_t err = localtime_s(tm, &time);
    if (err)
    {
        *pDOS_date = 0;
        *pDOS_time = 0;
        return;
    }
#else
    struct tm *tm = localtime(&time);
#endif /* #ifdef _MSC_VER */

    *pDOS_time = (tmz_uint16)(((tm->tm_hour) << 11) + ((tm->tm_min) << 5) + ((tm->tm_sec) >> 1));
    *pDOS_date = (tmz_uint16)(((tm->tm_year + 1900 - 1980) << 9) + ((tm->tm_mon + 1) << 5) + tm->tm_mday);
}
#endif /* tminiz_NO_ARCHIVE_WRITING_APIS */

#ifndef tminiz_NO_STDIO
#ifndef tminiz_NO_ARCHIVE_WRITING_APIS
static tmz_bool tmz_zip_get_file_modified_time(const char *pFilename, tmz_TIME_T *pTime)
{
    struct tmz_FILE_STAT_STRUCT file_stat;

    /* On Linux with x86 glibc, this call will fail on large files (I think >= 0x80000000 bytes) unless you compiled with _LARGEFILE64_SOURCE. Argh. */
    if (tmz_FILE_STAT(pFilename, &file_stat) != 0)
        return tmz_FALSE;

    *pTime = file_stat.st_mtime;

    return tmz_TRUE;
}
#endif /* #ifndef tminiz_NO_ARCHIVE_WRITING_APIS*/

static tmz_bool tmz_zip_set_file_times(const char *pFilename, tmz_TIME_T access_time, tmz_TIME_T modified_time)
{
    struct utimbuf t;

    memset(&t, 0, sizeof(t));
    t.actime = access_time;
    t.modtime = modified_time;

    return !utime(pFilename, &t);
}
#endif /* #ifndef tminiz_NO_STDIO */
#endif /* #ifndef tminiz_NO_TIME */

static tmz_FORCEINLINE tmz_bool tmz_zip_set_error(tmz_zip_archive *pZip, tmz_zip_error err_num)
{
    if (pZip)
        pZip->m_last_error = err_num;
    return tmz_FALSE;
}

static tmz_bool tmz_zip_reader_init_internal(tmz_zip_archive *pZip, tmz_uint flags)
{
    (void)flags;
    if ((!pZip) || (pZip->m_pState) || (pZip->m_zip_mode != tmz_ZIP_MODE_INVALID))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (!pZip->m_pAlloc)
        pZip->m_pAlloc = tminiz_def_alloc_func;
    if (!pZip->m_pFree)
        pZip->m_pFree = tminiz_def_free_func;
    if (!pZip->m_pRealloc)
        pZip->m_pRealloc = tminiz_def_realloc_func;

    pZip->m_archive_size = 0;
    pZip->m_central_directory_file_ofs = 0;
    pZip->m_total_files = 0;
    pZip->m_last_error = tmz_ZIP_NO_ERROR;

    if (NULL == (pZip->m_pState = (tmz_zip_internal_state *)pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, sizeof(tmz_zip_internal_state))))
        return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

    memset(pZip->m_pState, 0, sizeof(tmz_zip_internal_state));
    tmz_ZIP_ARRAY_SET_ELEMENT_SIZE(&pZip->m_pState->m_central_dir, sizeof(tmz_uint8));
    tmz_ZIP_ARRAY_SET_ELEMENT_SIZE(&pZip->m_pState->m_central_dir_offsets, sizeof(tmz_uint32));
    tmz_ZIP_ARRAY_SET_ELEMENT_SIZE(&pZip->m_pState->m_sorted_central_dir_offsets, sizeof(tmz_uint32));
    pZip->m_pState->m_init_flags = flags;
    pZip->m_pState->m_zip64 = tmz_FALSE;
    pZip->m_pState->m_zip64_has_extended_info_fields = tmz_FALSE;

    pZip->m_zip_mode = tmz_ZIP_MODE_READING;

    return tmz_TRUE;
}

static tmz_FORCEINLINE tmz_bool tmz_zip_reader_filename_less(const tmz_zip_array *pCentral_dir_array, const tmz_zip_array *pCentral_dir_offsets, tmz_uint l_index, tmz_uint r_index)
{
    const tmz_uint8 *pL = &tmz_ZIP_ARRAY_ELEMENT(pCentral_dir_array, tmz_uint8, tmz_ZIP_ARRAY_ELEMENT(pCentral_dir_offsets, tmz_uint32, l_index)), *pE;
    const tmz_uint8 *pR = &tmz_ZIP_ARRAY_ELEMENT(pCentral_dir_array, tmz_uint8, tmz_ZIP_ARRAY_ELEMENT(pCentral_dir_offsets, tmz_uint32, r_index));
    tmz_uint l_len = tmz_READ_LE16(pL + tmz_ZIP_CDH_FILENAME_LEN_OFS), r_len = tmz_READ_LE16(pR + tmz_ZIP_CDH_FILENAME_LEN_OFS);
    tmz_uint8 l = 0, r = 0;
    pL += tmz_ZIP_CENTRAL_DIR_HEADER_SIZE;
    pR += tmz_ZIP_CENTRAL_DIR_HEADER_SIZE;
    pE = pL + tmz_MIN(l_len, r_len);
    while (pL < pE)
    {
        if ((l = tmz_TOLOWER(*pL)) != (r = tmz_TOLOWER(*pR)))
            break;
        pL++;
        pR++;
    }
    return (pL == pE) ? (l_len < r_len) : (l < r);
}

#define tmz_SWAP_UINT32(a, b) \
    do                       \
    {                        \
        tmz_uint32 t = a;     \
        a = b;               \
        b = t;               \
    }                        \
    tmz_MACRO_END

/* Heap sort of lowercased filenames, used to help accelerate plain central directory searches by tmz_zip_reader_locate_file(). (Could also use qsort(), but it could allocate memory.) */
static void tmz_zip_reader_sort_central_dir_offsets_by_filename(tmz_zip_archive *pZip)
{
    tmz_zip_internal_state *pState = pZip->m_pState;
    const tmz_zip_array *pCentral_dir_offsets = &pState->m_central_dir_offsets;
    const tmz_zip_array *pCentral_dir = &pState->m_central_dir;
    tmz_uint32 *pIndices;
    tmz_uint32 start, end;
    const tmz_uint32 size = pZip->m_total_files;

    if (size <= 1U)
        return;

    pIndices = &tmz_ZIP_ARRAY_ELEMENT(&pState->m_sorted_central_dir_offsets, tmz_uint32, 0);

    start = (size - 2U) >> 1U;
    for (;;)
    {
        tmz_uint64 child, root = start;
        for (;;)
        {
            if ((child = (root << 1U) + 1U) >= size)
                break;
            child += (((child + 1U) < size) && (tmz_zip_reader_filename_less(pCentral_dir, pCentral_dir_offsets, pIndices[child], pIndices[child + 1U])));
            if (!tmz_zip_reader_filename_less(pCentral_dir, pCentral_dir_offsets, pIndices[root], pIndices[child]))
                break;
            tmz_SWAP_UINT32(pIndices[root], pIndices[child]);
            root = child;
        }
        if (!start)
            break;
        start--;
    }

    end = size - 1;
    while (end > 0)
    {
        tmz_uint64 child, root = 0;
        tmz_SWAP_UINT32(pIndices[end], pIndices[0]);
        for (;;)
        {
            if ((child = (root << 1U) + 1U) >= end)
                break;
            child += (((child + 1U) < end) && tmz_zip_reader_filename_less(pCentral_dir, pCentral_dir_offsets, pIndices[child], pIndices[child + 1U]));
            if (!tmz_zip_reader_filename_less(pCentral_dir, pCentral_dir_offsets, pIndices[root], pIndices[child]))
                break;
            tmz_SWAP_UINT32(pIndices[root], pIndices[child]);
            root = child;
        }
        end--;
    }
}

static tmz_bool tmz_zip_reader_locate_header_sig(tmz_zip_archive *pZip, tmz_uint32 record_sig, tmz_uint32 record_size, tmz_int64 *pOfs)
{
    tmz_int64 cur_file_ofs;
    tmz_uint32 buf_u32[4096 / sizeof(tmz_uint32)];
    tmz_uint8 *pBuf = (tmz_uint8 *)buf_u32;

    /* Basic sanity checks - reject files which are too small */
    if (pZip->m_archive_size < record_size)
        return tmz_FALSE;

    /* Find the record by scanning the file from the end towards the beginning. */
    cur_file_ofs = tmz_MAX((tmz_int64)pZip->m_archive_size - (tmz_int64)sizeof(buf_u32), 0);
    for (;;)
    {
        int i, n = (int)tmz_MIN(sizeof(buf_u32), pZip->m_archive_size - cur_file_ofs);

        if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs, pBuf, n) != (tmz_uint)n)
            return tmz_FALSE;

        for (i = n - 4; i >= 0; --i)
        {
            tmz_uint s = tmz_READ_LE32(pBuf + i);
            if (s == record_sig)
            {
                if ((pZip->m_archive_size - (cur_file_ofs + i)) >= record_size)
                    break;
            }
        }

        if (i >= 0)
        {
            cur_file_ofs += i;
            break;
        }

        /* Give up if we've searched the entire file, or we've gone back "too far" (~64kb) */
        if ((!cur_file_ofs) || ((pZip->m_archive_size - cur_file_ofs) >= (tmz_UINT16_MAX + record_size)))
            return tmz_FALSE;

        cur_file_ofs = tmz_MAX(cur_file_ofs - (sizeof(buf_u32) - 3), 0);
    }

    *pOfs = cur_file_ofs;
    return tmz_TRUE;
}

static tmz_bool tmz_zip_reader_read_central_dir(tmz_zip_archive *pZip, tmz_uint flags)
{
    tmz_uint cdir_size = 0, cdir_entries_on_this_disk = 0, num_this_disk = 0, cdir_disk_index = 0;
    tmz_uint64 cdir_ofs = 0;
    tmz_int64 cur_file_ofs = 0;
    const tmz_uint8 *p;

    tmz_uint32 buf_u32[4096 / sizeof(tmz_uint32)];
    tmz_uint8 *pBuf = (tmz_uint8 *)buf_u32;
    tmz_bool sort_central_dir = ((flags & tmz_ZIP_FLAG_DO_NOT_SORT_CENTRAL_DIRECTORY) == 0);
    tmz_uint32 zip64_end_of_central_dir_locator_u32[(tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE + sizeof(tmz_uint32) - 1) / sizeof(tmz_uint32)];
    tmz_uint8 *pZip64_locator = (tmz_uint8 *)zip64_end_of_central_dir_locator_u32;

    tmz_uint32 zip64_end_of_central_dir_header_u32[(tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE + sizeof(tmz_uint32) - 1) / sizeof(tmz_uint32)];
    tmz_uint8 *pZip64_end_of_central_dir = (tmz_uint8 *)zip64_end_of_central_dir_header_u32;

    tmz_uint64 zip64_end_of_central_dir_ofs = 0;

    /* Basic sanity checks - reject files which are too small, and check the first 4 bytes of the file to make sure a local header is there. */
    if (pZip->m_archive_size < tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_NOT_AN_ARCHIVE);

    if (!tmz_zip_reader_locate_header_sig(pZip, tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIG, tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE, &cur_file_ofs))
        return tmz_zip_set_error(pZip, tmz_ZIP_FAILED_FINDING_CENTRAL_DIR);

    /* Read and verify the end of central directory record. */
    if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs, pBuf, tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE) != tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);

    if (tmz_READ_LE32(pBuf + tmz_ZIP_ECDH_SIG_OFS) != tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIG)
        return tmz_zip_set_error(pZip, tmz_ZIP_NOT_AN_ARCHIVE);

    if (cur_file_ofs >= (tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE + tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE))
    {
        if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs - tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE, pZip64_locator, tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE) == tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE)
        {
            if (tmz_READ_LE32(pZip64_locator + tmz_ZIP64_ECDL_SIG_OFS) == tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIG)
            {
                zip64_end_of_central_dir_ofs = tmz_READ_LE64(pZip64_locator + tmz_ZIP64_ECDL_REL_OFS_TO_ZIP64_ECDR_OFS);
                if (zip64_end_of_central_dir_ofs > (pZip->m_archive_size - tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE))
                    return tmz_zip_set_error(pZip, tmz_ZIP_NOT_AN_ARCHIVE);

                if (pZip->m_pRead(pZip->m_pIO_opaque, zip64_end_of_central_dir_ofs, pZip64_end_of_central_dir, tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE) == tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE)
                {
                    if (tmz_READ_LE32(pZip64_end_of_central_dir + tmz_ZIP64_ECDH_SIG_OFS) == tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIG)
                    {
                        pZip->m_pState->m_zip64 = tmz_TRUE;
                    }
                }
            }
        }
    }

    pZip->m_total_files = tmz_READ_LE16(pBuf + tmz_ZIP_ECDH_CDIR_TOTAL_ENTRIES_OFS);
    cdir_entries_on_this_disk = tmz_READ_LE16(pBuf + tmz_ZIP_ECDH_CDIR_NUM_ENTRIES_ON_DISK_OFS);
    num_this_disk = tmz_READ_LE16(pBuf + tmz_ZIP_ECDH_NUM_THIS_DISK_OFS);
    cdir_disk_index = tmz_READ_LE16(pBuf + tmz_ZIP_ECDH_NUM_DISK_CDIR_OFS);
    cdir_size = tmz_READ_LE32(pBuf + tmz_ZIP_ECDH_CDIR_SIZE_OFS);
    cdir_ofs = tmz_READ_LE32(pBuf + tmz_ZIP_ECDH_CDIR_OFS_OFS);

    if (pZip->m_pState->m_zip64)
    {
        tmz_uint32 zip64_total_num_of_disks = tmz_READ_LE32(pZip64_locator + tmz_ZIP64_ECDL_TOTAL_NUMBER_OF_DISKS_OFS);
        tmz_uint64 zip64_cdir_total_entries = tmz_READ_LE64(pZip64_end_of_central_dir + tmz_ZIP64_ECDH_CDIR_TOTAL_ENTRIES_OFS);
        tmz_uint64 zip64_cdir_total_entries_on_this_disk = tmz_READ_LE64(pZip64_end_of_central_dir + tmz_ZIP64_ECDH_CDIR_NUM_ENTRIES_ON_DISK_OFS);
        tmz_uint64 zip64_size_of_end_of_central_dir_record = tmz_READ_LE64(pZip64_end_of_central_dir + tmz_ZIP64_ECDH_SIZE_OF_RECORD_OFS);
        tmz_uint64 zip64_size_of_central_directory = tmz_READ_LE64(pZip64_end_of_central_dir + tmz_ZIP64_ECDH_CDIR_SIZE_OFS);

        if (zip64_size_of_end_of_central_dir_record < (tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE - 12))
            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

        if (zip64_total_num_of_disks != 1U)
            return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_MULTIDISK);

        /* Check for miniz's practical limits */
        if (zip64_cdir_total_entries > tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);

        pZip->m_total_files = (tmz_uint32)zip64_cdir_total_entries;

        if (zip64_cdir_total_entries_on_this_disk > tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);

        cdir_entries_on_this_disk = (tmz_uint32)zip64_cdir_total_entries_on_this_disk;

        /* Check for miniz's current practical limits (sorry, this should be enough for millions of files) */
        if (zip64_size_of_central_directory > tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_CDIR_SIZE);

        cdir_size = (tmz_uint32)zip64_size_of_central_directory;

        num_this_disk = tmz_READ_LE32(pZip64_end_of_central_dir + tmz_ZIP64_ECDH_NUM_THIS_DISK_OFS);

        cdir_disk_index = tmz_READ_LE32(pZip64_end_of_central_dir + tmz_ZIP64_ECDH_NUM_DISK_CDIR_OFS);

        cdir_ofs = tmz_READ_LE64(pZip64_end_of_central_dir + tmz_ZIP64_ECDH_CDIR_OFS_OFS);
    }

    if (pZip->m_total_files != cdir_entries_on_this_disk)
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_MULTIDISK);

    if (((num_this_disk | cdir_disk_index) != 0) && ((num_this_disk != 1) || (cdir_disk_index != 1)))
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_MULTIDISK);

    if (cdir_size < pZip->m_total_files * tmz_ZIP_CENTRAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    if ((cdir_ofs + (tmz_uint64)cdir_size) > pZip->m_archive_size)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    pZip->m_central_directory_file_ofs = cdir_ofs;

    if (pZip->m_total_files)
    {
        tmz_uint i, n;
        /* Read the entire central directory into a heap block, and allocate another heap block to hold the unsorted central dir file record offsets, and possibly another to hold the sorted indices. */
        if ((!tmz_zip_array_resize(pZip, &pZip->m_pState->m_central_dir, cdir_size, tmz_FALSE)) ||
            (!tmz_zip_array_resize(pZip, &pZip->m_pState->m_central_dir_offsets, pZip->m_total_files, tmz_FALSE)))
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

        if (sort_central_dir)
        {
            if (!tmz_zip_array_resize(pZip, &pZip->m_pState->m_sorted_central_dir_offsets, pZip->m_total_files, tmz_FALSE))
                return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }

        if (pZip->m_pRead(pZip->m_pIO_opaque, cdir_ofs, pZip->m_pState->m_central_dir.m_p, cdir_size) != cdir_size)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);

        /* Now create an index into the central directory file records, do some basic sanity checking on each record */
        p = (const tmz_uint8 *)pZip->m_pState->m_central_dir.m_p;
        for (n = cdir_size, i = 0; i < pZip->m_total_files; ++i)
        {
            tmz_uint total_header_size, disk_index, bit_flags, filename_size, ext_data_size;
            tmz_uint64 comp_size, decomp_size, local_header_ofs;

            if ((n < tmz_ZIP_CENTRAL_DIR_HEADER_SIZE) || (tmz_READ_LE32(p) != tmz_ZIP_CENTRAL_DIR_HEADER_SIG))
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

            tmz_ZIP_ARRAY_ELEMENT(&pZip->m_pState->m_central_dir_offsets, tmz_uint32, i) = (tmz_uint32)(p - (const tmz_uint8 *)pZip->m_pState->m_central_dir.m_p);

            if (sort_central_dir)
                tmz_ZIP_ARRAY_ELEMENT(&pZip->m_pState->m_sorted_central_dir_offsets, tmz_uint32, i) = i;

            comp_size = tmz_READ_LE32(p + tmz_ZIP_CDH_COMPRESSED_SIZE_OFS);
            decomp_size = tmz_READ_LE32(p + tmz_ZIP_CDH_DECOMPRESSED_SIZE_OFS);
            local_header_ofs = tmz_READ_LE32(p + tmz_ZIP_CDH_LOCAL_HEADER_OFS);
            filename_size = tmz_READ_LE16(p + tmz_ZIP_CDH_FILENAME_LEN_OFS);
            ext_data_size = tmz_READ_LE16(p + tmz_ZIP_CDH_EXTRA_LEN_OFS);

            if ((!pZip->m_pState->m_zip64_has_extended_info_fields) &&
                (ext_data_size) &&
                (tmz_MAX(tmz_MAX(comp_size, decomp_size), local_header_ofs) == tmz_UINT32_MAX))
            {
                /* Attempt to find zip64 extended information field in the entry's extra data */
                tmz_uint32 extra_size_remaining = ext_data_size;

                if (extra_size_remaining)
                {
                    const tmz_uint8 *pExtra_data = p + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + filename_size;

                    do
                    {
						tmz_uint32 field_id;
						tmz_uint32 field_data_size;

                        if (extra_size_remaining < (sizeof(tmz_uint16) * 2))
                            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

                        field_id = tmz_READ_LE16(pExtra_data);
                        field_data_size = tmz_READ_LE16(pExtra_data + sizeof(tmz_uint16));

                        if ((field_data_size + sizeof(tmz_uint16) * 2) > extra_size_remaining)
                            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

                        if (field_id == tmz_ZIP64_EXTENDED_INFORMATION_FIELD_HEADER_ID)
                        {
                            /* Ok, the archive didn't have any zip64 headers but it uses a zip64 extended information field so mark it as zip64 anyway (this can occur with infozip's zip util when it reads compresses files from stdin). */
                            pZip->m_pState->m_zip64 = tmz_TRUE;
                            pZip->m_pState->m_zip64_has_extended_info_fields = tmz_TRUE;
                            break;
                        }

                        pExtra_data += sizeof(tmz_uint16) * 2 + field_data_size;
                        extra_size_remaining = extra_size_remaining - sizeof(tmz_uint16) * 2 - field_data_size;
                    } while (extra_size_remaining);
                }
            }

            /* I've seen archives that aren't marked as zip64 that uses zip64 ext data, argh */
            if ((comp_size != tmz_UINT32_MAX) && (decomp_size != tmz_UINT32_MAX))
            {
                if (((!tmz_READ_LE32(p + tmz_ZIP_CDH_METHOD_OFS)) && (decomp_size != comp_size)) || (decomp_size && !comp_size))
                    return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);
            }

            disk_index = tmz_READ_LE16(p + tmz_ZIP_CDH_DISK_START_OFS);
            if ((disk_index == tmz_UINT16_MAX) || ((disk_index != num_this_disk) && (disk_index != 1)))
                return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_MULTIDISK);

            if (comp_size != tmz_UINT32_MAX)
            {
                if (((tmz_uint64)tmz_READ_LE32(p + tmz_ZIP_CDH_LOCAL_HEADER_OFS) + tmz_ZIP_LOCAL_DIR_HEADER_SIZE + comp_size) > pZip->m_archive_size)
                    return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);
            }

            bit_flags = tmz_READ_LE16(p + tmz_ZIP_CDH_BIT_FLAG_OFS);
            if (bit_flags & tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_LOCAL_DIR_IS_MASKED)
                return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_ENCRYPTION);

            if ((total_header_size = tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + tmz_READ_LE16(p + tmz_ZIP_CDH_FILENAME_LEN_OFS) + tmz_READ_LE16(p + tmz_ZIP_CDH_EXTRA_LEN_OFS) + tmz_READ_LE16(p + tmz_ZIP_CDH_COMMENT_LEN_OFS)) > n)
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

            n -= total_header_size;
            p += total_header_size;
        }
    }

    if (sort_central_dir)
        tmz_zip_reader_sort_central_dir_offsets_by_filename(pZip);

    return tmz_TRUE;
}

void tmz_zip_zero_struct(tmz_zip_archive *pZip)
{
    if (pZip)
        tmz_CLEAR_OBJ(*pZip);
}

static tmz_bool tmz_zip_reader_end_internal(tmz_zip_archive *pZip, tmz_bool set_last_error)
{
    tmz_bool status = tmz_TRUE;

    if (!pZip)
        return tmz_FALSE;

    if ((!pZip->m_pState) || (!pZip->m_pAlloc) || (!pZip->m_pFree) || (pZip->m_zip_mode != tmz_ZIP_MODE_READING))
    {
        if (set_last_error)
            pZip->m_last_error = tmz_ZIP_INVALID_PARAMETER;

        return tmz_FALSE;
    }

    if (pZip->m_pState)
    {
        tmz_zip_internal_state *pState = pZip->m_pState;
        pZip->m_pState = NULL;

        tmz_zip_array_clear(pZip, &pState->m_central_dir);
        tmz_zip_array_clear(pZip, &pState->m_central_dir_offsets);
        tmz_zip_array_clear(pZip, &pState->m_sorted_central_dir_offsets);

#ifndef tminiz_NO_STDIO
        if (pState->m_pFile)
        {
            if (pZip->m_zip_type == tmz_ZIP_TYPE_FILE)
            {
                if (tmz_FCLOSE(pState->m_pFile) == EOF)
                {
                    if (set_last_error)
                        pZip->m_last_error = tmz_ZIP_FILE_CLOSE_FAILED;
                    status = tmz_FALSE;
                }
            }
            pState->m_pFile = NULL;
        }
#endif /* #ifndef tminiz_NO_STDIO */

        pZip->m_pFree(pZip->m_pAlloc_opaque, pState);
    }
    pZip->m_zip_mode = tmz_ZIP_MODE_INVALID;

    return status;
}

tmz_bool tmz_zip_reader_end(tmz_zip_archive *pZip)
{
    return tmz_zip_reader_end_internal(pZip, tmz_TRUE);
}
tmz_bool tmz_zip_reader_init(tmz_zip_archive *pZip, tmz_uint64 size, tmz_uint flags)
{
    if ((!pZip) || (!pZip->m_pRead))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (!tmz_zip_reader_init_internal(pZip, flags))
        return tmz_FALSE;

    pZip->m_zip_type = tmz_ZIP_TYPE_USER;
    pZip->m_archive_size = size;

    if (!tmz_zip_reader_read_central_dir(pZip, flags))
    {
        tmz_zip_reader_end_internal(pZip, tmz_FALSE);
        return tmz_FALSE;
    }

    return tmz_TRUE;
}

static size_t tmz_zip_mem_read_func(void *pOpaque, tmz_uint64 file_ofs, void *pBuf, size_t n)
{
    tmz_zip_archive *pZip = (tmz_zip_archive *)pOpaque;
    size_t s = (file_ofs >= pZip->m_archive_size) ? 0 : (size_t)tmz_MIN(pZip->m_archive_size - file_ofs, n);
    memcpy(pBuf, (const tmz_uint8 *)pZip->m_pState->m_pMem + file_ofs, s);
    return s;
}

tmz_bool tmz_zip_reader_init_mem(tmz_zip_archive *pZip, const void *pMem, size_t size, tmz_uint flags)
{
    if (!pMem)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (size < tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_NOT_AN_ARCHIVE);

    if (!tmz_zip_reader_init_internal(pZip, flags))
        return tmz_FALSE;

    pZip->m_zip_type = tmz_ZIP_TYPE_MEMORY;
    pZip->m_archive_size = size;
    pZip->m_pRead = tmz_zip_mem_read_func;
    pZip->m_pIO_opaque = pZip;
	pZip->m_pNeeds_keepalive = NULL;

#ifdef __cplusplus
    pZip->m_pState->m_pMem = const_cast<void *>(pMem);
#else
    pZip->m_pState->m_pMem = (void *)pMem;
#endif

    pZip->m_pState->m_mem_size = size;

    if (!tmz_zip_reader_read_central_dir(pZip, flags))
    {
        tmz_zip_reader_end_internal(pZip, tmz_FALSE);
        return tmz_FALSE;
    }

    return tmz_TRUE;
}

#ifndef tminiz_NO_STDIO
static size_t tmz_zip_file_read_func(void *pOpaque, tmz_uint64 file_ofs, void *pBuf, size_t n)
{
    tmz_zip_archive *pZip = (tmz_zip_archive *)pOpaque;
    tmz_int64 cur_ofs = tmz_FTELL64(pZip->m_pState->m_pFile);

    file_ofs += pZip->m_pState->m_file_archive_start_ofs;

    if (((tmz_int64)file_ofs < 0) || (((cur_ofs != (tmz_int64)file_ofs)) && (tmz_FSEEK64(pZip->m_pState->m_pFile, (tmz_int64)file_ofs, SEEK_SET))))
        return 0;

    return tmz_FREAD(pBuf, 1, n, pZip->m_pState->m_pFile);
}

tmz_bool tmz_zip_reader_init_file(tmz_zip_archive *pZip, const char *pFilename, tmz_uint32 flags)
{
    return tmz_zip_reader_init_file_v2(pZip, pFilename, flags, 0, 0);
}

tmz_bool tmz_zip_reader_init_file_v2(tmz_zip_archive *pZip, const char *pFilename, tmz_uint flags, tmz_uint64 file_start_ofs, tmz_uint64 archive_size)
{
	tmz_uint64 file_size;
	tmz_FILE *pFile;

    if ((!pZip) || (!pFilename) || ((archive_size) && (archive_size < tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE)))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

	pFile = tmz_FOPEN(pFilename, "rb");
    if (!pFile)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_OPEN_FAILED);

    file_size = archive_size;
    if (!file_size)
    {
        if (tmz_FSEEK64(pFile, 0, SEEK_END))
        {
            tmz_FCLOSE(pFile);
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_SEEK_FAILED);
        }

        file_size = tmz_FTELL64(pFile);
    }

    /* TODO: Better sanity check archive_size and the # of actual remaining bytes */

    if (file_size < tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_NOT_AN_ARCHIVE);

    if (!tmz_zip_reader_init_internal(pZip, flags))
    {
        tmz_FCLOSE(pFile);
        return tmz_FALSE;
    }

    pZip->m_zip_type = tmz_ZIP_TYPE_FILE;
    pZip->m_pRead = tmz_zip_file_read_func;
    pZip->m_pIO_opaque = pZip;
    pZip->m_pState->m_pFile = pFile;
    pZip->m_archive_size = file_size;
    pZip->m_pState->m_file_archive_start_ofs = file_start_ofs;

    if (!tmz_zip_reader_read_central_dir(pZip, flags))
    {
        tmz_zip_reader_end_internal(pZip, tmz_FALSE);
        return tmz_FALSE;
    }

    return tmz_TRUE;
}

tmz_bool tmz_zip_reader_init_cfile(tmz_zip_archive *pZip, tmz_FILE *pFile, tmz_uint64 archive_size, tmz_uint flags)
{
    tmz_uint64 cur_file_ofs;

    if ((!pZip) || (!pFile))
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_OPEN_FAILED);

    cur_file_ofs = tmz_FTELL64(pFile);

    if (!archive_size)
    {
        if (tmz_FSEEK64(pFile, 0, SEEK_END))
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_SEEK_FAILED);

        archive_size = tmz_FTELL64(pFile) - cur_file_ofs;

        if (archive_size < tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE)
            return tmz_zip_set_error(pZip, tmz_ZIP_NOT_AN_ARCHIVE);
    }

    if (!tmz_zip_reader_init_internal(pZip, flags))
        return tmz_FALSE;

    pZip->m_zip_type = tmz_ZIP_TYPE_CFILE;
    pZip->m_pRead = tmz_zip_file_read_func;

    pZip->m_pIO_opaque = pZip;
    pZip->m_pState->m_pFile = pFile;
    pZip->m_archive_size = archive_size;
    pZip->m_pState->m_file_archive_start_ofs = cur_file_ofs;

    if (!tmz_zip_reader_read_central_dir(pZip, flags))
    {
        tmz_zip_reader_end_internal(pZip, tmz_FALSE);
        return tmz_FALSE;
    }

    return tmz_TRUE;
}

#endif /* #ifndef tminiz_NO_STDIO */

static tmz_FORCEINLINE const tmz_uint8 *tmz_zip_get_cdh(tmz_zip_archive *pZip, tmz_uint file_index)
{
    if ((!pZip) || (!pZip->m_pState) || (file_index >= pZip->m_total_files))
        return NULL;
    return &tmz_ZIP_ARRAY_ELEMENT(&pZip->m_pState->m_central_dir, tmz_uint8, tmz_ZIP_ARRAY_ELEMENT(&pZip->m_pState->m_central_dir_offsets, tmz_uint32, file_index));
}

tmz_bool tmz_zip_reader_is_file_encrypted(tmz_zip_archive *pZip, tmz_uint file_index)
{
    tmz_uint m_bit_flag;
    const tmz_uint8 *p = tmz_zip_get_cdh(pZip, file_index);
    if (!p)
    {
        tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
        return tmz_FALSE;
    }

    m_bit_flag = tmz_READ_LE16(p + tmz_ZIP_CDH_BIT_FLAG_OFS);
    return (m_bit_flag & (tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_IS_ENCRYPTED | tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_USES_STRONG_ENCRYPTION)) != 0;
}

tmz_bool tmz_zip_reader_is_file_supported(tmz_zip_archive *pZip, tmz_uint file_index)
{
    tmz_uint bit_flag;
    tmz_uint method;

    const tmz_uint8 *p = tmz_zip_get_cdh(pZip, file_index);
    if (!p)
    {
        tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
        return tmz_FALSE;
    }

    method = tmz_READ_LE16(p + tmz_ZIP_CDH_METHOD_OFS);
    bit_flag = tmz_READ_LE16(p + tmz_ZIP_CDH_BIT_FLAG_OFS);

    if ((method != 0) && (method != tmz_DEFLATED))
    {
        tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_METHOD);
        return tmz_FALSE;
    }

    if (bit_flag & (tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_IS_ENCRYPTED | tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_USES_STRONG_ENCRYPTION))
    {
        tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_ENCRYPTION);
        return tmz_FALSE;
    }

    if (bit_flag & tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_COMPRESSED_PATCH_FLAG)
    {
        tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_FEATURE);
        return tmz_FALSE;
    }

    return tmz_TRUE;
}

tmz_bool tmz_zip_reader_is_file_a_directory(tmz_zip_archive *pZip, tmz_uint file_index)
{
    tmz_uint filename_len, attribute_mapping_id, external_attr;
    const tmz_uint8 *p = tmz_zip_get_cdh(pZip, file_index);
    if (!p)
    {
        tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
        return tmz_FALSE;
    }

    filename_len = tmz_READ_LE16(p + tmz_ZIP_CDH_FILENAME_LEN_OFS);
    if (filename_len)
    {
        if (*(p + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + filename_len - 1) == '/')
            return tmz_TRUE;
    }

    /* Bugfix: This code was also checking if the internal attribute was non-zero, which wasn't correct. */
    /* Most/all zip writers (hopefully) set DOS file/directory attributes in the low 16-bits, so check for the DOS directory flag and ignore the source OS ID in the created by field. */
    /* FIXME: Remove this check? Is it necessary - we already check the filename. */
    attribute_mapping_id = tmz_READ_LE16(p + tmz_ZIP_CDH_VERSION_MADE_BY_OFS) >> 8;
    (void)attribute_mapping_id;

    external_attr = tmz_READ_LE32(p + tmz_ZIP_CDH_EXTERNAL_ATTR_OFS);
    if ((external_attr & tmz_ZIP_DOS_DIR_ATTRIBUTE_BITFLAG) != 0)
    {
        return tmz_TRUE;
    }

    return tmz_FALSE;
}

static tmz_bool tmz_zip_file_stat_internal(tmz_zip_archive *pZip, tmz_uint file_index, const tmz_uint8 *pCentral_dir_header, tmz_zip_archive_file_stat *pStat, tmz_bool *pFound_zip64_extra_data)
{
    tmz_uint n;
    const tmz_uint8 *p = pCentral_dir_header;

    if (pFound_zip64_extra_data)
        *pFound_zip64_extra_data = tmz_FALSE;

    if ((!p) || (!pStat))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    /* Extract fields from the central directory record. */
    pStat->m_file_index = file_index;
    pStat->m_central_dir_ofs = tmz_ZIP_ARRAY_ELEMENT(&pZip->m_pState->m_central_dir_offsets, tmz_uint32, file_index);
    pStat->m_version_made_by = tmz_READ_LE16(p + tmz_ZIP_CDH_VERSION_MADE_BY_OFS);
    pStat->m_version_needed = tmz_READ_LE16(p + tmz_ZIP_CDH_VERSION_NEEDED_OFS);
    pStat->m_bit_flag = tmz_READ_LE16(p + tmz_ZIP_CDH_BIT_FLAG_OFS);
    pStat->m_method = tmz_READ_LE16(p + tmz_ZIP_CDH_METHOD_OFS);
#ifndef tminiz_NO_TIME
    pStat->m_time = tmz_zip_dos_to_time_t(tmz_READ_LE16(p + tmz_ZIP_CDH_FILE_TIME_OFS), tmz_READ_LE16(p + tmz_ZIP_CDH_FILE_DATE_OFS));
#endif
    pStat->m_crc32 = tmz_READ_LE32(p + tmz_ZIP_CDH_CRC32_OFS);
    pStat->m_comp_size = tmz_READ_LE32(p + tmz_ZIP_CDH_COMPRESSED_SIZE_OFS);
    pStat->m_uncomp_size = tmz_READ_LE32(p + tmz_ZIP_CDH_DECOMPRESSED_SIZE_OFS);
    pStat->m_internal_attr = tmz_READ_LE16(p + tmz_ZIP_CDH_INTERNAL_ATTR_OFS);
    pStat->m_external_attr = tmz_READ_LE32(p + tmz_ZIP_CDH_EXTERNAL_ATTR_OFS);
    pStat->m_local_header_ofs = tmz_READ_LE32(p + tmz_ZIP_CDH_LOCAL_HEADER_OFS);

    /* Copy as much of the filename and comment as possible. */
    n = tmz_READ_LE16(p + tmz_ZIP_CDH_FILENAME_LEN_OFS);
    n = tmz_MIN(n, tmz_ZIP_MAX_ARCHIVE_FILENAME_SIZE - 1);
    memcpy(pStat->m_filename, p + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE, n);
    pStat->m_filename[n] = '\0';

    n = tmz_READ_LE16(p + tmz_ZIP_CDH_COMMENT_LEN_OFS);
    n = tmz_MIN(n, tmz_ZIP_MAX_ARCHIVE_FILE_COMMENT_SIZE - 1);
    pStat->m_comment_size = n;
    memcpy(pStat->m_comment, p + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + tmz_READ_LE16(p + tmz_ZIP_CDH_FILENAME_LEN_OFS) + tmz_READ_LE16(p + tmz_ZIP_CDH_EXTRA_LEN_OFS), n);
    pStat->m_comment[n] = '\0';

    /* Set some flags for convienance */
    pStat->m_is_directory = tmz_zip_reader_is_file_a_directory(pZip, file_index);
    pStat->m_is_encrypted = tmz_zip_reader_is_file_encrypted(pZip, file_index);
    pStat->m_is_supported = tmz_zip_reader_is_file_supported(pZip, file_index);

    /* See if we need to read any zip64 extended information fields. */
    /* Confusingly, these zip64 fields can be present even on non-zip64 archives (Debian zip on a huge files from stdin piped to stdout creates them). */
    if (tmz_MAX(tmz_MAX(pStat->m_comp_size, pStat->m_uncomp_size), pStat->m_local_header_ofs) == tmz_UINT32_MAX)
    {
        /* Attempt to find zip64 extended information field in the entry's extra data */
        tmz_uint32 extra_size_remaining = tmz_READ_LE16(p + tmz_ZIP_CDH_EXTRA_LEN_OFS);

        if (extra_size_remaining)
        {
            const tmz_uint8 *pExtra_data = p + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + tmz_READ_LE16(p + tmz_ZIP_CDH_FILENAME_LEN_OFS);

            do
            {
				tmz_uint32 field_id;
				tmz_uint32 field_data_size;

                if (extra_size_remaining < (sizeof(tmz_uint16) * 2))
                    return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

                field_id = tmz_READ_LE16(pExtra_data);
                field_data_size = tmz_READ_LE16(pExtra_data + sizeof(tmz_uint16));

                if ((field_data_size + sizeof(tmz_uint16) * 2) > extra_size_remaining)
                    return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

                if (field_id == tmz_ZIP64_EXTENDED_INFORMATION_FIELD_HEADER_ID)
                {
                    const tmz_uint8 *pField_data = pExtra_data + sizeof(tmz_uint16) * 2;
                    tmz_uint32 field_data_remaining = field_data_size;

                    if (pFound_zip64_extra_data)
                        *pFound_zip64_extra_data = tmz_TRUE;

                    if (pStat->m_uncomp_size == tmz_UINT32_MAX)
                    {
                        if (field_data_remaining < sizeof(tmz_uint64))
                            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

                        pStat->m_uncomp_size = tmz_READ_LE64(pField_data);
                        pField_data += sizeof(tmz_uint64);
                        field_data_remaining -= sizeof(tmz_uint64);
                    }

                    if (pStat->m_comp_size == tmz_UINT32_MAX)
                    {
                        if (field_data_remaining < sizeof(tmz_uint64))
                            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

                        pStat->m_comp_size = tmz_READ_LE64(pField_data);
                        pField_data += sizeof(tmz_uint64);
                        field_data_remaining -= sizeof(tmz_uint64);
                    }

                    if (pStat->m_local_header_ofs == tmz_UINT32_MAX)
                    {
                        if (field_data_remaining < sizeof(tmz_uint64))
                            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

                        pStat->m_local_header_ofs = tmz_READ_LE64(pField_data);
                        pField_data += sizeof(tmz_uint64);
                        field_data_remaining -= sizeof(tmz_uint64);
                    }

                    break;
                }

                pExtra_data += sizeof(tmz_uint16) * 2 + field_data_size;
                extra_size_remaining = extra_size_remaining - sizeof(tmz_uint16) * 2 - field_data_size;
            } while (extra_size_remaining);
        }
    }

    return tmz_TRUE;
}

static tmz_FORCEINLINE tmz_bool tmz_zip_string_equal(const char *pA, const char *pB, tmz_uint len, tmz_uint flags)
{
    tmz_uint i;
    if (flags & tmz_ZIP_FLAG_CASE_SENSITIVE)
        return 0 == memcmp(pA, pB, len);
    for (i = 0; i < len; ++i)
        if (tmz_TOLOWER(pA[i]) != tmz_TOLOWER(pB[i]))
            return tmz_FALSE;
    return tmz_TRUE;
}

static tmz_FORCEINLINE int tmz_zip_filename_compare(const tmz_zip_array *pCentral_dir_array, const tmz_zip_array *pCentral_dir_offsets, tmz_uint l_index, const char *pR, tmz_uint r_len)
{
    const tmz_uint8 *pL = &tmz_ZIP_ARRAY_ELEMENT(pCentral_dir_array, tmz_uint8, tmz_ZIP_ARRAY_ELEMENT(pCentral_dir_offsets, tmz_uint32, l_index)), *pE;
    tmz_uint l_len = tmz_READ_LE16(pL + tmz_ZIP_CDH_FILENAME_LEN_OFS);
    tmz_uint8 l = 0, r = 0;
    pL += tmz_ZIP_CENTRAL_DIR_HEADER_SIZE;
    pE = pL + tmz_MIN(l_len, r_len);
    while (pL < pE)
    {
        if ((l = tmz_TOLOWER(*pL)) != (r = tmz_TOLOWER(*pR)))
            break;
        pL++;
        pR++;
    }
    return (pL == pE) ? (int)(l_len - r_len) : (l - r);
}

static tmz_bool tmz_zip_locate_file_binary_search(tmz_zip_archive *pZip, const char *pFilename, tmz_uint32 *pIndex)
{
    tmz_zip_internal_state *pState = pZip->m_pState;
    const tmz_zip_array *pCentral_dir_offsets = &pState->m_central_dir_offsets;
    const tmz_zip_array *pCentral_dir = &pState->m_central_dir;
    tmz_uint32 *pIndices = &tmz_ZIP_ARRAY_ELEMENT(&pState->m_sorted_central_dir_offsets, tmz_uint32, 0);
    const uint32_t size = pZip->m_total_files;
    const tmz_uint filename_len = (tmz_uint)strlen(pFilename);

    if (pIndex)
        *pIndex = 0;

    if (size)
    {
        /* yes I could use uint32_t's, but then we would have to add some special case checks in the loop, argh, and */
        /* honestly the major expense here on 32-bit CPU's will still be the filename compare */
        tmz_int64 l = 0, h = (tmz_int64)size - 1;

        while (l <= h)
        {
            tmz_int64 m = l + ((h - l) >> 1);
            uint32_t file_index = pIndices[(uint32_t)m];

            int comp = tmz_zip_filename_compare(pCentral_dir, pCentral_dir_offsets, file_index, pFilename, filename_len);
            if (!comp)
            {
                if (pIndex)
                    *pIndex = file_index;
                return tmz_TRUE;
            }
            else if (comp < 0)
                l = m + 1;
            else
                h = m - 1;
        }
    }

    return tmz_zip_set_error(pZip, tmz_ZIP_FILE_NOT_FOUND);
}

int tmz_zip_reader_locate_file(tmz_zip_archive *pZip, const char *pName, const char *pComment, tmz_uint flags)
{
    tmz_uint32 index;
    if (!tmz_zip_reader_locate_file_v2(pZip, pName, pComment, flags, &index))
        return -1;
    else
        return (int)index;
}

tmz_bool tmz_zip_reader_locate_file_v2(tmz_zip_archive *pZip, const char *pName, const char *pComment, tmz_uint flags, tmz_uint32 *pIndex)
{
    tmz_uint file_index;
    size_t name_len, comment_len;

    if (pIndex)
        *pIndex = 0;

    if ((!pZip) || (!pZip->m_pState) || (!pName))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    /* See if we can use a binary search */
    if (((pZip->m_pState->m_init_flags & tmz_ZIP_FLAG_DO_NOT_SORT_CENTRAL_DIRECTORY) == 0) &&
        (pZip->m_zip_mode == tmz_ZIP_MODE_READING) &&
        ((flags & (tmz_ZIP_FLAG_IGNORE_PATH | tmz_ZIP_FLAG_CASE_SENSITIVE)) == 0) && (!pComment) && (pZip->m_pState->m_sorted_central_dir_offsets.m_size))
    {
        return tmz_zip_locate_file_binary_search(pZip, pName, pIndex);
    }

    /* Locate the entry by scanning the entire central directory */
    name_len = strlen(pName);
    if (name_len > tmz_UINT16_MAX)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    comment_len = pComment ? strlen(pComment) : 0;
    if (comment_len > tmz_UINT16_MAX)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    for (file_index = 0; file_index < pZip->m_total_files; file_index++)
    {
        const tmz_uint8 *pHeader = &tmz_ZIP_ARRAY_ELEMENT(&pZip->m_pState->m_central_dir, tmz_uint8, tmz_ZIP_ARRAY_ELEMENT(&pZip->m_pState->m_central_dir_offsets, tmz_uint32, file_index));
        tmz_uint filename_len = tmz_READ_LE16(pHeader + tmz_ZIP_CDH_FILENAME_LEN_OFS);
        const char *pFilename = (const char *)pHeader + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE;
        if (filename_len < name_len)
            continue;
        if (comment_len)
        {
            tmz_uint file_extra_len = tmz_READ_LE16(pHeader + tmz_ZIP_CDH_EXTRA_LEN_OFS), file_comment_len = tmz_READ_LE16(pHeader + tmz_ZIP_CDH_COMMENT_LEN_OFS);
            const char *pFile_comment = pFilename + filename_len + file_extra_len;
            if ((file_comment_len != comment_len) || (!tmz_zip_string_equal(pComment, pFile_comment, file_comment_len, flags)))
                continue;
        }
        if ((flags & tmz_ZIP_FLAG_IGNORE_PATH) && (filename_len))
        {
            int ofs = filename_len - 1;
            do
            {
                if ((pFilename[ofs] == '/') || (pFilename[ofs] == '\\') || (pFilename[ofs] == ':'))
                    break;
            } while (--ofs >= 0);
            ofs++;
            pFilename += ofs;
            filename_len -= ofs;
        }
        if ((filename_len == name_len) && (tmz_zip_string_equal(pName, pFilename, filename_len, flags)))
        {
            if (pIndex)
                *pIndex = file_index;
            return tmz_TRUE;
        }
    }

    return tmz_zip_set_error(pZip, tmz_ZIP_FILE_NOT_FOUND);
}

tmz_bool tmz_zip_reader_extract_to_mem_no_alloc(tmz_zip_archive *pZip, tmz_uint file_index, void *pBuf, size_t buf_size, tmz_uint flags, void *pUser_read_buf, size_t user_read_buf_size)
{
    int status = TINFL_STATUS_DONE;
    tmz_uint64 needed_size, cur_file_ofs, comp_remaining, out_buf_ofs = 0, read_buf_size, read_buf_ofs = 0, read_buf_avail;
    tmz_zip_archive_file_stat file_stat;
    void *pRead_buf;
    tmz_uint32 local_header_u32[(tmz_ZIP_LOCAL_DIR_HEADER_SIZE + sizeof(tmz_uint32) - 1) / sizeof(tmz_uint32)];
    tmz_uint8 *pLocal_header = (tmz_uint8 *)local_header_u32;
    tinfl_decompressor inflator;

    if ((!pZip) || (!pZip->m_pState) || ((buf_size) && (!pBuf)) || ((user_read_buf_size) && (!pUser_read_buf)) || (!pZip->m_pRead))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (!tmz_zip_reader_file_stat(pZip, file_index, &file_stat))
        return tmz_FALSE;

    /* A directory or zero length file */
    if ((file_stat.m_is_directory) || (!file_stat.m_comp_size))
        return tmz_TRUE;

    /* Encryption and patch files are not supported. */
    if (file_stat.m_bit_flag & (tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_IS_ENCRYPTED | tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_USES_STRONG_ENCRYPTION | tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_COMPRESSED_PATCH_FLAG))
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_ENCRYPTION);

    /* This function only supports decompressing stored and deflate. */
    if ((!(flags & tmz_ZIP_FLAG_COMPRESSED_DATA)) && (file_stat.m_method != 0) && (file_stat.m_method != tmz_DEFLATED))
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_METHOD);

    /* Ensure supplied output buffer is large enough. */
    needed_size = (flags & tmz_ZIP_FLAG_COMPRESSED_DATA) ? file_stat.m_comp_size : file_stat.m_uncomp_size;
    if (buf_size < needed_size)
        return tmz_zip_set_error(pZip, tmz_ZIP_BUF_TOO_SMALL);

    /* Read and parse the local directory entry. */
    cur_file_ofs = file_stat.m_local_header_ofs;
    if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs, pLocal_header, tmz_ZIP_LOCAL_DIR_HEADER_SIZE) != tmz_ZIP_LOCAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);

    if (tmz_READ_LE32(pLocal_header) != tmz_ZIP_LOCAL_DIR_HEADER_SIG)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    cur_file_ofs += tmz_ZIP_LOCAL_DIR_HEADER_SIZE + tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_FILENAME_LEN_OFS) + tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_EXTRA_LEN_OFS);
    if ((cur_file_ofs + file_stat.m_comp_size) > pZip->m_archive_size)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    if ((flags & tmz_ZIP_FLAG_COMPRESSED_DATA) || (!file_stat.m_method))
    {
        /* The file is stored or the caller has requested the compressed data. */
        if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs, pBuf, (size_t)needed_size) != needed_size)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);

#ifndef tminiz_DISABLE_ZIP_READER_CRC32_CHECKS
        if ((flags & tmz_ZIP_FLAG_COMPRESSED_DATA) == 0)
        {
            if (tmz_crc32(tmz_CRC32_INIT, (const tmz_uint8 *)pBuf, (size_t)file_stat.m_uncomp_size) != file_stat.m_crc32)
                return tmz_zip_set_error(pZip, tmz_ZIP_CRC_CHECK_FAILED);
        }
#endif

        return tmz_TRUE;
    }

    /* Decompress the file either directly from memory or from a file input buffer. */
    tinfl_init(&inflator);

    if (pZip->m_pState->m_pMem)
    {
        /* Read directly from the archive in memory. */
        pRead_buf = (tmz_uint8 *)pZip->m_pState->m_pMem + cur_file_ofs;
        read_buf_size = read_buf_avail = file_stat.m_comp_size;
        comp_remaining = 0;
    }
    else if (pUser_read_buf)
    {
        /* Use a user provided read buffer. */
        if (!user_read_buf_size)
            return tmz_FALSE;
        pRead_buf = (tmz_uint8 *)pUser_read_buf;
        read_buf_size = user_read_buf_size;
        read_buf_avail = 0;
        comp_remaining = file_stat.m_comp_size;
    }
    else
    {
        /* Temporarily allocate a read buffer. */
        read_buf_size = tmz_MIN(file_stat.m_comp_size, (tmz_uint64)tmz_ZIP_MAX_IO_BUF_SIZE);
        if (((sizeof(size_t) == sizeof(tmz_uint32))) && (read_buf_size > 0x7FFFFFFF))
            return tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);

        if (NULL == (pRead_buf = pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, (size_t)read_buf_size)))
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

        read_buf_avail = 0;
        comp_remaining = file_stat.m_comp_size;
    }

    do
    {
        /* The size_t cast here should be OK because we've verified that the output buffer is >= file_stat.m_uncomp_size above */
        size_t in_buf_size, out_buf_size = (size_t)(file_stat.m_uncomp_size - out_buf_ofs);
        if ((!read_buf_avail) && (!pZip->m_pState->m_pMem))
        {
            read_buf_avail = tmz_MIN(read_buf_size, comp_remaining);
            if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs, pRead_buf, (size_t)read_buf_avail) != read_buf_avail)
            {
                status = TINFL_STATUS_FAILED;
                tmz_zip_set_error(pZip, tmz_ZIP_DECOMPRESSION_FAILED);
                break;
            }
            cur_file_ofs += read_buf_avail;
            comp_remaining -= read_buf_avail;
            read_buf_ofs = 0;
        }
        in_buf_size = (size_t)read_buf_avail;
        status = tinfl_decompress(&inflator, (tmz_uint8 *)pRead_buf + read_buf_ofs, &in_buf_size, (tmz_uint8 *)pBuf, (tmz_uint8 *)pBuf + out_buf_ofs, &out_buf_size, TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF | (comp_remaining ? TINFL_FLAG_HAS_MORE_INPUT : 0));
        read_buf_avail -= in_buf_size;
        read_buf_ofs += in_buf_size;
        out_buf_ofs += out_buf_size;
    } while (status == TINFL_STATUS_NEEDS_MORE_INPUT);

    if (status == TINFL_STATUS_DONE)
    {
        /* Make sure the entire file was decompressed, and check its CRC. */
        if (out_buf_ofs != file_stat.m_uncomp_size)
        {
            tmz_zip_set_error(pZip, tmz_ZIP_UNEXPECTED_DECOMPRESSED_SIZE);
            status = TINFL_STATUS_FAILED;
        }
#ifndef tminiz_DISABLE_ZIP_READER_CRC32_CHECKS
        else if (tmz_crc32(tmz_CRC32_INIT, (const tmz_uint8 *)pBuf, (size_t)file_stat.m_uncomp_size) != file_stat.m_crc32)
        {
            tmz_zip_set_error(pZip, tmz_ZIP_CRC_CHECK_FAILED);
            status = TINFL_STATUS_FAILED;
        }
#endif
    }

    if ((!pZip->m_pState->m_pMem) && (!pUser_read_buf))
        pZip->m_pFree(pZip->m_pAlloc_opaque, pRead_buf);

    return status == TINFL_STATUS_DONE;
}

tmz_bool tmz_zip_reader_extract_file_to_mem_no_alloc(tmz_zip_archive *pZip, const char *pFilename, void *pBuf, size_t buf_size, tmz_uint flags, void *pUser_read_buf, size_t user_read_buf_size)
{
    tmz_uint32 file_index;
    if (!tmz_zip_reader_locate_file_v2(pZip, pFilename, NULL, flags, &file_index))
        return tmz_FALSE;
    return tmz_zip_reader_extract_to_mem_no_alloc(pZip, file_index, pBuf, buf_size, flags, pUser_read_buf, user_read_buf_size);
}

tmz_bool tmz_zip_reader_extract_to_mem(tmz_zip_archive *pZip, tmz_uint file_index, void *pBuf, size_t buf_size, tmz_uint flags)
{
    return tmz_zip_reader_extract_to_mem_no_alloc(pZip, file_index, pBuf, buf_size, flags, NULL, 0);
}

tmz_bool tmz_zip_reader_extract_file_to_mem(tmz_zip_archive *pZip, const char *pFilename, void *pBuf, size_t buf_size, tmz_uint flags)
{
    return tmz_zip_reader_extract_file_to_mem_no_alloc(pZip, pFilename, pBuf, buf_size, flags, NULL, 0);
}

void *tmz_zip_reader_extract_to_heap(tmz_zip_archive *pZip, tmz_uint file_index, size_t *pSize, tmz_uint flags)
{
    tmz_uint64 comp_size, uncomp_size, alloc_size;
    const tmz_uint8 *p = tmz_zip_get_cdh(pZip, file_index);
    void *pBuf;

    if (pSize)
        *pSize = 0;

    if (!p)
    {
        tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
        return NULL;
    }

    comp_size = tmz_READ_LE32(p + tmz_ZIP_CDH_COMPRESSED_SIZE_OFS);
    uncomp_size = tmz_READ_LE32(p + tmz_ZIP_CDH_DECOMPRESSED_SIZE_OFS);

    alloc_size = (flags & tmz_ZIP_FLAG_COMPRESSED_DATA) ? comp_size : uncomp_size;
    if (((sizeof(size_t) == sizeof(tmz_uint32))) && (alloc_size > 0x7FFFFFFF))
    {
        tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);
        return NULL;
    }

    if (NULL == (pBuf = pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, (size_t)alloc_size)))
    {
        tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        return NULL;
    }

    if (!tmz_zip_reader_extract_to_mem(pZip, file_index, pBuf, (size_t)alloc_size, flags))
    {
        pZip->m_pFree(pZip->m_pAlloc_opaque, pBuf);
        return NULL;
    }

    if (pSize)
        *pSize = (size_t)alloc_size;
    return pBuf;
}

void *tmz_zip_reader_extract_file_to_heap(tmz_zip_archive *pZip, const char *pFilename, size_t *pSize, tmz_uint flags)
{
    tmz_uint32 file_index;
    if (!tmz_zip_reader_locate_file_v2(pZip, pFilename, NULL, flags, &file_index))
    {
        if (pSize)
            *pSize = 0;
        return tmz_FALSE;
    }
    return tmz_zip_reader_extract_to_heap(pZip, file_index, pSize, flags);
}

tmz_bool tmz_zip_reader_extract_to_callback(tmz_zip_archive *pZip, tmz_uint file_index, tmz_file_write_func pCallback, void *pOpaque, tmz_uint flags)
{
    int status = TINFL_STATUS_DONE;
    tmz_uint file_crc32 = tmz_CRC32_INIT;
    tmz_uint64 read_buf_size, read_buf_ofs = 0, read_buf_avail, comp_remaining, out_buf_ofs = 0, cur_file_ofs;
    tmz_zip_archive_file_stat file_stat;
    void *pRead_buf = NULL;
    void *pWrite_buf = NULL;
    tmz_uint32 local_header_u32[(tmz_ZIP_LOCAL_DIR_HEADER_SIZE + sizeof(tmz_uint32) - 1) / sizeof(tmz_uint32)];
    tmz_uint8 *pLocal_header = (tmz_uint8 *)local_header_u32;

    if ((!pZip) || (!pZip->m_pState) || (!pCallback) || (!pZip->m_pRead))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (!tmz_zip_reader_file_stat(pZip, file_index, &file_stat))
        return tmz_FALSE;

    /* A directory or zero length file */
    if ((file_stat.m_is_directory) || (!file_stat.m_comp_size))
        return tmz_TRUE;

    /* Encryption and patch files are not supported. */
    if (file_stat.m_bit_flag & (tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_IS_ENCRYPTED | tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_USES_STRONG_ENCRYPTION | tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_COMPRESSED_PATCH_FLAG))
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_ENCRYPTION);

    /* This function only supports decompressing stored and deflate. */
    if ((!(flags & tmz_ZIP_FLAG_COMPRESSED_DATA)) && (file_stat.m_method != 0) && (file_stat.m_method != tmz_DEFLATED))
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_METHOD);

    /* Read and do some minimal validation of the local directory entry (this doesn't crack the zip64 stuff, which we already have from the central dir) */
    cur_file_ofs = file_stat.m_local_header_ofs;
    if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs, pLocal_header, tmz_ZIP_LOCAL_DIR_HEADER_SIZE) != tmz_ZIP_LOCAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);

    if (tmz_READ_LE32(pLocal_header) != tmz_ZIP_LOCAL_DIR_HEADER_SIG)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    cur_file_ofs += tmz_ZIP_LOCAL_DIR_HEADER_SIZE + tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_FILENAME_LEN_OFS) + tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_EXTRA_LEN_OFS);
    if ((cur_file_ofs + file_stat.m_comp_size) > pZip->m_archive_size)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    /* Decompress the file either directly from memory or from a file input buffer. */
    if (pZip->m_pState->m_pMem)
    {
        pRead_buf = (tmz_uint8 *)pZip->m_pState->m_pMem + cur_file_ofs;
        read_buf_size = read_buf_avail = file_stat.m_comp_size;
        comp_remaining = 0;
    }
    else
    {
        read_buf_size = tmz_MIN(file_stat.m_comp_size, (tmz_uint64)tmz_ZIP_MAX_IO_BUF_SIZE);
        if (NULL == (pRead_buf = pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, (size_t)read_buf_size)))
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

        read_buf_avail = 0;
        comp_remaining = file_stat.m_comp_size;
    }

    if ((flags & tmz_ZIP_FLAG_COMPRESSED_DATA) || (!file_stat.m_method))
    {
        /* The file is stored or the caller has requested the compressed data. */
        if (pZip->m_pState->m_pMem)
        {
            if (((sizeof(size_t) == sizeof(tmz_uint32))) && (file_stat.m_comp_size > tmz_UINT32_MAX))
                return tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);

            if (pCallback(pOpaque, out_buf_ofs, pRead_buf, (size_t)file_stat.m_comp_size) != file_stat.m_comp_size)
            {
                tmz_zip_set_error(pZip, tmz_ZIP_WRITE_CALLBACK_FAILED);
                status = TINFL_STATUS_FAILED;
            }
            else if (!(flags & tmz_ZIP_FLAG_COMPRESSED_DATA))
            {
#ifndef tminiz_DISABLE_ZIP_READER_CRC32_CHECKS
                file_crc32 = (tmz_uint32)tmz_crc32(file_crc32, (const tmz_uint8 *)pRead_buf, (size_t)file_stat.m_comp_size);
#endif
            }

            cur_file_ofs += file_stat.m_comp_size;
            out_buf_ofs += file_stat.m_comp_size;
            comp_remaining = 0;
        }
        else
        {
            while (comp_remaining)
            {
                read_buf_avail = tmz_MIN(read_buf_size, comp_remaining);
                if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs, pRead_buf, (size_t)read_buf_avail) != read_buf_avail)
                {
                    tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
                    status = TINFL_STATUS_FAILED;
                    break;
                }

#ifndef tminiz_DISABLE_ZIP_READER_CRC32_CHECKS
                if (!(flags & tmz_ZIP_FLAG_COMPRESSED_DATA))
                {
                    file_crc32 = (tmz_uint32)tmz_crc32(file_crc32, (const tmz_uint8 *)pRead_buf, (size_t)read_buf_avail);
                }
#endif

                if (pCallback(pOpaque, out_buf_ofs, pRead_buf, (size_t)read_buf_avail) != read_buf_avail)
                {
                    tmz_zip_set_error(pZip, tmz_ZIP_WRITE_CALLBACK_FAILED);
                    status = TINFL_STATUS_FAILED;
                    break;
                }

                cur_file_ofs += read_buf_avail;
                out_buf_ofs += read_buf_avail;
                comp_remaining -= read_buf_avail;
            }
        }
    }
    else
    {
        tinfl_decompressor inflator;
        tinfl_init(&inflator);

        if (NULL == (pWrite_buf = pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, TINFL_LZ_DICT_SIZE)))
        {
            tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
            status = TINFL_STATUS_FAILED;
        }
        else
        {
            do
            {
                tmz_uint8 *pWrite_buf_cur = (tmz_uint8 *)pWrite_buf + (out_buf_ofs & (TINFL_LZ_DICT_SIZE - 1));
                size_t in_buf_size, out_buf_size = TINFL_LZ_DICT_SIZE - (out_buf_ofs & (TINFL_LZ_DICT_SIZE - 1));
                if ((!read_buf_avail) && (!pZip->m_pState->m_pMem))
                {
                    read_buf_avail = tmz_MIN(read_buf_size, comp_remaining);
                    if (pZip->m_pRead(pZip->m_pIO_opaque, cur_file_ofs, pRead_buf, (size_t)read_buf_avail) != read_buf_avail)
                    {
                        tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
                        status = TINFL_STATUS_FAILED;
                        break;
                    }
                    cur_file_ofs += read_buf_avail;
                    comp_remaining -= read_buf_avail;
                    read_buf_ofs = 0;
                }

                in_buf_size = (size_t)read_buf_avail;
                status = tinfl_decompress(&inflator, (const tmz_uint8 *)pRead_buf + read_buf_ofs, &in_buf_size, (tmz_uint8 *)pWrite_buf, pWrite_buf_cur, &out_buf_size, comp_remaining ? TINFL_FLAG_HAS_MORE_INPUT : 0);
                read_buf_avail -= in_buf_size;
                read_buf_ofs += in_buf_size;

                if (out_buf_size)
                {
                    if (pCallback(pOpaque, out_buf_ofs, pWrite_buf_cur, out_buf_size) != out_buf_size)
                    {
                        tmz_zip_set_error(pZip, tmz_ZIP_WRITE_CALLBACK_FAILED);
                        status = TINFL_STATUS_FAILED;
                        break;
                    }

#ifndef tminiz_DISABLE_ZIP_READER_CRC32_CHECKS
                    file_crc32 = (tmz_uint32)tmz_crc32(file_crc32, pWrite_buf_cur, out_buf_size);
#endif
                    if ((out_buf_ofs += out_buf_size) > file_stat.m_uncomp_size)
                    {
                        tmz_zip_set_error(pZip, tmz_ZIP_DECOMPRESSION_FAILED);
                        status = TINFL_STATUS_FAILED;
                        break;
                    }
                }
            } while ((status == TINFL_STATUS_NEEDS_MORE_INPUT) || (status == TINFL_STATUS_HAS_MORE_OUTPUT));
        }
    }

    if ((status == TINFL_STATUS_DONE) && (!(flags & tmz_ZIP_FLAG_COMPRESSED_DATA)))
    {
        /* Make sure the entire file was decompressed, and check its CRC. */
        if (out_buf_ofs != file_stat.m_uncomp_size)
        {
            tmz_zip_set_error(pZip, tmz_ZIP_UNEXPECTED_DECOMPRESSED_SIZE);
            status = TINFL_STATUS_FAILED;
        }
#ifndef tminiz_DISABLE_ZIP_READER_CRC32_CHECKS
        else if (file_crc32 != file_stat.m_crc32)
        {
            tmz_zip_set_error(pZip, tmz_ZIP_DECOMPRESSION_FAILED);
            status = TINFL_STATUS_FAILED;
        }
#endif
    }

    if (!pZip->m_pState->m_pMem)
        pZip->m_pFree(pZip->m_pAlloc_opaque, pRead_buf);

    if (pWrite_buf)
        pZip->m_pFree(pZip->m_pAlloc_opaque, pWrite_buf);

    return status == TINFL_STATUS_DONE;
}

tmz_bool tmz_zip_reader_extract_file_to_callback(tmz_zip_archive *pZip, const char *pFilename, tmz_file_write_func pCallback, void *pOpaque, tmz_uint flags)
{
    tmz_uint32 file_index;
    if (!tmz_zip_reader_locate_file_v2(pZip, pFilename, NULL, flags, &file_index))
        return tmz_FALSE;

    return tmz_zip_reader_extract_to_callback(pZip, file_index, pCallback, pOpaque, flags);
}

#ifndef tminiz_NO_STDIO
static size_t tmz_zip_file_write_callback(void *pOpaque, tmz_uint64 ofs, const void *pBuf, size_t n)
{
    (void)ofs;

    return tmz_FWRITE(pBuf, 1, n, (tmz_FILE *)pOpaque);
}

tmz_bool tmz_zip_reader_extract_to_file(tmz_zip_archive *pZip, tmz_uint file_index, const char *pDst_filename, tmz_uint flags)
{
    tmz_bool status;
    tmz_zip_archive_file_stat file_stat;
    tmz_FILE *pFile;

    if (!tmz_zip_reader_file_stat(pZip, file_index, &file_stat))
        return tmz_FALSE;

    if ((file_stat.m_is_directory) || (!file_stat.m_is_supported))
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_FEATURE);

    pFile = tmz_FOPEN(pDst_filename, "wb");
    if (!pFile)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_OPEN_FAILED);

    status = tmz_zip_reader_extract_to_callback(pZip, file_index, tmz_zip_file_write_callback, pFile, flags);

    if (tmz_FCLOSE(pFile) == EOF)
    {
        if (status)
            tmz_zip_set_error(pZip, tmz_ZIP_FILE_CLOSE_FAILED);

        status = tmz_FALSE;
    }

#if !defined(tminiz_NO_TIME) && !defined(tminiz_NO_STDIO)
    if (status)
        tmz_zip_set_file_times(pDst_filename, file_stat.m_time, file_stat.m_time);
#endif

    return status;
}

tmz_bool tmz_zip_reader_extract_file_to_file(tmz_zip_archive *pZip, const char *pArchive_filename, const char *pDst_filename, tmz_uint flags)
{
    tmz_uint32 file_index;
    if (!tmz_zip_reader_locate_file_v2(pZip, pArchive_filename, NULL, flags, &file_index))
        return tmz_FALSE;

    return tmz_zip_reader_extract_to_file(pZip, file_index, pDst_filename, flags);
}

tmz_bool tmz_zip_reader_extract_to_cfile(tmz_zip_archive *pZip, tmz_uint file_index, tmz_FILE *pFile, tmz_uint flags)
{
    tmz_zip_archive_file_stat file_stat;

    if (!tmz_zip_reader_file_stat(pZip, file_index, &file_stat))
        return tmz_FALSE;

    if ((file_stat.m_is_directory) || (!file_stat.m_is_supported))
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_FEATURE);

    return tmz_zip_reader_extract_to_callback(pZip, file_index, tmz_zip_file_write_callback, pFile, flags);
}

tmz_bool tmz_zip_reader_extract_file_to_cfile(tmz_zip_archive *pZip, const char *pArchive_filename, tmz_FILE *pFile, tmz_uint flags)
{
    tmz_uint32 file_index;
    if (!tmz_zip_reader_locate_file_v2(pZip, pArchive_filename, NULL, flags, &file_index))
        return tmz_FALSE;

    return tmz_zip_reader_extract_to_cfile(pZip, file_index, pFile, flags);
}
#endif /* #ifndef tminiz_NO_STDIO */

static size_t tmz_zip_compute_crc32_callback(void *pOpaque, tmz_uint64 file_ofs, const void *pBuf, size_t n)
{
    tmz_uint32 *p = (tmz_uint32 *)pOpaque;
    (void)file_ofs;
    *p = (tmz_uint32)tmz_crc32(*p, (const tmz_uint8 *)pBuf, n);
    return n;
}

tmz_bool tmz_zip_validate_file(tmz_zip_archive *pZip, tmz_uint file_index, tmz_uint flags)
{
    tmz_zip_archive_file_stat file_stat;
    tmz_zip_internal_state *pState;
    const tmz_uint8 *pCentral_dir_header;
    tmz_bool found_zip64_ext_data_in_cdir = tmz_FALSE;
    tmz_bool found_zip64_ext_data_in_ldir = tmz_FALSE;
    tmz_uint32 local_header_u32[(tmz_ZIP_LOCAL_DIR_HEADER_SIZE + sizeof(tmz_uint32) - 1) / sizeof(tmz_uint32)];
    tmz_uint8 *pLocal_header = (tmz_uint8 *)local_header_u32;
    tmz_uint64 local_header_ofs = 0;
    tmz_uint32 local_header_filename_len, local_header_extra_len, local_header_crc32;
    tmz_uint64 local_header_comp_size, local_header_uncomp_size;
    tmz_uint32 uncomp_crc32 = tmz_CRC32_INIT;
    tmz_bool has_data_descriptor;
    tmz_uint32 local_header_bit_flags;

    tmz_zip_array file_data_array;
    tmz_zip_array_init(&file_data_array, 1);

    if ((!pZip) || (!pZip->m_pState) || (!pZip->m_pAlloc) || (!pZip->m_pFree) || (!pZip->m_pRead))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (file_index > pZip->m_total_files)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    pState = pZip->m_pState;

    pCentral_dir_header = tmz_zip_get_cdh(pZip, file_index);

    if (!tmz_zip_file_stat_internal(pZip, file_index, pCentral_dir_header, &file_stat, &found_zip64_ext_data_in_cdir))
        return tmz_FALSE;

    /* A directory or zero length file */
    if ((file_stat.m_is_directory) || (!file_stat.m_uncomp_size))
        return tmz_TRUE;

    /* Encryption and patch files are not supported. */
    if (file_stat.m_is_encrypted)
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_ENCRYPTION);

    /* This function only supports stored and deflate. */
    if ((file_stat.m_method != 0) && (file_stat.m_method != tmz_DEFLATED))
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_METHOD);

    if (!file_stat.m_is_supported)
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_FEATURE);

    /* Read and parse the local directory entry. */
    local_header_ofs = file_stat.m_local_header_ofs;
    if (pZip->m_pRead(pZip->m_pIO_opaque, local_header_ofs, pLocal_header, tmz_ZIP_LOCAL_DIR_HEADER_SIZE) != tmz_ZIP_LOCAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);

    if (tmz_READ_LE32(pLocal_header) != tmz_ZIP_LOCAL_DIR_HEADER_SIG)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    local_header_filename_len = tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_FILENAME_LEN_OFS);
    local_header_extra_len = tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_EXTRA_LEN_OFS);
    local_header_comp_size = tmz_READ_LE32(pLocal_header + tmz_ZIP_LDH_COMPRESSED_SIZE_OFS);
    local_header_uncomp_size = tmz_READ_LE32(pLocal_header + tmz_ZIP_LDH_DECOMPRESSED_SIZE_OFS);
    local_header_crc32 = tmz_READ_LE32(pLocal_header + tmz_ZIP_LDH_CRC32_OFS);
    local_header_bit_flags = tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_BIT_FLAG_OFS);
    has_data_descriptor = (local_header_bit_flags & 8) != 0;

    if (local_header_filename_len != strlen(file_stat.m_filename))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    if ((local_header_ofs + tmz_ZIP_LOCAL_DIR_HEADER_SIZE + local_header_filename_len + local_header_extra_len + file_stat.m_comp_size) > pZip->m_archive_size)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    if (!tmz_zip_array_resize(pZip, &file_data_array, tmz_MAX(local_header_filename_len, local_header_extra_len), tmz_FALSE))
        return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

    if (local_header_filename_len)
    {
        if (pZip->m_pRead(pZip->m_pIO_opaque, local_header_ofs + tmz_ZIP_LOCAL_DIR_HEADER_SIZE, file_data_array.m_p, local_header_filename_len) != local_header_filename_len)
        {
            tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
            goto handle_failure;
        }

        /* I've seen 1 archive that had the same pathname, but used backslashes in the local dir and forward slashes in the central dir. Do we care about this? For now, this case will fail validation. */
        if (memcmp(file_stat.m_filename, file_data_array.m_p, local_header_filename_len) != 0)
        {
            tmz_zip_set_error(pZip, tmz_ZIP_VALIDATION_FAILED);
            goto handle_failure;
        }
    }

    if ((local_header_extra_len) && ((local_header_comp_size == tmz_UINT32_MAX) || (local_header_uncomp_size == tmz_UINT32_MAX)))
    {
		tmz_uint32 extra_size_remaining = local_header_extra_len;
		const tmz_uint8 *pExtra_data = (const tmz_uint8 *)file_data_array.m_p;

        if (pZip->m_pRead(pZip->m_pIO_opaque, local_header_ofs + tmz_ZIP_LOCAL_DIR_HEADER_SIZE + local_header_filename_len, file_data_array.m_p, local_header_extra_len) != local_header_extra_len)
        {
            tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
            goto handle_failure;
        }

        do
        {
            tmz_uint32 field_id, field_data_size, field_total_size;

            if (extra_size_remaining < (sizeof(tmz_uint16) * 2))
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

            field_id = tmz_READ_LE16(pExtra_data);
            field_data_size = tmz_READ_LE16(pExtra_data + sizeof(tmz_uint16));
            field_total_size = field_data_size + sizeof(tmz_uint16) * 2;

            if (field_total_size > extra_size_remaining)
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

            if (field_id == tmz_ZIP64_EXTENDED_INFORMATION_FIELD_HEADER_ID)
            {
                const tmz_uint8 *pSrc_field_data = pExtra_data + sizeof(tmz_uint32);

                if (field_data_size < sizeof(tmz_uint64) * 2)
                {
                    tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);
                    goto handle_failure;
                }

                local_header_uncomp_size = tmz_READ_LE64(pSrc_field_data);
                local_header_comp_size = tmz_READ_LE64(pSrc_field_data + sizeof(tmz_uint64));

                found_zip64_ext_data_in_ldir = tmz_TRUE;
                break;
            }

            pExtra_data += field_total_size;
            extra_size_remaining -= field_total_size;
        } while (extra_size_remaining);
    }

    /* TODO: parse local header extra data when local_header_comp_size is 0xFFFFFFFF! (big_descriptor.zip) */
    /* I've seen zips in the wild with the data descriptor bit set, but proper local header values and bogus data descriptors */
    if ((has_data_descriptor) && (!local_header_comp_size) && (!local_header_crc32))
    {
        tmz_uint8 descriptor_buf[32];
		tmz_bool has_id;
		const tmz_uint8 *pSrc;
		tmz_uint32 file_crc32;
		tmz_uint64 comp_size = 0, uncomp_size = 0;

        tmz_uint32 num_descriptor_uint32s = ((pState->m_zip64) || (found_zip64_ext_data_in_ldir)) ? 6 : 4;

        if (pZip->m_pRead(pZip->m_pIO_opaque, local_header_ofs + tmz_ZIP_LOCAL_DIR_HEADER_SIZE + local_header_filename_len + local_header_extra_len + file_stat.m_comp_size, descriptor_buf, sizeof(tmz_uint32) * num_descriptor_uint32s) != (sizeof(tmz_uint32) * num_descriptor_uint32s))
        {
            tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
            goto handle_failure;
        }

        has_id = (tmz_READ_LE32(descriptor_buf) == tmz_ZIP_DATA_DESCRIPTOR_ID);
		pSrc = has_id ? (descriptor_buf + sizeof(tmz_uint32)) : descriptor_buf;

        file_crc32 = tmz_READ_LE32(pSrc);        

        if ((pState->m_zip64) || (found_zip64_ext_data_in_ldir))
        {
            comp_size = tmz_READ_LE64(pSrc + sizeof(tmz_uint32));
            uncomp_size = tmz_READ_LE64(pSrc + sizeof(tmz_uint32) + sizeof(tmz_uint64));
        }
        else
        {
            comp_size = tmz_READ_LE32(pSrc + sizeof(tmz_uint32));
            uncomp_size = tmz_READ_LE32(pSrc + sizeof(tmz_uint32) + sizeof(tmz_uint32));
        }

        if ((file_crc32 != file_stat.m_crc32) || (comp_size != file_stat.m_comp_size) || (uncomp_size != file_stat.m_uncomp_size))
        {
            tmz_zip_set_error(pZip, tmz_ZIP_VALIDATION_FAILED);
            goto handle_failure;
        }
    }
    else
    {
        if ((local_header_crc32 != file_stat.m_crc32) || (local_header_comp_size != file_stat.m_comp_size) || (local_header_uncomp_size != file_stat.m_uncomp_size))
        {
            tmz_zip_set_error(pZip, tmz_ZIP_VALIDATION_FAILED);
            goto handle_failure;
        }
    }

    tmz_zip_array_clear(pZip, &file_data_array);

    if ((flags & tmz_ZIP_FLAG_VALIDATE_HEADERS_ONLY) == 0)
    {
        if (!tmz_zip_reader_extract_to_callback(pZip, file_index, tmz_zip_compute_crc32_callback, &uncomp_crc32, 0))
            return tmz_FALSE;

        /* 1 more check to be sure, although the extract checks too. */
        if (uncomp_crc32 != file_stat.m_crc32)
        {
            tmz_zip_set_error(pZip, tmz_ZIP_VALIDATION_FAILED);
            return tmz_FALSE;
        }
    }

    return tmz_TRUE;

handle_failure:
    tmz_zip_array_clear(pZip, &file_data_array);
    return tmz_FALSE;
}

tmz_bool tmz_zip_validate_archive(tmz_zip_archive *pZip, tmz_uint flags)
{
    tmz_zip_internal_state *pState;
    uint32_t i;

    if ((!pZip) || (!pZip->m_pState) || (!pZip->m_pAlloc) || (!pZip->m_pFree) || (!pZip->m_pRead))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    pState = pZip->m_pState;

    /* Basic sanity checks */
    if (!pState->m_zip64)
    {
        if (pZip->m_total_files > tmz_UINT16_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);

        if (pZip->m_archive_size > tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);
    }
    else
    {
        if (pZip->m_total_files >= tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);

        if (pState->m_central_dir.m_size >= tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);
    }

    for (i = 0; i < pZip->m_total_files; i++)
    {
        if (tmz_ZIP_FLAG_VALIDATE_LOCATE_FILE_FLAG & flags)
        {
            tmz_uint32 found_index;
            tmz_zip_archive_file_stat stat;

            if (!tmz_zip_reader_file_stat(pZip, i, &stat))
                return tmz_FALSE;

            if (!tmz_zip_reader_locate_file_v2(pZip, stat.m_filename, NULL, 0, &found_index))
                return tmz_FALSE;

            /* This check can fail if there are duplicate filenames in the archive (which we don't check for when writing - that's up to the user) */
            if (found_index != i)
                return tmz_zip_set_error(pZip, tmz_ZIP_VALIDATION_FAILED);
        }

        if (!tmz_zip_validate_file(pZip, i, flags))
            return tmz_FALSE;
    }

    return tmz_TRUE;
}

tmz_bool tmz_zip_validate_mem_archive(const void *pMem, size_t size, tmz_uint flags, tmz_zip_error *pErr)
{
    tmz_bool success = tmz_TRUE;
    tmz_zip_archive zip;
    tmz_zip_error actual_err = tmz_ZIP_NO_ERROR;

    if ((!pMem) || (!size))
    {
        if (pErr)
            *pErr = tmz_ZIP_INVALID_PARAMETER;
        return tmz_FALSE;
    }

    tmz_zip_zero_struct(&zip);

    if (!tmz_zip_reader_init_mem(&zip, pMem, size, flags))
    {
        if (pErr)
            *pErr = zip.m_last_error;
        return tmz_FALSE;
    }

    if (!tmz_zip_validate_archive(&zip, flags))
    {
        actual_err = zip.m_last_error;
        success = tmz_FALSE;
    }

    if (!tmz_zip_reader_end_internal(&zip, success))
    {
        if (!actual_err)
            actual_err = zip.m_last_error;
        success = tmz_FALSE;
    }

    if (pErr)
        *pErr = actual_err;

    return success;
}

#ifndef tminiz_NO_STDIO
tmz_bool tmz_zip_validate_file_archive(const char *pFilename, tmz_uint flags, tmz_zip_error *pErr)
{
    tmz_bool success = tmz_TRUE;
    tmz_zip_archive zip;
    tmz_zip_error actual_err = tmz_ZIP_NO_ERROR;

    if (!pFilename)
    {
        if (pErr)
            *pErr = tmz_ZIP_INVALID_PARAMETER;
        return tmz_FALSE;
    }

    tmz_zip_zero_struct(&zip);

    if (!tmz_zip_reader_init_file_v2(&zip, pFilename, flags, 0, 0))
    {
        if (pErr)
            *pErr = zip.m_last_error;
        return tmz_FALSE;
    }

    if (!tmz_zip_validate_archive(&zip, flags))
    {
        actual_err = zip.m_last_error;
        success = tmz_FALSE;
    }

    if (!tmz_zip_reader_end_internal(&zip, success))
    {
        if (!actual_err)
            actual_err = zip.m_last_error;
        success = tmz_FALSE;
    }

    if (pErr)
        *pErr = actual_err;

    return success;
}
#endif /* #ifndef tminiz_NO_STDIO */

/* ------------------- .ZIP archive writing */

#ifndef tminiz_NO_ARCHIVE_WRITING_APIS

static tmz_FORCEINLINE void tmz_write_le16(tmz_uint8 *p, tmz_uint16 v)
{
    p[0] = (tmz_uint8)v;
    p[1] = (tmz_uint8)(v >> 8);
}
static tmz_FORCEINLINE void tmz_write_le32(tmz_uint8 *p, tmz_uint32 v)
{
    p[0] = (tmz_uint8)v;
    p[1] = (tmz_uint8)(v >> 8);
    p[2] = (tmz_uint8)(v >> 16);
    p[3] = (tmz_uint8)(v >> 24);
}
static tmz_FORCEINLINE void tmz_write_le64(tmz_uint8 *p, tmz_uint64 v)
{
    tmz_write_le32(p, (tmz_uint32)v);
    tmz_write_le32(p + sizeof(tmz_uint32), (tmz_uint32)(v >> 32));
}

#define tmz_WRITE_LE16(p, v) tmz_write_le16((tmz_uint8 *)(p), (tmz_uint16)(v))
#define tmz_WRITE_LE32(p, v) tmz_write_le32((tmz_uint8 *)(p), (tmz_uint32)(v))
#define tmz_WRITE_LE64(p, v) tmz_write_le64((tmz_uint8 *)(p), (tmz_uint64)(v))

static size_t tmz_zip_heap_write_func(void *pOpaque, tmz_uint64 file_ofs, const void *pBuf, size_t n)
{
    tmz_zip_archive *pZip = (tmz_zip_archive *)pOpaque;
    tmz_zip_internal_state *pState = pZip->m_pState;
    tmz_uint64 new_size = tmz_MAX(file_ofs + n, pState->m_mem_size);

    if (!n)
        return 0;

    /* An allocation this big is likely to just fail on 32-bit systems, so don't even go there. */
    if ((sizeof(size_t) == sizeof(tmz_uint32)) && (new_size > 0x7FFFFFFF))
    {
        tmz_zip_set_error(pZip, tmz_ZIP_FILE_TOO_LARGE);
        return 0;
    }

    if (new_size > pState->m_mem_capacity)
    {
        void *pNew_block;
        size_t new_capacity = tmz_MAX(64, pState->m_mem_capacity);

        while (new_capacity < new_size)
            new_capacity *= 2;

        if (NULL == (pNew_block = pZip->m_pRealloc(pZip->m_pAlloc_opaque, pState->m_pMem, 1, new_capacity)))
        {
            tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
            return 0;
        }

        pState->m_pMem = pNew_block;
        pState->m_mem_capacity = new_capacity;
    }
    memcpy((tmz_uint8 *)pState->m_pMem + file_ofs, pBuf, n);
    pState->m_mem_size = (size_t)new_size;
    return n;
}

static tmz_bool tmz_zip_writer_end_internal(tmz_zip_archive *pZip, tmz_bool set_last_error)
{
    tmz_zip_internal_state *pState;
    tmz_bool status = tmz_TRUE;

    if ((!pZip) || (!pZip->m_pState) || (!pZip->m_pAlloc) || (!pZip->m_pFree) || ((pZip->m_zip_mode != tmz_ZIP_MODE_WRITING) && (pZip->m_zip_mode != tmz_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED)))
    {
        if (set_last_error)
            tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
        return tmz_FALSE;
    }

    pState = pZip->m_pState;
    pZip->m_pState = NULL;
    tmz_zip_array_clear(pZip, &pState->m_central_dir);
    tmz_zip_array_clear(pZip, &pState->m_central_dir_offsets);
    tmz_zip_array_clear(pZip, &pState->m_sorted_central_dir_offsets);

#ifndef tminiz_NO_STDIO
    if (pState->m_pFile)
    {
        if (pZip->m_zip_type == tmz_ZIP_TYPE_FILE)
        {
            if (tmz_FCLOSE(pState->m_pFile) == EOF)
            {
                if (set_last_error)
                    tmz_zip_set_error(pZip, tmz_ZIP_FILE_CLOSE_FAILED);
                status = tmz_FALSE;
            }
        }

        pState->m_pFile = NULL;
    }
#endif /* #ifndef tminiz_NO_STDIO */

    if ((pZip->m_pWrite == tmz_zip_heap_write_func) && (pState->m_pMem))
    {
        pZip->m_pFree(pZip->m_pAlloc_opaque, pState->m_pMem);
        pState->m_pMem = NULL;
    }

    pZip->m_pFree(pZip->m_pAlloc_opaque, pState);
    pZip->m_zip_mode = tmz_ZIP_MODE_INVALID;
    return status;
}

tmz_bool tmz_zip_writer_init_v2(tmz_zip_archive *pZip, tmz_uint64 existing_size, tmz_uint flags)
{
    tmz_bool zip64 = (flags & tmz_ZIP_FLAG_WRITE_ZIP64) != 0;

    if ((!pZip) || (pZip->m_pState) || (!pZip->m_pWrite) || (pZip->m_zip_mode != tmz_ZIP_MODE_INVALID))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (flags & tmz_ZIP_FLAG_WRITE_ALLOW_READING)
    {
        if (!pZip->m_pRead)
            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
    }

    if (pZip->m_file_offset_alignment)
    {
        /* Ensure user specified file offset alignment is a power of 2. */
        if (pZip->m_file_offset_alignment & (pZip->m_file_offset_alignment - 1))
            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
    }

    if (!pZip->m_pAlloc)
        pZip->m_pAlloc = tminiz_def_alloc_func;
    if (!pZip->m_pFree)
        pZip->m_pFree = tminiz_def_free_func;
    if (!pZip->m_pRealloc)
        pZip->m_pRealloc = tminiz_def_realloc_func;

    pZip->m_archive_size = existing_size;
    pZip->m_central_directory_file_ofs = 0;
    pZip->m_total_files = 0;

    if (NULL == (pZip->m_pState = (tmz_zip_internal_state *)pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, sizeof(tmz_zip_internal_state))))
        return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

    memset(pZip->m_pState, 0, sizeof(tmz_zip_internal_state));

    tmz_ZIP_ARRAY_SET_ELEMENT_SIZE(&pZip->m_pState->m_central_dir, sizeof(tmz_uint8));
    tmz_ZIP_ARRAY_SET_ELEMENT_SIZE(&pZip->m_pState->m_central_dir_offsets, sizeof(tmz_uint32));
    tmz_ZIP_ARRAY_SET_ELEMENT_SIZE(&pZip->m_pState->m_sorted_central_dir_offsets, sizeof(tmz_uint32));

    pZip->m_pState->m_zip64 = zip64;
    pZip->m_pState->m_zip64_has_extended_info_fields = zip64;

    pZip->m_zip_type = tmz_ZIP_TYPE_USER;
    pZip->m_zip_mode = tmz_ZIP_MODE_WRITING;

    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_init(tmz_zip_archive *pZip, tmz_uint64 existing_size)
{
    return tmz_zip_writer_init_v2(pZip, existing_size, 0);
}

tmz_bool tmz_zip_writer_init_heap_v2(tmz_zip_archive *pZip, size_t size_to_reserve_at_beginning, size_t initial_allocation_size, tmz_uint flags)
{
    pZip->m_pWrite = tmz_zip_heap_write_func;
	pZip->m_pNeeds_keepalive = NULL;

    if (flags & tmz_ZIP_FLAG_WRITE_ALLOW_READING)
        pZip->m_pRead = tmz_zip_mem_read_func;

    pZip->m_pIO_opaque = pZip;

    if (!tmz_zip_writer_init_v2(pZip, size_to_reserve_at_beginning, flags))
        return tmz_FALSE;

    pZip->m_zip_type = tmz_ZIP_TYPE_HEAP;

    if (0 != (initial_allocation_size = tmz_MAX(initial_allocation_size, size_to_reserve_at_beginning)))
    {
        if (NULL == (pZip->m_pState->m_pMem = pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, initial_allocation_size)))
        {
            tmz_zip_writer_end_internal(pZip, tmz_FALSE);
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }
        pZip->m_pState->m_mem_capacity = initial_allocation_size;
    }

    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_init_heap(tmz_zip_archive *pZip, size_t size_to_reserve_at_beginning, size_t initial_allocation_size)
{
    return tmz_zip_writer_init_heap_v2(pZip, size_to_reserve_at_beginning, initial_allocation_size, 0);
}

#ifndef tminiz_NO_STDIO
static size_t tmz_zip_file_write_func(void *pOpaque, tmz_uint64 file_ofs, const void *pBuf, size_t n)
{
    tmz_zip_archive *pZip = (tmz_zip_archive *)pOpaque;
    tmz_int64 cur_ofs = tmz_FTELL64(pZip->m_pState->m_pFile);

    file_ofs += pZip->m_pState->m_file_archive_start_ofs;

    if (((tmz_int64)file_ofs < 0) || (((cur_ofs != (tmz_int64)file_ofs)) && (tmz_FSEEK64(pZip->m_pState->m_pFile, (tmz_int64)file_ofs, SEEK_SET))))
    {
        tmz_zip_set_error(pZip, tmz_ZIP_FILE_SEEK_FAILED);
        return 0;
    }

    return tmz_FWRITE(pBuf, 1, n, pZip->m_pState->m_pFile);
}

tmz_bool tmz_zip_writer_init_file(tmz_zip_archive *pZip, const char *pFilename, tmz_uint64 size_to_reserve_at_beginning)
{
    return tmz_zip_writer_init_file_v2(pZip, pFilename, size_to_reserve_at_beginning, 0);
}

tmz_bool tmz_zip_writer_init_file_v2(tmz_zip_archive *pZip, const char *pFilename, tmz_uint64 size_to_reserve_at_beginning, tmz_uint flags)
{
    tmz_FILE *pFile;

    pZip->m_pWrite = tmz_zip_file_write_func;
	pZip->m_pNeeds_keepalive = NULL;

    if (flags & tmz_ZIP_FLAG_WRITE_ALLOW_READING)
        pZip->m_pRead = tmz_zip_file_read_func;

    pZip->m_pIO_opaque = pZip;

    if (!tmz_zip_writer_init_v2(pZip, size_to_reserve_at_beginning, flags))
        return tmz_FALSE;

    if (NULL == (pFile = tmz_FOPEN(pFilename, (flags & tmz_ZIP_FLAG_WRITE_ALLOW_READING) ? "w+b" : "wb")))
    {
        tmz_zip_writer_end(pZip);
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_OPEN_FAILED);
    }

    pZip->m_pState->m_pFile = pFile;
    pZip->m_zip_type = tmz_ZIP_TYPE_FILE;

    if (size_to_reserve_at_beginning)
    {
        tmz_uint64 cur_ofs = 0;
        char buf[4096];

        tmz_CLEAR_OBJ(buf);

        do
        {
            size_t n = (size_t)tmz_MIN(sizeof(buf), size_to_reserve_at_beginning);
            if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_ofs, buf, n) != n)
            {
                tmz_zip_writer_end(pZip);
                return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
            }
            cur_ofs += n;
            size_to_reserve_at_beginning -= n;
        } while (size_to_reserve_at_beginning);
    }

    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_init_cfile(tmz_zip_archive *pZip, tmz_FILE *pFile, tmz_uint flags)
{
    pZip->m_pWrite = tmz_zip_file_write_func;
	pZip->m_pNeeds_keepalive = NULL;

    if (flags & tmz_ZIP_FLAG_WRITE_ALLOW_READING)
        pZip->m_pRead = tmz_zip_file_read_func;

    pZip->m_pIO_opaque = pZip;

    if (!tmz_zip_writer_init_v2(pZip, 0, flags))
        return tmz_FALSE;

    pZip->m_pState->m_pFile = pFile;
    pZip->m_pState->m_file_archive_start_ofs = tmz_FTELL64(pZip->m_pState->m_pFile);
    pZip->m_zip_type = tmz_ZIP_TYPE_CFILE;

    return tmz_TRUE;
}
#endif /* #ifndef tminiz_NO_STDIO */

tmz_bool tmz_zip_writer_init_from_reader_v2(tmz_zip_archive *pZip, const char *pFilename, tmz_uint flags)
{
    tmz_zip_internal_state *pState;

    if ((!pZip) || (!pZip->m_pState) || (pZip->m_zip_mode != tmz_ZIP_MODE_READING))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (flags & tmz_ZIP_FLAG_WRITE_ZIP64)
    {
        /* We don't support converting a non-zip64 file to zip64 - this seems like more trouble than it's worth. (What about the existing 32-bit data descriptors that could follow the compressed data?) */
        if (!pZip->m_pState->m_zip64)
            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
    }

    /* No sense in trying to write to an archive that's already at the support max size */
    if (pZip->m_pState->m_zip64)
    {
        if (pZip->m_total_files == tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);
    }
    else
    {
        if (pZip->m_total_files == tmz_UINT16_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);

        if ((pZip->m_archive_size + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + tmz_ZIP_LOCAL_DIR_HEADER_SIZE) > tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_TOO_LARGE);
    }

    pState = pZip->m_pState;

    if (pState->m_pFile)
    {
#ifdef tminiz_NO_STDIO
        (void)pFilename;
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
#else
        if (pZip->m_pIO_opaque != pZip)
            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

        if (pZip->m_zip_type == tmz_ZIP_TYPE_FILE)
        {
            if (!pFilename)
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

            /* Archive is being read from stdio and was originally opened only for reading. Try to reopen as writable. */
            if (NULL == (pState->m_pFile = tmz_FREOPEN(pFilename, "r+b", pState->m_pFile)))
            {
                /* The tmz_zip_archive is now in a bogus state because pState->m_pFile is NULL, so just close it. */
                tmz_zip_reader_end_internal(pZip, tmz_FALSE);
                return tmz_zip_set_error(pZip, tmz_ZIP_FILE_OPEN_FAILED);
            }
        }

        pZip->m_pWrite = tmz_zip_file_write_func;
		pZip->m_pNeeds_keepalive = NULL;
#endif /* #ifdef tminiz_NO_STDIO */
    }
    else if (pState->m_pMem)
    {
        /* Archive lives in a memory block. Assume it's from the heap that we can resize using the realloc callback. */
        if (pZip->m_pIO_opaque != pZip)
            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

        pState->m_mem_capacity = pState->m_mem_size;
        pZip->m_pWrite = tmz_zip_heap_write_func;
		pZip->m_pNeeds_keepalive = NULL;
    }
    /* Archive is being read via a user provided read function - make sure the user has specified a write function too. */
    else if (!pZip->m_pWrite)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    /* Start writing new files at the archive's current central directory location. */
    /* TODO: We could add a flag that lets the user start writing immediately AFTER the existing central dir - this would be safer. */
    pZip->m_archive_size = pZip->m_central_directory_file_ofs;
    pZip->m_central_directory_file_ofs = 0;

    /* Clear the sorted central dir offsets, they aren't useful or maintained now. */
    /* Even though we're now in write mode, files can still be extracted and verified, but file locates will be slow. */
    /* TODO: We could easily maintain the sorted central directory offsets. */
    tmz_zip_array_clear(pZip, &pZip->m_pState->m_sorted_central_dir_offsets);

    pZip->m_zip_mode = tmz_ZIP_MODE_WRITING;

    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_init_from_reader(tmz_zip_archive *pZip, const char *pFilename)
{
    return tmz_zip_writer_init_from_reader_v2(pZip, pFilename, 0);
}

/* TODO: pArchive_name is a terrible name here! */
tmz_bool tmz_zip_writer_add_mem(tmz_zip_archive *pZip, const char *pArchive_name, const void *pBuf, size_t buf_size, tmz_uint level_and_flags)
{
    return tmz_zip_writer_add_mem_ex(pZip, pArchive_name, pBuf, buf_size, NULL, 0, level_and_flags, 0, 0);
}

typedef struct
{
    tmz_zip_archive *m_pZip;
    tmz_uint64 m_cur_archive_file_ofs;
    tmz_uint64 m_comp_size;
} tmz_zip_writer_add_state;

static tmz_bool tmz_zip_writer_add_put_buf_callback(const void *pBuf, int len, void *pUser)
{
    tmz_zip_writer_add_state *pState = (tmz_zip_writer_add_state *)pUser;
    if ((int)pState->m_pZip->m_pWrite(pState->m_pZip->m_pIO_opaque, pState->m_cur_archive_file_ofs, pBuf, len) != len)
        return tmz_FALSE;

    pState->m_cur_archive_file_ofs += len;
    pState->m_comp_size += len;
    return tmz_TRUE;
}

#define tmz_ZIP64_MAX_LOCAL_EXTRA_FIELD_SIZE (sizeof(tmz_uint16) * 2 + sizeof(tmz_uint64) * 2)
#define tmz_ZIP64_MAX_CENTRAL_EXTRA_FIELD_SIZE (sizeof(tmz_uint16) * 2 + sizeof(tmz_uint64) * 3)
static tmz_uint32 tmz_zip_writer_create_zip64_extra_data(tmz_uint8 *pBuf, tmz_uint64 *pUncomp_size, tmz_uint64 *pComp_size, tmz_uint64 *pLocal_header_ofs)
{
    tmz_uint8 *pDst = pBuf;
	tmz_uint32 field_size = 0;

    tmz_WRITE_LE16(pDst + 0, tmz_ZIP64_EXTENDED_INFORMATION_FIELD_HEADER_ID);
    tmz_WRITE_LE16(pDst + 2, 0);
    pDst += sizeof(tmz_uint16) * 2;

    if (pUncomp_size)
    {
        tmz_WRITE_LE64(pDst, *pUncomp_size);
        pDst += sizeof(tmz_uint64);
        field_size += sizeof(tmz_uint64);
    }

    if (pComp_size)
    {
        tmz_WRITE_LE64(pDst, *pComp_size);
        pDst += sizeof(tmz_uint64);
        field_size += sizeof(tmz_uint64);
    }

    if (pLocal_header_ofs)
    {
        tmz_WRITE_LE64(pDst, *pLocal_header_ofs);
        pDst += sizeof(tmz_uint64);
        field_size += sizeof(tmz_uint64);
    }

    tmz_WRITE_LE16(pBuf + 2, field_size);

    return (tmz_uint32)(pDst - pBuf);
}

static tmz_bool tmz_zip_writer_create_local_dir_header(tmz_zip_archive *pZip, tmz_uint8 *pDst, tmz_uint16 filename_size, tmz_uint16 extra_size, tmz_uint64 uncomp_size, tmz_uint64 comp_size, tmz_uint32 uncomp_crc32, tmz_uint16 method, tmz_uint16 bit_flags, tmz_uint16 dos_time, tmz_uint16 dos_date)
{
    (void)pZip;
    memset(pDst, 0, tmz_ZIP_LOCAL_DIR_HEADER_SIZE);
    tmz_WRITE_LE32(pDst + tmz_ZIP_LDH_SIG_OFS, tmz_ZIP_LOCAL_DIR_HEADER_SIG);
    tmz_WRITE_LE16(pDst + tmz_ZIP_LDH_VERSION_NEEDED_OFS, method ? 20 : 0);
    tmz_WRITE_LE16(pDst + tmz_ZIP_LDH_BIT_FLAG_OFS, bit_flags);
    tmz_WRITE_LE16(pDst + tmz_ZIP_LDH_METHOD_OFS, method);
    tmz_WRITE_LE16(pDst + tmz_ZIP_LDH_FILE_TIME_OFS, dos_time);
    tmz_WRITE_LE16(pDst + tmz_ZIP_LDH_FILE_DATE_OFS, dos_date);
    tmz_WRITE_LE32(pDst + tmz_ZIP_LDH_CRC32_OFS, uncomp_crc32);
    tmz_WRITE_LE32(pDst + tmz_ZIP_LDH_COMPRESSED_SIZE_OFS, tmz_MIN(comp_size, tmz_UINT32_MAX));
    tmz_WRITE_LE32(pDst + tmz_ZIP_LDH_DECOMPRESSED_SIZE_OFS, tmz_MIN(uncomp_size, tmz_UINT32_MAX));
    tmz_WRITE_LE16(pDst + tmz_ZIP_LDH_FILENAME_LEN_OFS, filename_size);
    tmz_WRITE_LE16(pDst + tmz_ZIP_LDH_EXTRA_LEN_OFS, extra_size);
    return tmz_TRUE;
}

static tmz_bool tmz_zip_writer_create_central_dir_header(tmz_zip_archive *pZip, tmz_uint8 *pDst,
                                                       tmz_uint16 filename_size, tmz_uint16 extra_size, tmz_uint16 comment_size,
                                                       tmz_uint64 uncomp_size, tmz_uint64 comp_size, tmz_uint32 uncomp_crc32,
                                                       tmz_uint16 method, tmz_uint16 bit_flags, tmz_uint16 dos_time, tmz_uint16 dos_date,
                                                       tmz_uint64 local_header_ofs, tmz_uint32 ext_attributes)
{
    (void)pZip;
    memset(pDst, 0, tmz_ZIP_CENTRAL_DIR_HEADER_SIZE);
    tmz_WRITE_LE32(pDst + tmz_ZIP_CDH_SIG_OFS, tmz_ZIP_CENTRAL_DIR_HEADER_SIG);
    tmz_WRITE_LE16(pDst + tmz_ZIP_CDH_VERSION_NEEDED_OFS, method ? 20 : 0);
    tmz_WRITE_LE16(pDst + tmz_ZIP_CDH_BIT_FLAG_OFS, bit_flags);
    tmz_WRITE_LE16(pDst + tmz_ZIP_CDH_METHOD_OFS, method);
    tmz_WRITE_LE16(pDst + tmz_ZIP_CDH_FILE_TIME_OFS, dos_time);
    tmz_WRITE_LE16(pDst + tmz_ZIP_CDH_FILE_DATE_OFS, dos_date);
    tmz_WRITE_LE32(pDst + tmz_ZIP_CDH_CRC32_OFS, uncomp_crc32);
    tmz_WRITE_LE32(pDst + tmz_ZIP_CDH_COMPRESSED_SIZE_OFS, tmz_MIN(comp_size, tmz_UINT32_MAX));
    tmz_WRITE_LE32(pDst + tmz_ZIP_CDH_DECOMPRESSED_SIZE_OFS, tmz_MIN(uncomp_size, tmz_UINT32_MAX));
    tmz_WRITE_LE16(pDst + tmz_ZIP_CDH_FILENAME_LEN_OFS, filename_size);
    tmz_WRITE_LE16(pDst + tmz_ZIP_CDH_EXTRA_LEN_OFS, extra_size);
    tmz_WRITE_LE16(pDst + tmz_ZIP_CDH_COMMENT_LEN_OFS, comment_size);
    tmz_WRITE_LE32(pDst + tmz_ZIP_CDH_EXTERNAL_ATTR_OFS, ext_attributes);
    tmz_WRITE_LE32(pDst + tmz_ZIP_CDH_LOCAL_HEADER_OFS, tmz_MIN(local_header_ofs, tmz_UINT32_MAX));
    return tmz_TRUE;
}

static tmz_bool tmz_zip_writer_add_to_central_dir(tmz_zip_archive *pZip, const char *pFilename, tmz_uint16 filename_size,
                                                const void *pExtra, tmz_uint16 extra_size, const void *pComment, tmz_uint16 comment_size,
                                                tmz_uint64 uncomp_size, tmz_uint64 comp_size, tmz_uint32 uncomp_crc32,
                                                tmz_uint16 method, tmz_uint16 bit_flags, tmz_uint16 dos_time, tmz_uint16 dos_date,
                                                tmz_uint64 local_header_ofs, tmz_uint32 ext_attributes,
                                                const char *user_extra_data, tmz_uint user_extra_data_len)
{
    tmz_zip_internal_state *pState = pZip->m_pState;
    tmz_uint32 central_dir_ofs = (tmz_uint32)pState->m_central_dir.m_size;
    size_t orig_central_dir_size = pState->m_central_dir.m_size;
    tmz_uint8 central_dir_header[tmz_ZIP_CENTRAL_DIR_HEADER_SIZE];

    if (!pZip->m_pState->m_zip64)
    {
        if (local_header_ofs > 0xFFFFFFFF)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_TOO_LARGE);
    }

    /* miniz doesn't support central dirs >= tmz_UINT32_MAX bytes yet */
    if (((tmz_uint64)pState->m_central_dir.m_size + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + filename_size + extra_size + user_extra_data_len + comment_size) >= tmz_UINT32_MAX)
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_CDIR_SIZE);

    if (!tmz_zip_writer_create_central_dir_header(pZip, central_dir_header, filename_size, extra_size + user_extra_data_len, comment_size, uncomp_size, comp_size, uncomp_crc32, method, bit_flags, dos_time, dos_date, local_header_ofs, ext_attributes))
        return tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);

    if ((!tmz_zip_array_push_back(pZip, &pState->m_central_dir, central_dir_header, tmz_ZIP_CENTRAL_DIR_HEADER_SIZE)) ||
        (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, pFilename, filename_size)) ||
        (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, pExtra, extra_size)) ||
        (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, user_extra_data, user_extra_data_len)) ||
        (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, pComment, comment_size)) ||
        (!tmz_zip_array_push_back(pZip, &pState->m_central_dir_offsets, &central_dir_ofs, 1)))
    {
        /* Try to resize the central directory array back into its original state. */
        tmz_zip_array_resize(pZip, &pState->m_central_dir, orig_central_dir_size, tmz_FALSE);
        return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
    }

    return tmz_TRUE;
}

static tmz_bool tmz_zip_writer_validate_archive_name(const char *pArchive_name)
{
    /* Basic ZIP archive filename validity checks: Valid filenames cannot start with a forward slash, cannot contain a drive letter, and cannot use DOS-style backward slashes. */
    if (*pArchive_name == '/')
        return tmz_FALSE;

    while (*pArchive_name)
    {
        if ((*pArchive_name == '\\') || (*pArchive_name == ':'))
            return tmz_FALSE;

        pArchive_name++;
    }

    return tmz_TRUE;
}

static tmz_uint tmz_zip_writer_compute_padding_needed_for_file_alignment(tmz_zip_archive *pZip)
{
    tmz_uint32 n;
    if (!pZip->m_file_offset_alignment)
        return 0;
    n = (tmz_uint32)(pZip->m_archive_size & (pZip->m_file_offset_alignment - 1));
    return (tmz_uint)((pZip->m_file_offset_alignment - n) & (pZip->m_file_offset_alignment - 1));
}

static tmz_bool tmz_zip_writer_write_zeros(tmz_zip_archive *pZip, tmz_uint64 cur_file_ofs, tmz_uint32 n)
{
    char buf[4096];
    memset(buf, 0, tmz_MIN(sizeof(buf), n));
    while (n)
    {
        tmz_uint32 s = tmz_MIN(sizeof(buf), n);
        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_file_ofs, buf, s) != s)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        cur_file_ofs += s;
        n -= s;
    }
    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_add_mem_ex(tmz_zip_archive *pZip, const char *pArchive_name, const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags,
                                 tmz_uint64 uncomp_size, tmz_uint32 uncomp_crc32)
{
    return tmz_zip_writer_add_mem_ex_v2(pZip, pArchive_name, pBuf, buf_size, pComment, comment_size, level_and_flags, uncomp_size, uncomp_crc32, NULL, NULL, 0, NULL, 0);
}

tmz_bool tmz_zip_writer_add_mem_ex_v2(tmz_zip_archive *pZip, const char *pArchive_name, const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size,
                                    tmz_uint level_and_flags, tmz_uint64 uncomp_size, tmz_uint32 uncomp_crc32, tmz_TIME_T *last_modified,
                                    const char *user_extra_data, tmz_uint user_extra_data_len, const char *user_extra_data_central, tmz_uint user_extra_data_central_len)
{
    tmz_uint16 method = 0, dos_time = 0, dos_date = 0;
    tmz_uint level, ext_attributes = 0, num_alignment_padding_bytes;
    tmz_uint64 local_dir_header_ofs = pZip->m_archive_size, cur_archive_file_ofs = pZip->m_archive_size, comp_size = 0;
    size_t archive_name_size;
    tmz_uint8 local_dir_header[tmz_ZIP_LOCAL_DIR_HEADER_SIZE];
    tdefl_compressor *pComp = NULL;
    tmz_bool store_data_uncompressed;
    tmz_zip_internal_state *pState;
    tmz_uint8 *pExtra_data = NULL;
    tmz_uint32 extra_size = 0;
    tmz_uint8 extra_data[tmz_ZIP64_MAX_CENTRAL_EXTRA_FIELD_SIZE];
    tmz_uint16 bit_flags = 0;

    if (uncomp_size || (buf_size && !(level_and_flags & tmz_ZIP_FLAG_COMPRESSED_DATA)))
        bit_flags |= tmz_ZIP_LDH_BIT_FLAG_HAS_LOCATOR;

    if (!(level_and_flags & tmz_ZIP_FLAG_ASCII_FILENAME))
        bit_flags |= tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_UTF8;

    if ((int)level_and_flags < 0)
        level_and_flags = tmz_DEFAULT_LEVEL;
    level = level_and_flags & 0xF;
    store_data_uncompressed = ((!level) || (level_and_flags & tmz_ZIP_FLAG_COMPRESSED_DATA));

    if ((!pZip) || (!pZip->m_pState) || (pZip->m_zip_mode != tmz_ZIP_MODE_WRITING) || ((buf_size) && (!pBuf)) || (!pArchive_name) || ((comment_size) && (!pComment)) || (level > tmz_UBER_COMPRESSION))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    pState = pZip->m_pState;

    if (pState->m_zip64)
    {
        if (pZip->m_total_files == tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);
    }
    else
    {
        if (pZip->m_total_files == tmz_UINT16_MAX)
        {
            pState->m_zip64 = tmz_TRUE;
            /*return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES); */
        }
        if ((buf_size > 0xFFFFFFFF) || (uncomp_size > 0xFFFFFFFF))
        {
            pState->m_zip64 = tmz_TRUE;
            /*return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE); */
        }
    }

    if ((!(level_and_flags & tmz_ZIP_FLAG_COMPRESSED_DATA)) && (uncomp_size))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (!tmz_zip_writer_validate_archive_name(pArchive_name))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_FILENAME);

#ifndef tminiz_NO_TIME
    if (last_modified != NULL)
    {
        tmz_zip_time_t_to_dos_time(*last_modified, &dos_time, &dos_date);
    }
    else
    {
		tmz_TIME_T cur_time;
		time(&cur_time);
		tmz_zip_time_t_to_dos_time(cur_time, &dos_time, &dos_date);
    }
#endif /* #ifndef tminiz_NO_TIME */

    archive_name_size = strlen(pArchive_name);
    if (archive_name_size > tmz_UINT16_MAX)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_FILENAME);

    num_alignment_padding_bytes = tmz_zip_writer_compute_padding_needed_for_file_alignment(pZip);

    /* miniz doesn't support central dirs >= tmz_UINT32_MAX bytes yet */
    if (((tmz_uint64)pState->m_central_dir.m_size + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + archive_name_size + tmz_ZIP64_MAX_CENTRAL_EXTRA_FIELD_SIZE + comment_size) >= tmz_UINT32_MAX)
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_CDIR_SIZE);

    if (!pState->m_zip64)
    {
        /* Bail early if the archive would obviously become too large */
        if ((pZip->m_archive_size + num_alignment_padding_bytes + tmz_ZIP_LOCAL_DIR_HEADER_SIZE + archive_name_size + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + archive_name_size + comment_size + pState->m_central_dir.m_size + tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE) > 0xFFFFFFFF)
        {
            pState->m_zip64 = tmz_TRUE;
            /*return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE); */
        }
    }

    if ((archive_name_size) && (pArchive_name[archive_name_size - 1] == '/'))
    {
        /* Set DOS Subdirectory attribute bit. */
        ext_attributes |= tmz_ZIP_DOS_DIR_ATTRIBUTE_BITFLAG;

        /* Subdirectories cannot contain data. */
        if ((buf_size) || (uncomp_size))
            return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
    }

    /* Try to do any allocations before writing to the archive, so if an allocation fails the file remains unmodified. (A good idea if we're doing an in-place modification.) */
    if ((!tmz_zip_array_ensure_room(pZip, &pState->m_central_dir, tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + archive_name_size + comment_size + (pState->m_zip64 ? tmz_ZIP64_MAX_CENTRAL_EXTRA_FIELD_SIZE : 0))) || (!tmz_zip_array_ensure_room(pZip, &pState->m_central_dir_offsets, 1)))
        return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

    if ((!store_data_uncompressed) && (buf_size))
    {
        if (NULL == (pComp = (tdefl_compressor *)pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, sizeof(tdefl_compressor))))
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
    }

    if (!tmz_zip_writer_write_zeros(pZip, cur_archive_file_ofs, num_alignment_padding_bytes))
    {
        pZip->m_pFree(pZip->m_pAlloc_opaque, pComp);
        return tmz_FALSE;
    }

    local_dir_header_ofs += num_alignment_padding_bytes;
    if (pZip->m_file_offset_alignment)
    {
        tmz_ASSERT((local_dir_header_ofs & (pZip->m_file_offset_alignment - 1)) == 0);
    }
    cur_archive_file_ofs += num_alignment_padding_bytes;

    tmz_CLEAR_OBJ(local_dir_header);

    if (!store_data_uncompressed || (level_and_flags & tmz_ZIP_FLAG_COMPRESSED_DATA))
    {
        method = tmz_DEFLATED;
    }

    if (pState->m_zip64)
    {
        if (uncomp_size >= tmz_UINT32_MAX || local_dir_header_ofs >= tmz_UINT32_MAX)
        {
            pExtra_data = extra_data;
            extra_size = tmz_zip_writer_create_zip64_extra_data(extra_data, (uncomp_size >= tmz_UINT32_MAX) ? &uncomp_size : NULL,
                                                               (uncomp_size >= tmz_UINT32_MAX) ? &comp_size : NULL, (local_dir_header_ofs >= tmz_UINT32_MAX) ? &local_dir_header_ofs : NULL);
        }

        if (!tmz_zip_writer_create_local_dir_header(pZip, local_dir_header, (tmz_uint16)archive_name_size, extra_size + user_extra_data_len, 0, 0, 0, method, bit_flags, dos_time, dos_date))
            return tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);

        if (pZip->m_pWrite(pZip->m_pIO_opaque, local_dir_header_ofs, local_dir_header, sizeof(local_dir_header)) != sizeof(local_dir_header))
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        cur_archive_file_ofs += sizeof(local_dir_header);

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, pArchive_name, archive_name_size) != archive_name_size)
        {
            pZip->m_pFree(pZip->m_pAlloc_opaque, pComp);
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
        }
        cur_archive_file_ofs += archive_name_size;

        if (pExtra_data != NULL)
        {
            if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, extra_data, extra_size) != extra_size)
                return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

            cur_archive_file_ofs += extra_size;
        }
    }
    else
    {
        if ((comp_size > tmz_UINT32_MAX) || (cur_archive_file_ofs > tmz_UINT32_MAX))
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);
        if (!tmz_zip_writer_create_local_dir_header(pZip, local_dir_header, (tmz_uint16)archive_name_size, user_extra_data_len, 0, 0, 0, method, bit_flags, dos_time, dos_date))
            return tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);

        if (pZip->m_pWrite(pZip->m_pIO_opaque, local_dir_header_ofs, local_dir_header, sizeof(local_dir_header)) != sizeof(local_dir_header))
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        cur_archive_file_ofs += sizeof(local_dir_header);

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, pArchive_name, archive_name_size) != archive_name_size)
        {
            pZip->m_pFree(pZip->m_pAlloc_opaque, pComp);
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
        }
        cur_archive_file_ofs += archive_name_size;
    }

    if (user_extra_data_len > 0)
    {
        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, user_extra_data, user_extra_data_len) != user_extra_data_len)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        cur_archive_file_ofs += user_extra_data_len;
    }

    if (!(level_and_flags & tmz_ZIP_FLAG_COMPRESSED_DATA))
    {
        uncomp_crc32 = (tmz_uint32)tmz_crc32(tmz_CRC32_INIT, (const tmz_uint8 *)pBuf, buf_size);
        uncomp_size = buf_size;
        if (uncomp_size <= 3)
        {
            level = 0;
            store_data_uncompressed = tmz_TRUE;
        }
    }

    if (store_data_uncompressed)
    {
        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, pBuf, buf_size) != buf_size)
        {
            pZip->m_pFree(pZip->m_pAlloc_opaque, pComp);
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
        }

        cur_archive_file_ofs += buf_size;
        comp_size = buf_size;
    }
    else if (buf_size)
    {
        tmz_zip_writer_add_state state;

        state.m_pZip = pZip;
        state.m_cur_archive_file_ofs = cur_archive_file_ofs;
        state.m_comp_size = 0;

        if ((tdefl_init(pComp, tmz_zip_writer_add_put_buf_callback, &state, tdefl_create_comp_flags_from_zip_params(level, -15, tmz_DEFAULT_STRATEGY)) != TDEFL_STATUS_OKAY) ||
            (tdefl_compress_buffer(pComp, pBuf, buf_size, TDEFL_FINISH) != TDEFL_STATUS_DONE))
        {
            pZip->m_pFree(pZip->m_pAlloc_opaque, pComp);
            return tmz_zip_set_error(pZip, tmz_ZIP_COMPRESSION_FAILED);
        }

        comp_size = state.m_comp_size;
        cur_archive_file_ofs = state.m_cur_archive_file_ofs;
    }

    pZip->m_pFree(pZip->m_pAlloc_opaque, pComp);
    pComp = NULL;

    if (uncomp_size)
    {
		tmz_uint8 local_dir_footer[tmz_ZIP_DATA_DESCRIPTER_SIZE64];
		tmz_uint32 local_dir_footer_size = tmz_ZIP_DATA_DESCRIPTER_SIZE32;

        tmz_ASSERT(bit_flags & tmz_ZIP_LDH_BIT_FLAG_HAS_LOCATOR);

        tmz_WRITE_LE32(local_dir_footer + 0, tmz_ZIP_DATA_DESCRIPTOR_ID);
        tmz_WRITE_LE32(local_dir_footer + 4, uncomp_crc32);
        if (pExtra_data == NULL)
        {
            if ((comp_size > tmz_UINT32_MAX) || (cur_archive_file_ofs > tmz_UINT32_MAX))
                return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);

            tmz_WRITE_LE32(local_dir_footer + 8, comp_size);
            tmz_WRITE_LE32(local_dir_footer + 12, uncomp_size);
        }
        else
        {
            tmz_WRITE_LE64(local_dir_footer + 8, comp_size);
            tmz_WRITE_LE64(local_dir_footer + 16, uncomp_size);
            local_dir_footer_size = tmz_ZIP_DATA_DESCRIPTER_SIZE64;
        }

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, local_dir_footer, local_dir_footer_size) != local_dir_footer_size)
            return tmz_FALSE;

        cur_archive_file_ofs += local_dir_footer_size;
    }

    if (pExtra_data != NULL)
    {
        extra_size = tmz_zip_writer_create_zip64_extra_data(extra_data, (uncomp_size >= tmz_UINT32_MAX) ? &uncomp_size : NULL,
                                                           (uncomp_size >= tmz_UINT32_MAX) ? &comp_size : NULL, (local_dir_header_ofs >= tmz_UINT32_MAX) ? &local_dir_header_ofs : NULL);
    }

    if (!tmz_zip_writer_add_to_central_dir(pZip, pArchive_name, (tmz_uint16)archive_name_size, pExtra_data, extra_size, pComment,
                                          comment_size, uncomp_size, comp_size, uncomp_crc32, method, bit_flags, dos_time, dos_date, local_dir_header_ofs, ext_attributes,
                                          user_extra_data_central, user_extra_data_central_len))
        return tmz_FALSE;

    pZip->m_total_files++;
    pZip->m_archive_size = cur_archive_file_ofs;

    return tmz_TRUE;
}

#ifndef tminiz_NO_STDIO
tmz_bool tmz_zip_writer_add_cfile(tmz_zip_archive *pZip, const char *pArchive_name, tmz_FILE *pSrc_file, tmz_uint64 size_to_add, const tmz_TIME_T *pFile_time, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags,
                                const char *user_extra_data, tmz_uint user_extra_data_len, const char *user_extra_data_central, tmz_uint user_extra_data_central_len)
{
    tmz_uint16 gen_flags = tmz_ZIP_LDH_BIT_FLAG_HAS_LOCATOR;
    tmz_uint uncomp_crc32 = tmz_CRC32_INIT, level, num_alignment_padding_bytes;
    tmz_uint16 method = 0, dos_time = 0, dos_date = 0, ext_attributes = 0;
    tmz_uint64 local_dir_header_ofs, cur_archive_file_ofs = pZip->m_archive_size, uncomp_size = size_to_add, comp_size = 0;
    size_t archive_name_size;
    tmz_uint8 local_dir_header[tmz_ZIP_LOCAL_DIR_HEADER_SIZE];
    tmz_uint8 *pExtra_data = NULL;
    tmz_uint32 extra_size = 0;
    tmz_uint8 extra_data[tmz_ZIP64_MAX_CENTRAL_EXTRA_FIELD_SIZE];
    tmz_zip_internal_state *pState;

    if (!(level_and_flags & tmz_ZIP_FLAG_ASCII_FILENAME))
        gen_flags |= tmz_ZIP_GENERAL_PURPOSE_BIT_FLAG_UTF8;

    if ((int)level_and_flags < 0)
        level_and_flags = tmz_DEFAULT_LEVEL;
    level = level_and_flags & 0xF;

    /* Sanity checks */
    if ((!pZip) || (!pZip->m_pState) || (pZip->m_zip_mode != tmz_ZIP_MODE_WRITING) || (!pArchive_name) || ((comment_size) && (!pComment)) || (level > tmz_UBER_COMPRESSION))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    pState = pZip->m_pState;

    if ((!pState->m_zip64) && (uncomp_size > tmz_UINT32_MAX))
    {
        /* Source file is too large for non-zip64 */
        /*return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE); */
        pState->m_zip64 = tmz_TRUE;
    }

    /* We could support this, but why? */
    if (level_and_flags & tmz_ZIP_FLAG_COMPRESSED_DATA)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (!tmz_zip_writer_validate_archive_name(pArchive_name))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_FILENAME);

    if (pState->m_zip64)
    {
        if (pZip->m_total_files == tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);
    }
    else
    {
        if (pZip->m_total_files == tmz_UINT16_MAX)
        {
            pState->m_zip64 = tmz_TRUE;
            /*return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES); */
        }
    }

    archive_name_size = strlen(pArchive_name);
    if (archive_name_size > tmz_UINT16_MAX)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_FILENAME);

    num_alignment_padding_bytes = tmz_zip_writer_compute_padding_needed_for_file_alignment(pZip);

    /* miniz doesn't support central dirs >= tmz_UINT32_MAX bytes yet */
    if (((tmz_uint64)pState->m_central_dir.m_size + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + archive_name_size + tmz_ZIP64_MAX_CENTRAL_EXTRA_FIELD_SIZE + comment_size) >= tmz_UINT32_MAX)
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_CDIR_SIZE);

    if (!pState->m_zip64)
    {
        /* Bail early if the archive would obviously become too large */
        if ((pZip->m_archive_size + num_alignment_padding_bytes + tmz_ZIP_LOCAL_DIR_HEADER_SIZE + archive_name_size + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + archive_name_size + comment_size + pState->m_central_dir.m_size + tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE + 1024) > 0xFFFFFFFF)
        {
            pState->m_zip64 = tmz_TRUE;
            /*return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE); */
        }
    }

#ifndef tminiz_NO_TIME
    if (pFile_time)
    {
        tmz_zip_time_t_to_dos_time(*pFile_time, &dos_time, &dos_date);
    }
#endif

    if (uncomp_size <= 3)
        level = 0;

    if (!tmz_zip_writer_write_zeros(pZip, cur_archive_file_ofs, num_alignment_padding_bytes))
    {
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
    }

    cur_archive_file_ofs += num_alignment_padding_bytes;
    local_dir_header_ofs = cur_archive_file_ofs;

    if (pZip->m_file_offset_alignment)
    {
        tmz_ASSERT((cur_archive_file_ofs & (pZip->m_file_offset_alignment - 1)) == 0);
    }

    if (uncomp_size && level)
    {
        method = tmz_DEFLATED;
    }

    tmz_CLEAR_OBJ(local_dir_header);
    if (pState->m_zip64)
    {
        if (uncomp_size >= tmz_UINT32_MAX || local_dir_header_ofs >= tmz_UINT32_MAX)
        {
            pExtra_data = extra_data;
            extra_size = tmz_zip_writer_create_zip64_extra_data(extra_data, (uncomp_size >= tmz_UINT32_MAX) ? &uncomp_size : NULL,
                                                               (uncomp_size >= tmz_UINT32_MAX) ? &comp_size : NULL, (local_dir_header_ofs >= tmz_UINT32_MAX) ? &local_dir_header_ofs : NULL);
        }

        if (!tmz_zip_writer_create_local_dir_header(pZip, local_dir_header, (tmz_uint16)archive_name_size, extra_size + user_extra_data_len, 0, 0, 0, method, gen_flags, dos_time, dos_date))
            return tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, local_dir_header, sizeof(local_dir_header)) != sizeof(local_dir_header))
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        cur_archive_file_ofs += sizeof(local_dir_header);

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, pArchive_name, archive_name_size) != archive_name_size)
        {
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
        }

        cur_archive_file_ofs += archive_name_size;

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, extra_data, extra_size) != extra_size)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        cur_archive_file_ofs += extra_size;
    }
    else
    {
        if ((comp_size > tmz_UINT32_MAX) || (cur_archive_file_ofs > tmz_UINT32_MAX))
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);
        if (!tmz_zip_writer_create_local_dir_header(pZip, local_dir_header, (tmz_uint16)archive_name_size, user_extra_data_len, 0, 0, 0, method, gen_flags, dos_time, dos_date))
            return tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, local_dir_header, sizeof(local_dir_header)) != sizeof(local_dir_header))
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        cur_archive_file_ofs += sizeof(local_dir_header);

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, pArchive_name, archive_name_size) != archive_name_size)
        {
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
        }

        cur_archive_file_ofs += archive_name_size;
    }

    if (user_extra_data_len > 0)
    {
        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, user_extra_data, user_extra_data_len) != user_extra_data_len)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        cur_archive_file_ofs += user_extra_data_len;
    }

    if (uncomp_size)
    {
        tmz_uint64 uncomp_remaining = uncomp_size;
        void *pRead_buf = pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, tmz_ZIP_MAX_IO_BUF_SIZE);
        if (!pRead_buf)
        {
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }

        if (!level)
        {
            while (uncomp_remaining)
            {
                tmz_uint n = (tmz_uint)tmz_MIN((tmz_uint64)tmz_ZIP_MAX_IO_BUF_SIZE, uncomp_remaining);
                if ((tmz_FREAD(pRead_buf, 1, n, pSrc_file) != n) || (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, pRead_buf, n) != n))
                {
                    pZip->m_pFree(pZip->m_pAlloc_opaque, pRead_buf);
                    return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
                }
                uncomp_crc32 = (tmz_uint32)tmz_crc32(uncomp_crc32, (const tmz_uint8 *)pRead_buf, n);
                uncomp_remaining -= n;
                cur_archive_file_ofs += n;
            }
            comp_size = uncomp_size;
        }
        else
        {
            tmz_bool result = tmz_FALSE;
            tmz_zip_writer_add_state state;
            tdefl_compressor *pComp = (tdefl_compressor *)pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, sizeof(tdefl_compressor));
            if (!pComp)
            {
                pZip->m_pFree(pZip->m_pAlloc_opaque, pRead_buf);
                return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
            }

            state.m_pZip = pZip;
            state.m_cur_archive_file_ofs = cur_archive_file_ofs;
            state.m_comp_size = 0;

            if (tdefl_init(pComp, tmz_zip_writer_add_put_buf_callback, &state, tdefl_create_comp_flags_from_zip_params(level, -15, tmz_DEFAULT_STRATEGY)) != TDEFL_STATUS_OKAY)
            {
                pZip->m_pFree(pZip->m_pAlloc_opaque, pComp);
                pZip->m_pFree(pZip->m_pAlloc_opaque, pRead_buf);
                return tmz_zip_set_error(pZip, tmz_ZIP_INTERNAL_ERROR);
            }

            for (;;)
            {
                size_t in_buf_size = (tmz_uint32)tmz_MIN(uncomp_remaining, (tmz_uint64)tmz_ZIP_MAX_IO_BUF_SIZE);
                tdefl_status status;
				tdefl_flush flush = TDEFL_NO_FLUSH;

                if (tmz_FREAD(pRead_buf, 1, in_buf_size, pSrc_file) != in_buf_size)
                {
                    tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
                    break;
                }

                uncomp_crc32 = (tmz_uint32)tmz_crc32(uncomp_crc32, (const tmz_uint8 *)pRead_buf, in_buf_size);
                uncomp_remaining -= in_buf_size;

				if (pZip->m_pNeeds_keepalive != NULL && pZip->m_pNeeds_keepalive(pZip->m_pIO_opaque))
					flush = TDEFL_FULL_FLUSH;

                status = tdefl_compress_buffer(pComp, pRead_buf, in_buf_size, uncomp_remaining ? flush : TDEFL_FINISH);
                if (status == TDEFL_STATUS_DONE)
                {
                    result = tmz_TRUE;
                    break;
                }
                else if (status != TDEFL_STATUS_OKAY)
                {
                    tmz_zip_set_error(pZip, tmz_ZIP_COMPRESSION_FAILED);
                    break;
                }
            }

            pZip->m_pFree(pZip->m_pAlloc_opaque, pComp);

            if (!result)
            {
                pZip->m_pFree(pZip->m_pAlloc_opaque, pRead_buf);
                return tmz_FALSE;
            }

            comp_size = state.m_comp_size;
            cur_archive_file_ofs = state.m_cur_archive_file_ofs;
        }

        pZip->m_pFree(pZip->m_pAlloc_opaque, pRead_buf);
    }

	{
		tmz_uint8 local_dir_footer[tmz_ZIP_DATA_DESCRIPTER_SIZE64];
		tmz_uint32 local_dir_footer_size = tmz_ZIP_DATA_DESCRIPTER_SIZE32;

		tmz_WRITE_LE32(local_dir_footer + 0, tmz_ZIP_DATA_DESCRIPTOR_ID);
		tmz_WRITE_LE32(local_dir_footer + 4, uncomp_crc32);
		if (pExtra_data == NULL)
		{
			if (comp_size > tmz_UINT32_MAX)
				return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);

			tmz_WRITE_LE32(local_dir_footer + 8, comp_size);
			tmz_WRITE_LE32(local_dir_footer + 12, uncomp_size);
		}
		else
		{
			tmz_WRITE_LE64(local_dir_footer + 8, comp_size);
			tmz_WRITE_LE64(local_dir_footer + 16, uncomp_size);
			local_dir_footer_size = tmz_ZIP_DATA_DESCRIPTER_SIZE64;
		}

		if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_archive_file_ofs, local_dir_footer, local_dir_footer_size) != local_dir_footer_size)
			return tmz_FALSE;

		cur_archive_file_ofs += local_dir_footer_size;
	}

    if (pExtra_data != NULL)
    {
        extra_size = tmz_zip_writer_create_zip64_extra_data(extra_data, (uncomp_size >= tmz_UINT32_MAX) ? &uncomp_size : NULL,
                                                           (uncomp_size >= tmz_UINT32_MAX) ? &comp_size : NULL, (local_dir_header_ofs >= tmz_UINT32_MAX) ? &local_dir_header_ofs : NULL);
    }

    if (!tmz_zip_writer_add_to_central_dir(pZip, pArchive_name, (tmz_uint16)archive_name_size, pExtra_data, extra_size, pComment, comment_size,
                                          uncomp_size, comp_size, uncomp_crc32, method, gen_flags, dos_time, dos_date, local_dir_header_ofs, ext_attributes,
                                          user_extra_data_central, user_extra_data_central_len))
        return tmz_FALSE;

    pZip->m_total_files++;
    pZip->m_archive_size = cur_archive_file_ofs;

    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_add_file(tmz_zip_archive *pZip, const char *pArchive_name, const char *pSrc_filename, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags)
{
    tmz_FILE *pSrc_file = NULL;
    tmz_uint64 uncomp_size = 0;
    tmz_TIME_T file_modified_time;
    tmz_TIME_T *pFile_time = NULL;
	tmz_bool status;

    memset(&file_modified_time, 0, sizeof(file_modified_time));

#if !defined(tminiz_NO_TIME) && !defined(tminiz_NO_STDIO)
    pFile_time = &file_modified_time;
    if (!tmz_zip_get_file_modified_time(pSrc_filename, &file_modified_time))
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_STAT_FAILED);
#endif

    pSrc_file = tmz_FOPEN(pSrc_filename, "rb");
    if (!pSrc_file)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_OPEN_FAILED);

    tmz_FSEEK64(pSrc_file, 0, SEEK_END);
    uncomp_size = tmz_FTELL64(pSrc_file);
    tmz_FSEEK64(pSrc_file, 0, SEEK_SET);

    status = tmz_zip_writer_add_cfile(pZip, pArchive_name, pSrc_file, uncomp_size, pFile_time, pComment, comment_size, level_and_flags, NULL, 0, NULL, 0);

    tmz_FCLOSE(pSrc_file);

    return status;
}
#endif /* #ifndef tminiz_NO_STDIO */

static tmz_bool tmz_zip_writer_update_zip64_extension_block(tmz_zip_array *pNew_ext, tmz_zip_archive *pZip, const tmz_uint8 *pExt, uint32_t ext_len, tmz_uint64 *pComp_size, tmz_uint64 *pUncomp_size, tmz_uint64 *pLocal_header_ofs, tmz_uint32 *pDisk_start)
{
    /* + 64 should be enough for any new zip64 data */
    if (!tmz_zip_array_reserve(pZip, pNew_ext, ext_len + 64, tmz_FALSE))
        return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

    tmz_zip_array_resize(pZip, pNew_ext, 0, tmz_FALSE);

    if ((pUncomp_size) || (pComp_size) || (pLocal_header_ofs) || (pDisk_start))
    {
        tmz_uint8 new_ext_block[64];
        tmz_uint8 *pDst = new_ext_block;
        tmz_write_le16(pDst, tmz_ZIP64_EXTENDED_INFORMATION_FIELD_HEADER_ID);
        tmz_write_le16(pDst + sizeof(tmz_uint16), 0);
        pDst += sizeof(tmz_uint16) * 2;

        if (pUncomp_size)
        {
            tmz_write_le64(pDst, *pUncomp_size);
            pDst += sizeof(tmz_uint64);
        }

        if (pComp_size)
        {
            tmz_write_le64(pDst, *pComp_size);
            pDst += sizeof(tmz_uint64);
        }

        if (pLocal_header_ofs)
        {
            tmz_write_le64(pDst, *pLocal_header_ofs);
            pDst += sizeof(tmz_uint64);
        }

        if (pDisk_start)
        {
            tmz_write_le32(pDst, *pDisk_start);
            pDst += sizeof(tmz_uint32);
        }

        tmz_write_le16(new_ext_block + sizeof(tmz_uint16), (tmz_uint16)((pDst - new_ext_block) - sizeof(tmz_uint16) * 2));

        if (!tmz_zip_array_push_back(pZip, pNew_ext, new_ext_block, pDst - new_ext_block))
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
    }

    if ((pExt) && (ext_len))
    {
        tmz_uint32 extra_size_remaining = ext_len;
        const tmz_uint8 *pExtra_data = pExt;

        do
        {
            tmz_uint32 field_id, field_data_size, field_total_size;

            if (extra_size_remaining < (sizeof(tmz_uint16) * 2))
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

            field_id = tmz_READ_LE16(pExtra_data);
            field_data_size = tmz_READ_LE16(pExtra_data + sizeof(tmz_uint16));
            field_total_size = field_data_size + sizeof(tmz_uint16) * 2;

            if (field_total_size > extra_size_remaining)
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

            if (field_id != tmz_ZIP64_EXTENDED_INFORMATION_FIELD_HEADER_ID)
            {
                if (!tmz_zip_array_push_back(pZip, pNew_ext, pExtra_data, field_total_size))
                    return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
            }

            pExtra_data += field_total_size;
            extra_size_remaining -= field_total_size;
        } while (extra_size_remaining);
    }

    return tmz_TRUE;
}

/* TODO: This func is now pretty freakin complex due to zip64, split it up? */
tmz_bool tmz_zip_writer_add_from_zip_reader(tmz_zip_archive *pZip, tmz_zip_archive *pSource_zip, tmz_uint src_file_index)
{
    tmz_uint n, bit_flags, num_alignment_padding_bytes, src_central_dir_following_data_size;
    tmz_uint64 src_archive_bytes_remaining, local_dir_header_ofs;
    tmz_uint64 cur_src_file_ofs, cur_dst_file_ofs;
    tmz_uint32 local_header_u32[(tmz_ZIP_LOCAL_DIR_HEADER_SIZE + sizeof(tmz_uint32) - 1) / sizeof(tmz_uint32)];
    tmz_uint8 *pLocal_header = (tmz_uint8 *)local_header_u32;
    tmz_uint8 new_central_header[tmz_ZIP_CENTRAL_DIR_HEADER_SIZE];
    size_t orig_central_dir_size;
    tmz_zip_internal_state *pState;
    void *pBuf;
    const tmz_uint8 *pSrc_central_header;
    tmz_zip_archive_file_stat src_file_stat;
    tmz_uint32 src_filename_len, src_comment_len, src_ext_len;
    tmz_uint32 local_header_filename_size, local_header_extra_len;
    tmz_uint64 local_header_comp_size, local_header_uncomp_size;
    tmz_bool found_zip64_ext_data_in_ldir = tmz_FALSE;

    /* Sanity checks */
    if ((!pZip) || (!pZip->m_pState) || (pZip->m_zip_mode != tmz_ZIP_MODE_WRITING) || (!pSource_zip->m_pRead))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    pState = pZip->m_pState;

    /* Don't support copying files from zip64 archives to non-zip64, even though in some cases this is possible */
    if ((pSource_zip->m_pState->m_zip64) && (!pZip->m_pState->m_zip64))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    /* Get pointer to the source central dir header and crack it */
    if (NULL == (pSrc_central_header = tmz_zip_get_cdh(pSource_zip, src_file_index)))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (tmz_READ_LE32(pSrc_central_header + tmz_ZIP_CDH_SIG_OFS) != tmz_ZIP_CENTRAL_DIR_HEADER_SIG)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    src_filename_len = tmz_READ_LE16(pSrc_central_header + tmz_ZIP_CDH_FILENAME_LEN_OFS);
    src_comment_len = tmz_READ_LE16(pSrc_central_header + tmz_ZIP_CDH_COMMENT_LEN_OFS);
    src_ext_len = tmz_READ_LE16(pSrc_central_header + tmz_ZIP_CDH_EXTRA_LEN_OFS);
    src_central_dir_following_data_size = src_filename_len + src_ext_len + src_comment_len;

    /* TODO: We don't support central dir's >= tmz_UINT32_MAX bytes right now (+32 fudge factor in case we need to add more extra data) */
    if ((pState->m_central_dir.m_size + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + src_central_dir_following_data_size + 32) >= tmz_UINT32_MAX)
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_CDIR_SIZE);

    num_alignment_padding_bytes = tmz_zip_writer_compute_padding_needed_for_file_alignment(pZip);

    if (!pState->m_zip64)
    {
        if (pZip->m_total_files == tmz_UINT16_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);
    }
    else
    {
        /* TODO: Our zip64 support still has some 32-bit limits that may not be worth fixing. */
        if (pZip->m_total_files == tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);
    }

    if (!tmz_zip_file_stat_internal(pSource_zip, src_file_index, pSrc_central_header, &src_file_stat, NULL))
        return tmz_FALSE;

    cur_src_file_ofs = src_file_stat.m_local_header_ofs;
    cur_dst_file_ofs = pZip->m_archive_size;

    /* Read the source archive's local dir header */
    if (pSource_zip->m_pRead(pSource_zip->m_pIO_opaque, cur_src_file_ofs, pLocal_header, tmz_ZIP_LOCAL_DIR_HEADER_SIZE) != tmz_ZIP_LOCAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);

    if (tmz_READ_LE32(pLocal_header) != tmz_ZIP_LOCAL_DIR_HEADER_SIG)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);

    cur_src_file_ofs += tmz_ZIP_LOCAL_DIR_HEADER_SIZE;

    /* Compute the total size we need to copy (filename+extra data+compressed data) */
    local_header_filename_size = tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_FILENAME_LEN_OFS);
    local_header_extra_len = tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_EXTRA_LEN_OFS);
    local_header_comp_size = tmz_READ_LE32(pLocal_header + tmz_ZIP_LDH_COMPRESSED_SIZE_OFS);
    local_header_uncomp_size = tmz_READ_LE32(pLocal_header + tmz_ZIP_LDH_DECOMPRESSED_SIZE_OFS);
    src_archive_bytes_remaining = local_header_filename_size + local_header_extra_len + src_file_stat.m_comp_size;

    /* Try to find a zip64 extended information field */
    if ((local_header_extra_len) && ((local_header_comp_size == tmz_UINT32_MAX) || (local_header_uncomp_size == tmz_UINT32_MAX)))
    {
        tmz_zip_array file_data_array;
		const tmz_uint8 *pExtra_data;
		tmz_uint32 extra_size_remaining = local_header_extra_len;

        tmz_zip_array_init(&file_data_array, 1);
        if (!tmz_zip_array_resize(pZip, &file_data_array, local_header_extra_len, tmz_FALSE))
        {
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }

        if (pSource_zip->m_pRead(pSource_zip->m_pIO_opaque, src_file_stat.m_local_header_ofs + tmz_ZIP_LOCAL_DIR_HEADER_SIZE + local_header_filename_size, file_data_array.m_p, local_header_extra_len) != local_header_extra_len)
        {
            tmz_zip_array_clear(pZip, &file_data_array);
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
        }

        pExtra_data = (const tmz_uint8 *)file_data_array.m_p;

        do
        {
            tmz_uint32 field_id, field_data_size, field_total_size;

            if (extra_size_remaining < (sizeof(tmz_uint16) * 2))
            {
                tmz_zip_array_clear(pZip, &file_data_array);
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);
            }

            field_id = tmz_READ_LE16(pExtra_data);
            field_data_size = tmz_READ_LE16(pExtra_data + sizeof(tmz_uint16));
            field_total_size = field_data_size + sizeof(tmz_uint16) * 2;

            if (field_total_size > extra_size_remaining)
            {
                tmz_zip_array_clear(pZip, &file_data_array);
                return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);
            }

            if (field_id == tmz_ZIP64_EXTENDED_INFORMATION_FIELD_HEADER_ID)
            {
                const tmz_uint8 *pSrc_field_data = pExtra_data + sizeof(tmz_uint32);

                if (field_data_size < sizeof(tmz_uint64) * 2)
                {
                    tmz_zip_array_clear(pZip, &file_data_array);
                    return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_HEADER_OR_CORRUPTED);
                }

                local_header_uncomp_size = tmz_READ_LE64(pSrc_field_data);
                local_header_comp_size = tmz_READ_LE64(pSrc_field_data + sizeof(tmz_uint64)); /* may be 0 if there's a descriptor */

                found_zip64_ext_data_in_ldir = tmz_TRUE;
                break;
            }

            pExtra_data += field_total_size;
            extra_size_remaining -= field_total_size;
        } while (extra_size_remaining);

        tmz_zip_array_clear(pZip, &file_data_array);
    }

    if (!pState->m_zip64)
    {
        /* Try to detect if the new archive will most likely wind up too big and bail early (+(sizeof(tmz_uint32) * 4) is for the optional descriptor which could be present, +64 is a fudge factor). */
        /* We also check when the archive is finalized so this doesn't need to be perfect. */
        tmz_uint64 approx_new_archive_size = cur_dst_file_ofs + num_alignment_padding_bytes + tmz_ZIP_LOCAL_DIR_HEADER_SIZE + src_archive_bytes_remaining + (sizeof(tmz_uint32) * 4) +
                                            pState->m_central_dir.m_size + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + src_central_dir_following_data_size + tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE + 64;

        if (approx_new_archive_size >= tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);
    }

    /* Write dest archive padding */
    if (!tmz_zip_writer_write_zeros(pZip, cur_dst_file_ofs, num_alignment_padding_bytes))
        return tmz_FALSE;

    cur_dst_file_ofs += num_alignment_padding_bytes;

    local_dir_header_ofs = cur_dst_file_ofs;
    if (pZip->m_file_offset_alignment)
    {
        tmz_ASSERT((local_dir_header_ofs & (pZip->m_file_offset_alignment - 1)) == 0);
    }

    /* The original zip's local header+ext block doesn't change, even with zip64, so we can just copy it over to the dest zip */
    if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_dst_file_ofs, pLocal_header, tmz_ZIP_LOCAL_DIR_HEADER_SIZE) != tmz_ZIP_LOCAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

    cur_dst_file_ofs += tmz_ZIP_LOCAL_DIR_HEADER_SIZE;

    /* Copy over the source archive bytes to the dest archive, also ensure we have enough buf space to handle optional data descriptor */
    if (NULL == (pBuf = pZip->m_pAlloc(pZip->m_pAlloc_opaque, 1, (size_t)tmz_MAX(32U, tmz_MIN((tmz_uint64)tmz_ZIP_MAX_IO_BUF_SIZE, src_archive_bytes_remaining)))))
        return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

    while (src_archive_bytes_remaining)
    {
        n = (tmz_uint)tmz_MIN((tmz_uint64)tmz_ZIP_MAX_IO_BUF_SIZE, src_archive_bytes_remaining);
        if (pSource_zip->m_pRead(pSource_zip->m_pIO_opaque, cur_src_file_ofs, pBuf, n) != n)
        {
            pZip->m_pFree(pZip->m_pAlloc_opaque, pBuf);
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
        }
        cur_src_file_ofs += n;

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_dst_file_ofs, pBuf, n) != n)
        {
            pZip->m_pFree(pZip->m_pAlloc_opaque, pBuf);
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
        }
        cur_dst_file_ofs += n;

        src_archive_bytes_remaining -= n;
    }

    /* Now deal with the optional data descriptor */
    bit_flags = tmz_READ_LE16(pLocal_header + tmz_ZIP_LDH_BIT_FLAG_OFS);
    if (bit_flags & 8)
    {
        /* Copy data descriptor */
        if ((pSource_zip->m_pState->m_zip64) || (found_zip64_ext_data_in_ldir))
        {
            /* src is zip64, dest must be zip64 */

            /* name			uint32_t's */
            /* id				1 (optional in zip64?) */
            /* crc			1 */
            /* comp_size	2 */
            /* uncomp_size 2 */
            if (pSource_zip->m_pRead(pSource_zip->m_pIO_opaque, cur_src_file_ofs, pBuf, (sizeof(tmz_uint32) * 6)) != (sizeof(tmz_uint32) * 6))
            {
                pZip->m_pFree(pZip->m_pAlloc_opaque, pBuf);
                return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
            }

            n = sizeof(tmz_uint32) * ((tmz_READ_LE32(pBuf) == tmz_ZIP_DATA_DESCRIPTOR_ID) ? 6 : 5);
        }
        else
        {
            /* src is NOT zip64 */
            tmz_bool has_id;

            if (pSource_zip->m_pRead(pSource_zip->m_pIO_opaque, cur_src_file_ofs, pBuf, sizeof(tmz_uint32) * 4) != sizeof(tmz_uint32) * 4)
            {
                pZip->m_pFree(pZip->m_pAlloc_opaque, pBuf);
                return tmz_zip_set_error(pZip, tmz_ZIP_FILE_READ_FAILED);
            }

            has_id = (tmz_READ_LE32(pBuf) == tmz_ZIP_DATA_DESCRIPTOR_ID);

            if (pZip->m_pState->m_zip64)
            {
                /* dest is zip64, so upgrade the data descriptor */
                const tmz_uint32 *pSrc_descriptor = (const tmz_uint32 *)((const tmz_uint8 *)pBuf + (has_id ? sizeof(tmz_uint32) : 0));
                const tmz_uint32 src_crc32 = pSrc_descriptor[0];
                const tmz_uint64 src_comp_size = pSrc_descriptor[1];
                const tmz_uint64 src_uncomp_size = pSrc_descriptor[2];

                tmz_write_le32((tmz_uint8 *)pBuf, tmz_ZIP_DATA_DESCRIPTOR_ID);
                tmz_write_le32((tmz_uint8 *)pBuf + sizeof(tmz_uint32) * 1, src_crc32);
                tmz_write_le64((tmz_uint8 *)pBuf + sizeof(tmz_uint32) * 2, src_comp_size);
                tmz_write_le64((tmz_uint8 *)pBuf + sizeof(tmz_uint32) * 4, src_uncomp_size);

                n = sizeof(tmz_uint32) * 6;
            }
            else
            {
                /* dest is NOT zip64, just copy it as-is */
                n = sizeof(tmz_uint32) * (has_id ? 4 : 3);
            }
        }

        if (pZip->m_pWrite(pZip->m_pIO_opaque, cur_dst_file_ofs, pBuf, n) != n)
        {
            pZip->m_pFree(pZip->m_pAlloc_opaque, pBuf);
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);
        }

        cur_src_file_ofs += n;
        cur_dst_file_ofs += n;
    }
    pZip->m_pFree(pZip->m_pAlloc_opaque, pBuf);

    /* Finally, add the new central dir header */
    orig_central_dir_size = pState->m_central_dir.m_size;

    memcpy(new_central_header, pSrc_central_header, tmz_ZIP_CENTRAL_DIR_HEADER_SIZE);

    if (pState->m_zip64)
    {
        /* This is the painful part: We need to write a new central dir header + ext block with updated zip64 fields, and ensure the old fields (if any) are not included. */
        const tmz_uint8 *pSrc_ext = pSrc_central_header + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + src_filename_len;
        tmz_zip_array new_ext_block;

        tmz_zip_array_init(&new_ext_block, sizeof(tmz_uint8));

        tmz_WRITE_LE32(new_central_header + tmz_ZIP_CDH_COMPRESSED_SIZE_OFS, tmz_UINT32_MAX);
        tmz_WRITE_LE32(new_central_header + tmz_ZIP_CDH_DECOMPRESSED_SIZE_OFS, tmz_UINT32_MAX);
        tmz_WRITE_LE32(new_central_header + tmz_ZIP_CDH_LOCAL_HEADER_OFS, tmz_UINT32_MAX);

        if (!tmz_zip_writer_update_zip64_extension_block(&new_ext_block, pZip, pSrc_ext, src_ext_len, &src_file_stat.m_comp_size, &src_file_stat.m_uncomp_size, &local_dir_header_ofs, NULL))
        {
            tmz_zip_array_clear(pZip, &new_ext_block);
            return tmz_FALSE;
        }

        tmz_WRITE_LE16(new_central_header + tmz_ZIP_CDH_EXTRA_LEN_OFS, new_ext_block.m_size);

        if (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, new_central_header, tmz_ZIP_CENTRAL_DIR_HEADER_SIZE))
        {
            tmz_zip_array_clear(pZip, &new_ext_block);
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }

        if (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, pSrc_central_header + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE, src_filename_len))
        {
            tmz_zip_array_clear(pZip, &new_ext_block);
            tmz_zip_array_resize(pZip, &pState->m_central_dir, orig_central_dir_size, tmz_FALSE);
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }

        if (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, new_ext_block.m_p, new_ext_block.m_size))
        {
            tmz_zip_array_clear(pZip, &new_ext_block);
            tmz_zip_array_resize(pZip, &pState->m_central_dir, orig_central_dir_size, tmz_FALSE);
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }

        if (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, pSrc_central_header + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE + src_filename_len + src_ext_len, src_comment_len))
        {
            tmz_zip_array_clear(pZip, &new_ext_block);
            tmz_zip_array_resize(pZip, &pState->m_central_dir, orig_central_dir_size, tmz_FALSE);
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }

        tmz_zip_array_clear(pZip, &new_ext_block);
    }
    else
    {
        /* sanity checks */
        if (cur_dst_file_ofs > tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);

        if (local_dir_header_ofs >= tmz_UINT32_MAX)
            return tmz_zip_set_error(pZip, tmz_ZIP_ARCHIVE_TOO_LARGE);

        tmz_WRITE_LE32(new_central_header + tmz_ZIP_CDH_LOCAL_HEADER_OFS, local_dir_header_ofs);

        if (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, new_central_header, tmz_ZIP_CENTRAL_DIR_HEADER_SIZE))
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);

        if (!tmz_zip_array_push_back(pZip, &pState->m_central_dir, pSrc_central_header + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE, src_central_dir_following_data_size))
        {
            tmz_zip_array_resize(pZip, &pState->m_central_dir, orig_central_dir_size, tmz_FALSE);
            return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
        }
    }

    /* This shouldn't trigger unless we screwed up during the initial sanity checks */
    if (pState->m_central_dir.m_size >= tmz_UINT32_MAX)
    {
        /* TODO: Support central dirs >= 32-bits in size */
        tmz_zip_array_resize(pZip, &pState->m_central_dir, orig_central_dir_size, tmz_FALSE);
        return tmz_zip_set_error(pZip, tmz_ZIP_UNSUPPORTED_CDIR_SIZE);
    }

    n = (tmz_uint32)orig_central_dir_size;
    if (!tmz_zip_array_push_back(pZip, &pState->m_central_dir_offsets, &n, 1))
    {
        tmz_zip_array_resize(pZip, &pState->m_central_dir, orig_central_dir_size, tmz_FALSE);
        return tmz_zip_set_error(pZip, tmz_ZIP_ALLOC_FAILED);
    }

    pZip->m_total_files++;
    pZip->m_archive_size = cur_dst_file_ofs;

    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_finalize_archive(tmz_zip_archive *pZip)
{
    tmz_zip_internal_state *pState;
    tmz_uint64 central_dir_ofs, central_dir_size;
    tmz_uint8 hdr[256];

    if ((!pZip) || (!pZip->m_pState) || (pZip->m_zip_mode != tmz_ZIP_MODE_WRITING))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    pState = pZip->m_pState;

    if (pState->m_zip64)
    {
        if ((pZip->m_total_files > tmz_UINT32_MAX) || (pState->m_central_dir.m_size >= tmz_UINT32_MAX))
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);
    }
    else
    {
        if ((pZip->m_total_files > tmz_UINT16_MAX) || ((pZip->m_archive_size + pState->m_central_dir.m_size + tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE) > tmz_UINT32_MAX))
            return tmz_zip_set_error(pZip, tmz_ZIP_TOO_MANY_FILES);
    }

    central_dir_ofs = 0;
    central_dir_size = 0;
    if (pZip->m_total_files)
    {
        /* Write central directory */
        central_dir_ofs = pZip->m_archive_size;
        central_dir_size = pState->m_central_dir.m_size;
        pZip->m_central_directory_file_ofs = central_dir_ofs;
        if (pZip->m_pWrite(pZip->m_pIO_opaque, central_dir_ofs, pState->m_central_dir.m_p, (size_t)central_dir_size) != central_dir_size)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        pZip->m_archive_size += central_dir_size;
    }

    if (pState->m_zip64)
    {
        /* Write zip64 end of central directory header */
        tmz_uint64 rel_ofs_to_zip64_ecdr = pZip->m_archive_size;

        tmz_CLEAR_OBJ(hdr);
        tmz_WRITE_LE32(hdr + tmz_ZIP64_ECDH_SIG_OFS, tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIG);
        tmz_WRITE_LE64(hdr + tmz_ZIP64_ECDH_SIZE_OF_RECORD_OFS, tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE - sizeof(tmz_uint32) - sizeof(tmz_uint64));
        tmz_WRITE_LE16(hdr + tmz_ZIP64_ECDH_VERSION_MADE_BY_OFS, 0x031E); /* TODO: always Unix */
        tmz_WRITE_LE16(hdr + tmz_ZIP64_ECDH_VERSION_NEEDED_OFS, 0x002D);
        tmz_WRITE_LE64(hdr + tmz_ZIP64_ECDH_CDIR_NUM_ENTRIES_ON_DISK_OFS, pZip->m_total_files);
        tmz_WRITE_LE64(hdr + tmz_ZIP64_ECDH_CDIR_TOTAL_ENTRIES_OFS, pZip->m_total_files);
        tmz_WRITE_LE64(hdr + tmz_ZIP64_ECDH_CDIR_SIZE_OFS, central_dir_size);
        tmz_WRITE_LE64(hdr + tmz_ZIP64_ECDH_CDIR_OFS_OFS, central_dir_ofs);
        if (pZip->m_pWrite(pZip->m_pIO_opaque, pZip->m_archive_size, hdr, tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE) != tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        pZip->m_archive_size += tmz_ZIP64_END_OF_CENTRAL_DIR_HEADER_SIZE;

        /* Write zip64 end of central directory locator */
        tmz_CLEAR_OBJ(hdr);
        tmz_WRITE_LE32(hdr + tmz_ZIP64_ECDL_SIG_OFS, tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIG);
        tmz_WRITE_LE64(hdr + tmz_ZIP64_ECDL_REL_OFS_TO_ZIP64_ECDR_OFS, rel_ofs_to_zip64_ecdr);
        tmz_WRITE_LE32(hdr + tmz_ZIP64_ECDL_TOTAL_NUMBER_OF_DISKS_OFS, 1);
        if (pZip->m_pWrite(pZip->m_pIO_opaque, pZip->m_archive_size, hdr, tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE) != tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE)
            return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

        pZip->m_archive_size += tmz_ZIP64_END_OF_CENTRAL_DIR_LOCATOR_SIZE;
    }

    /* Write end of central directory record */
    tmz_CLEAR_OBJ(hdr);
    tmz_WRITE_LE32(hdr + tmz_ZIP_ECDH_SIG_OFS, tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIG);
    tmz_WRITE_LE16(hdr + tmz_ZIP_ECDH_CDIR_NUM_ENTRIES_ON_DISK_OFS, tmz_MIN(tmz_UINT16_MAX, pZip->m_total_files));
    tmz_WRITE_LE16(hdr + tmz_ZIP_ECDH_CDIR_TOTAL_ENTRIES_OFS, tmz_MIN(tmz_UINT16_MAX, pZip->m_total_files));
    tmz_WRITE_LE32(hdr + tmz_ZIP_ECDH_CDIR_SIZE_OFS, tmz_MIN(tmz_UINT32_MAX, central_dir_size));
    tmz_WRITE_LE32(hdr + tmz_ZIP_ECDH_CDIR_OFS_OFS, tmz_MIN(tmz_UINT32_MAX, central_dir_ofs));

    if (pZip->m_pWrite(pZip->m_pIO_opaque, pZip->m_archive_size, hdr, tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE) != tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE)
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_WRITE_FAILED);

#ifndef tminiz_NO_STDIO
    if ((pState->m_pFile) && (tmz_FFLUSH(pState->m_pFile) == EOF))
        return tmz_zip_set_error(pZip, tmz_ZIP_FILE_CLOSE_FAILED);
#endif /* #ifndef tminiz_NO_STDIO */

    pZip->m_archive_size += tmz_ZIP_END_OF_CENTRAL_DIR_HEADER_SIZE;

    pZip->m_zip_mode = tmz_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED;
    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_finalize_heap_archive(tmz_zip_archive *pZip, void **ppBuf, size_t *pSize)
{
    if ((!ppBuf) || (!pSize))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    *ppBuf = NULL;
    *pSize = 0;

    if ((!pZip) || (!pZip->m_pState))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (pZip->m_pWrite != tmz_zip_heap_write_func)
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    if (!tmz_zip_writer_finalize_archive(pZip))
        return tmz_FALSE;

    *ppBuf = pZip->m_pState->m_pMem;
    *pSize = pZip->m_pState->m_mem_size;
    pZip->m_pState->m_pMem = NULL;
    pZip->m_pState->m_mem_size = pZip->m_pState->m_mem_capacity = 0;

    return tmz_TRUE;
}

tmz_bool tmz_zip_writer_end(tmz_zip_archive *pZip)
{
    return tmz_zip_writer_end_internal(pZip, tmz_TRUE);
}

#ifndef tminiz_NO_STDIO
tmz_bool tmz_zip_add_mem_to_archive_file_in_place(const char *pZip_filename, const char *pArchive_name, const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags)
{
    return tmz_zip_add_mem_to_archive_file_in_place_v2(pZip_filename, pArchive_name, pBuf, buf_size, pComment, comment_size, level_and_flags, NULL);
}

tmz_bool tmz_zip_add_mem_to_archive_file_in_place_v2(const char *pZip_filename, const char *pArchive_name, const void *pBuf, size_t buf_size, const void *pComment, tmz_uint16 comment_size, tmz_uint level_and_flags, tmz_zip_error *pErr)
{
    tmz_bool status, created_new_archive = tmz_FALSE;
    tmz_zip_archive zip_archive;
    struct tmz_FILE_STAT_STRUCT file_stat;
    tmz_zip_error actual_err = tmz_ZIP_NO_ERROR;

    tmz_zip_zero_struct(&zip_archive);
    if ((int)level_and_flags < 0)
        level_and_flags = tmz_DEFAULT_LEVEL;

    if ((!pZip_filename) || (!pArchive_name) || ((buf_size) && (!pBuf)) || ((comment_size) && (!pComment)) || ((level_and_flags & 0xF) > tmz_UBER_COMPRESSION))
    {
        if (pErr)
            *pErr = tmz_ZIP_INVALID_PARAMETER;
        return tmz_FALSE;
    }

    if (!tmz_zip_writer_validate_archive_name(pArchive_name))
    {
        if (pErr)
            *pErr = tmz_ZIP_INVALID_FILENAME;
        return tmz_FALSE;
    }

    /* Important: The regular non-64 bit version of stat() can fail here if the file is very large, which could cause the archive to be overwritten. */
    /* So be sure to compile with _LARGEFILE64_SOURCE 1 */
    if (tmz_FILE_STAT(pZip_filename, &file_stat) != 0)
    {
        /* Create a new archive. */
        if (!tmz_zip_writer_init_file_v2(&zip_archive, pZip_filename, 0, level_and_flags))
        {
            if (pErr)
                *pErr = zip_archive.m_last_error;
            return tmz_FALSE;
        }

        created_new_archive = tmz_TRUE;
    }
    else
    {
        /* Append to an existing archive. */
        if (!tmz_zip_reader_init_file_v2(&zip_archive, pZip_filename, level_and_flags | tmz_ZIP_FLAG_DO_NOT_SORT_CENTRAL_DIRECTORY, 0, 0))
        {
            if (pErr)
                *pErr = zip_archive.m_last_error;
            return tmz_FALSE;
        }

        if (!tmz_zip_writer_init_from_reader_v2(&zip_archive, pZip_filename, level_and_flags))
        {
            if (pErr)
                *pErr = zip_archive.m_last_error;

            tmz_zip_reader_end_internal(&zip_archive, tmz_FALSE);

            return tmz_FALSE;
        }
    }

    status = tmz_zip_writer_add_mem_ex(&zip_archive, pArchive_name, pBuf, buf_size, pComment, comment_size, level_and_flags, 0, 0);
    actual_err = zip_archive.m_last_error;

    /* Always finalize, even if adding failed for some reason, so we have a valid central directory. (This may not always succeed, but we can try.) */
    if (!tmz_zip_writer_finalize_archive(&zip_archive))
    {
        if (!actual_err)
            actual_err = zip_archive.m_last_error;

        status = tmz_FALSE;
    }

    if (!tmz_zip_writer_end_internal(&zip_archive, status))
    {
        if (!actual_err)
            actual_err = zip_archive.m_last_error;

        status = tmz_FALSE;
    }

    if ((!status) && (created_new_archive))
    {
        /* It's a new archive and something went wrong, so just delete it. */
        int ignoredStatus = tmz_DELETE_FILE(pZip_filename);
        (void)ignoredStatus;
    }

    if (pErr)
        *pErr = actual_err;

    return status;
}

void *tmz_zip_extract_archive_file_to_heap_v2(const char *pZip_filename, const char *pArchive_name, const char *pComment, size_t *pSize, tmz_uint flags, tmz_zip_error *pErr)
{
    tmz_uint32 file_index;
    tmz_zip_archive zip_archive;
    void *p = NULL;

    if (pSize)
        *pSize = 0;

    if ((!pZip_filename) || (!pArchive_name))
    {
        if (pErr)
            *pErr = tmz_ZIP_INVALID_PARAMETER;

        return NULL;
    }

    tmz_zip_zero_struct(&zip_archive);
    if (!tmz_zip_reader_init_file_v2(&zip_archive, pZip_filename, flags | tmz_ZIP_FLAG_DO_NOT_SORT_CENTRAL_DIRECTORY, 0, 0))
    {
        if (pErr)
            *pErr = zip_archive.m_last_error;

        return NULL;
    }

    if (tmz_zip_reader_locate_file_v2(&zip_archive, pArchive_name, pComment, flags, &file_index))
    {
        p = tmz_zip_reader_extract_to_heap(&zip_archive, file_index, pSize, flags);
    }

    tmz_zip_reader_end_internal(&zip_archive, p != NULL);

    if (pErr)
        *pErr = zip_archive.m_last_error;

    return p;
}

void *tmz_zip_extract_archive_file_to_heap(const char *pZip_filename, const char *pArchive_name, size_t *pSize, tmz_uint flags)
{
    return tmz_zip_extract_archive_file_to_heap_v2(pZip_filename, pArchive_name, NULL, pSize, flags, NULL);
}

#endif /* #ifndef tminiz_NO_STDIO */

#endif /* #ifndef tminiz_NO_ARCHIVE_WRITING_APIS */

/* ------------------- Misc utils */

tmz_zip_mode tmz_zip_get_mode(tmz_zip_archive *pZip)
{
    return pZip ? pZip->m_zip_mode : tmz_ZIP_MODE_INVALID;
}

tmz_zip_type tmz_zip_get_type(tmz_zip_archive *pZip)
{
    return pZip ? pZip->m_zip_type : tmz_ZIP_TYPE_INVALID;
}

tmz_zip_error tmz_zip_set_last_error(tmz_zip_archive *pZip, tmz_zip_error err_num)
{
    tmz_zip_error prev_err;

    if (!pZip)
        return tmz_ZIP_INVALID_PARAMETER;

    prev_err = pZip->m_last_error;

    pZip->m_last_error = err_num;
    return prev_err;
}

tmz_zip_error tmz_zip_peek_last_error(tmz_zip_archive *pZip)
{
    if (!pZip)
        return tmz_ZIP_INVALID_PARAMETER;

    return pZip->m_last_error;
}

tmz_zip_error tmz_zip_clear_last_error(tmz_zip_archive *pZip)
{
    return tmz_zip_set_last_error(pZip, tmz_ZIP_NO_ERROR);
}

tmz_zip_error tmz_zip_get_last_error(tmz_zip_archive *pZip)
{
    tmz_zip_error prev_err;

    if (!pZip)
        return tmz_ZIP_INVALID_PARAMETER;

    prev_err = pZip->m_last_error;

    pZip->m_last_error = tmz_ZIP_NO_ERROR;
    return prev_err;
}

const char *tmz_zip_get_error_string(tmz_zip_error tmz_err)
{
    switch (tmz_err)
    {
        case tmz_ZIP_NO_ERROR:
            return "no error";
        case tmz_ZIP_UNDEFINED_ERROR:
            return "undefined error";
        case tmz_ZIP_TOO_MANY_FILES:
            return "too many files";
        case tmz_ZIP_FILE_TOO_LARGE:
            return "file too large";
        case tmz_ZIP_UNSUPPORTED_METHOD:
            return "unsupported method";
        case tmz_ZIP_UNSUPPORTED_ENCRYPTION:
            return "unsupported encryption";
        case tmz_ZIP_UNSUPPORTED_FEATURE:
            return "unsupported feature";
        case tmz_ZIP_FAILED_FINDING_CENTRAL_DIR:
            return "failed finding central directory";
        case tmz_ZIP_NOT_AN_ARCHIVE:
            return "not a ZIP archive";
        case tmz_ZIP_INVALID_HEADER_OR_CORRUPTED:
            return "invalid header or archive is corrupted";
        case tmz_ZIP_UNSUPPORTED_MULTIDISK:
            return "unsupported multidisk archive";
        case tmz_ZIP_DECOMPRESSION_FAILED:
            return "decompression failed or archive is corrupted";
        case tmz_ZIP_COMPRESSION_FAILED:
            return "compression failed";
        case tmz_ZIP_UNEXPECTED_DECOMPRESSED_SIZE:
            return "unexpected decompressed size";
        case tmz_ZIP_CRC_CHECK_FAILED:
            return "CRC-32 check failed";
        case tmz_ZIP_UNSUPPORTED_CDIR_SIZE:
            return "unsupported central directory size";
        case tmz_ZIP_ALLOC_FAILED:
            return "allocation failed";
        case tmz_ZIP_FILE_OPEN_FAILED:
            return "file open failed";
        case tmz_ZIP_FILE_CREATE_FAILED:
            return "file create failed";
        case tmz_ZIP_FILE_WRITE_FAILED:
            return "file write failed";
        case tmz_ZIP_FILE_READ_FAILED:
            return "file read failed";
        case tmz_ZIP_FILE_CLOSE_FAILED:
            return "file close failed";
        case tmz_ZIP_FILE_SEEK_FAILED:
            return "file seek failed";
        case tmz_ZIP_FILE_STAT_FAILED:
            return "file stat failed";
        case tmz_ZIP_INVALID_PARAMETER:
            return "invalid parameter";
        case tmz_ZIP_INVALID_FILENAME:
            return "invalid filename";
        case tmz_ZIP_BUF_TOO_SMALL:
            return "buffer too small";
        case tmz_ZIP_INTERNAL_ERROR:
            return "internal error";
        case tmz_ZIP_FILE_NOT_FOUND:
            return "file not found";
        case tmz_ZIP_ARCHIVE_TOO_LARGE:
            return "archive is too large";
        case tmz_ZIP_VALIDATION_FAILED:
            return "validation failed";
        case tmz_ZIP_WRITE_CALLBACK_FAILED:
            return "write calledback failed";
        default:
            break;
    }

    return "unknown error";
}

/* Note: Just because the archive is not zip64 doesn't necessarily mean it doesn't have Zip64 extended information extra field, argh. */
tmz_bool tmz_zip_is_zip64(tmz_zip_archive *pZip)
{
    if ((!pZip) || (!pZip->m_pState))
        return tmz_FALSE;

    return pZip->m_pState->m_zip64;
}

size_t tmz_zip_get_central_dir_size(tmz_zip_archive *pZip)
{
    if ((!pZip) || (!pZip->m_pState))
        return 0;

    return pZip->m_pState->m_central_dir.m_size;
}

tmz_uint tmz_zip_reader_get_num_files(tmz_zip_archive *pZip)
{
    return pZip ? pZip->m_total_files : 0;
}

tmz_uint64 tmz_zip_get_archive_size(tmz_zip_archive *pZip)
{
    if (!pZip)
        return 0;
    return pZip->m_archive_size;
}

tmz_uint64 tmz_zip_get_archive_file_start_offset(tmz_zip_archive *pZip)
{
    if ((!pZip) || (!pZip->m_pState))
        return 0;
    return pZip->m_pState->m_file_archive_start_ofs;
}

tmz_FILE *tmz_zip_get_cfile(tmz_zip_archive *pZip)
{
    if ((!pZip) || (!pZip->m_pState))
        return 0;
    return pZip->m_pState->m_pFile;
}

size_t tmz_zip_read_archive_data(tmz_zip_archive *pZip, tmz_uint64 file_ofs, void *pBuf, size_t n)
{
    if ((!pZip) || (!pZip->m_pState) || (!pBuf) || (!pZip->m_pRead))
        return tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);

    return pZip->m_pRead(pZip->m_pIO_opaque, file_ofs, pBuf, n);
}

tmz_uint tmz_zip_reader_get_filename(tmz_zip_archive *pZip, tmz_uint file_index, char *pFilename, tmz_uint filename_buf_size)
{
    tmz_uint n;
    const tmz_uint8 *p = tmz_zip_get_cdh(pZip, file_index);
    if (!p)
    {
        if (filename_buf_size)
            pFilename[0] = '\0';
        tmz_zip_set_error(pZip, tmz_ZIP_INVALID_PARAMETER);
        return 0;
    }
    n = tmz_READ_LE16(p + tmz_ZIP_CDH_FILENAME_LEN_OFS);
    if (filename_buf_size)
    {
        n = tmz_MIN(n, filename_buf_size - 1);
        memcpy(pFilename, p + tmz_ZIP_CENTRAL_DIR_HEADER_SIZE, n);
        pFilename[n] = '\0';
    }
    return n + 1;
}

tmz_bool tmz_zip_reader_file_stat(tmz_zip_archive *pZip, tmz_uint file_index, tmz_zip_archive_file_stat *pStat)
{
    return tmz_zip_file_stat_internal(pZip, file_index, tmz_zip_get_cdh(pZip, file_index), pStat, NULL);
}

tmz_bool tmz_zip_end(tmz_zip_archive *pZip)
{
    if (!pZip)
        return tmz_FALSE;

    if (pZip->m_zip_mode == tmz_ZIP_MODE_READING)
        return tmz_zip_reader_end(pZip);
#ifndef tminiz_NO_ARCHIVE_WRITING_APIS
    else if ((pZip->m_zip_mode == tmz_ZIP_MODE_WRITING) || (pZip->m_zip_mode == tmz_ZIP_MODE_WRITING_HAS_BEEN_FINALIZED))
        return tmz_zip_writer_end(pZip);
#endif

    return tmz_FALSE;
}

#ifdef __cplusplus
}
#endif

#endif /*#ifndef tminiz_NO_ARCHIVE_APIS*/

