#ifndef _FILEEX_H____
#define _FILEEX_H____

#include <windows.h>
#include <tchar.h>
#include <string.h>
#include "winerr.h"

#ifdef _UNICODE
#define tstring			std::wstring
#else
#define tstring			std::string
#endif

	// disables the 
	// "C++ exception specification ignored except to indicate a function is not __declspec(nothrow)"
	// warning so it doesn't show up during compile. I have chosen to keep the statements there
	// to notify the user of this class what could be thrown from the function ( and in the off chance 
	// microsoft fixes this thing sometime )
#pragma warning ( disable : 4290 )

	// this defines how many bytes of the file is read/written in each
	// overlapped read/write operation
#define OVLFILE_DEFAULT_SEGSIZE		204800

	// defines the function pointer definition for a the callback that the
	// user can use with the read or write operations. This function will
	// effectivly be called after each read or write of OVLFILE_DEFAULT_SEGSIZE
	// bytes.
typedef BOOL ( CALLBACK* LPFN_LRGFILEOP_PROGCALLBACK )( DWORD dwWritten, DWORD dwTotalSize, LPVOID pParam );

	// this structure is an extended overlapped structure
	// used in the read and write large file functions
typedef struct _RDWROVERLAPPEDPLUS
{
	OVERLAPPED					ov;
	DWORD						dwTotalSizeToTransfer;
	DWORD						dwTotalSoFar;
	LPFN_LRGFILEOP_PROGCALLBACK	lpCallback;
	LPVOID						pParam;
	BOOL						bContinue;
	DWORD						dwError;
} RDWROVERLAPPEDPLUS, *LPRDWROVERLAPPEDPLUS;

#define FILEEX_ERR_NOERROR					0x00
#define FILEEX_ERR_INVALID_OPCLOSEDFILE		0x01
#define FILEEX_ERR_PARAM_OUTOFRANGE			0x02
#define FILEEX_ERR_OTHERIO_CURRENTLY_RUNNG	0x03
#define FILEEX_ERR_INVALIDPARAM				0x04

static const TCHAR* ERRORMESSAGES[] = 
{
	_T("The Operation Completed Successfully"),
	_T("Invalid Operation on Closed File"),
	_T("Parameter Supplied Is out of Range"),
	_T("Operation Failed. I/O Operations are Running"),
	_T("Parameter is Invalid")
};

//////////////////////////////////////////////////////////////////////
//	Desc:
//		This class defines an exception object that can be thrown from
//		various routines within the CFileEx class.  It is a custom 
//		exception object instead of inherting from CException or 
//		CAtlException to maintain it library independence.
//////////////////////////////////////////////////////////////////////
class CFileExException 
{
	public:
		CFileExException( DWORD dwErr, DWORD dwCustError )
		{
			m_dwCustError	= dwCustError;
			m_sCustMsg		= ERRORMESSAGES[ m_dwCustError ];
			m_winErr		= dwErr;
			m_sWinMsg		= m_winErr.GetFormattedMsg();
		}

		LPCTSTR GetCustomErrorMsg()	{	return m_sCustMsg.c_str(); 	}
		DWORD GetCustomError()		{	return m_dwCustError;		}
		LPCTSTR GetWinErrorMsg()	{	return m_sWinMsg.c_str();	}
		DWORD GetWinErrorCode()		{	return m_winErr;			}

	protected:
		CWinErr	m_winErr;
		DWORD	m_dwCustError;
		tstring	m_sCustMsg;
		tstring	m_sWinMsg;
};

//////////////////////////////////////////////////////////////
//	Desc:
//		This class defines a replacement for the MFC CFile
//		class which is suitable for most instances.  It extendes
//		functionality by providing methods for doing overlapped
//		I/O with a user defined callback function that reports
//		the progress of a file read/write.  It also provides
//		methods for doing normal "read/write full then return"
//		type operations.  Other standard file query calls can
//		be made to determine the status of the file. This
//		class has no dependency on MFC and can be used in any
//		type of project using C++.
//////////////////////////////////////////////////////////////
class CFileEx
{
	public:
		CFileEx();
		~CFileEx();
		BOOL		Open(	LPCTSTR lpFile, DWORD dwCreateDisp = OPEN_EXISTING, 
							DWORD dwAccess = GENERIC_READ | GENERIC_WRITE, DWORD dwShare = 0, 
							LPSECURITY_ATTRIBUTES lpSec = NULL ) throw( CFileExException );
		void		Close();
		DWORD		Read( BYTE* pBuffer, DWORD dwSize ) throw( CFileExException );
		DWORD		Write( BYTE* pBuffer, DWORD dwSize ) throw( CFileExException );
		DWORD		ReadOv( BYTE* pBuffer, DWORD dwSize, LPFN_LRGFILEOP_PROGCALLBACK lpCallback, LPVOID pParam, BOOL bUseMsgPump = TRUE ) throw( CFileExException );
		DWORD		WriteOv( BYTE* pBuffer, DWORD dwSize, LPFN_LRGFILEOP_PROGCALLBACK lpCallback, LPVOID pParam, BOOL bUseMsgPump = TRUE ) throw( CFileExException );
		BOOL		SetOvSegReadWriteSize( DWORD dwSegmentSize = OVLFILE_DEFAULT_SEGSIZE ) throw( CFileExException );
		DWORD		GetOvSegReadWriteSize();
		BOOL		AbortOverlappedOperation();
		BOOL		IsOpen();
		void		SetThrowErrors( BOOL bThrow = TRUE );
		ULONGLONG	GetFileSize() throw( CFileExException );
		ULONGLONG	Seek( LONGLONG lToMove, DWORD dwMoveFrom = FILE_BEGIN ) throw( CFileExException );
		ULONGLONG	SeekToBegin() throw( CFileExException );
		ULONGLONG	SeekToEnd() throw( CFileExException );
		void		Flush() throw( CFileExException );
		BOOL 		GetFileName( TCHAR* lpName, DWORD dwBufLen );
		BOOL		GetTimeLastAccessed( SYSTEMTIME& sys );
		BOOL		GetTimeLastModified( SYSTEMTIME& sys );
		BOOL		GetTimeCreated( SYSTEMTIME& sys );
				
	public:
		static BOOL GetTimeLastAccessed( LPCTSTR lpFile, SYSTEMTIME& sys );
		static BOOL GetTimeLastModified( LPCTSTR lpFile, SYSTEMTIME& sys );
		static BOOL GetTimeCreated( LPCTSTR lpFile, SYSTEMTIME& sys );
		
	public:
			// public data member. we can let people use 
			// the handle direct if they wish.
		HANDLE	m_hFile;
		
	protected:
			// these data memebers are for internal use only.
		HANDLE	m_hStop;		// event used to abort wait operations during overlapped reads/writes
		BOOL	m_bOpen;		// simple flag for open/closed status
		BOOL	m_bActiveOvIO;	// signals whether an IO operation is currently in progress.
		DWORD	m_dwSegSize;	// read/write segment size for overlapped operations with comp routines.
		BOOL	m_bThrowErrs;	// determines whether or not we should throw errors back to the user.
		BOOL	m_bShouldClose;	// tells the overlapped rountines to close the file after they come out of the loop.
		TCHAR	m_lpFile[ _MAX_PATH ];

	protected:
		DWORD DoFileOperationWithMsgPump( BOOL bWrite, BYTE* pBuffer, DWORD dwSize, 
										 LPFN_LRGFILEOP_PROGCALLBACK lpCallback, LPVOID pParam ) throw( CFileExException );
		DWORD DoFileOperation( BOOL bWrite, BYTE* pBuffer, DWORD dwSize, 
							  LPFN_LRGFILEOP_PROGCALLBACK lpCallback, LPVOID pParam ) throw( CFileExException );

		BOOL NextIoSegment( BOOL bWrite, RDWROVERLAPPEDPLUS& ov, BYTE* pBuffer, DWORD dwTtlSegs, DWORD dwCurSeg ) throw( CFileExException );
		void PumpMsgs();
		void IsOpenThrows() throw( CFileExException );
		void ThrowErr( DWORD dwCode, BOOL bIsCust ) throw( CFileExException );
		
		static void CALLBACK FileIoCompletionRoutine( DWORD dwErrorCode, DWORD dwNumTrans, LPOVERLAPPED lpOverlapped );
};


#endif //_FILEEX_H____