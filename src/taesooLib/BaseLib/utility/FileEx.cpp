#include "stdafx.h"
#include "FileEx.h"

#define POINTEROFFSET( ptr, offset ) ( ( void* )( ( DWORD_PTR )ptr + offset ) )

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		construction. Initializes data members. 
///////////////////////////////////////////////////////////////////////////////
CFileEx::CFileEx()
{
	m_hFile			= NULL;
	m_bOpen			= FALSE;
	m_hStop			= ::CreateEvent( NULL, TRUE, FALSE, NULL );
	m_dwSegSize		= OVLFILE_DEFAULT_SEGSIZE;
	m_bActiveOvIO	= FALSE;
	m_bThrowErrs	= TRUE;
	m_bShouldClose	= FALSE;

	ZeroMemory( m_lpFile, sizeof( TCHAR ) * _MAX_PATH );

	return;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Destructor.  Ensures that all of our operations are complete and the
//		file is closed gracefully before shutdown.
///////////////////////////////////////////////////////////////////////////////
CFileEx::~CFileEx()
{
	if( IsOpen() ) {
		Close();
	}

	::CloseHandle( m_hStop );
	m_hStop			= NULL;
	m_bActiveOvIO	= FALSE;

	return;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Sets the flag that determines whether or not we throw errors
//		back to the user. If TRUE the user must provides proper try {} catch
//		routines around functions that throw errors.
///////////////////////////////////////////////////////////////////////////////
void CFileEx::SetThrowErrors( BOOL bThrow )
{
	m_bThrowErrs = bThrow;
	return;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Generates a throws an error back to the user IF the throw errors 
//		flag is true.
///////////////////////////////////////////////////////////////////////////////
void CFileEx::ThrowErr( DWORD dwCode, BOOL bIsCust )
{
	if( m_bThrowErrs ) {
		if( bIsCust ) {
			throw CFileExException( 0, dwCode );
		}else{
			throw CFileExException( dwCode, 0 );
		}
	}

	return;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Checks to see if this file is already open and returns
//		the result to the caller.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::IsOpen()
{
	return m_bOpen;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Throws an exception if the current file is not open.
//		This is used internally when operations require the file be
//		opened before performing an operation.
///////////////////////////////////////////////////////////////////////////////
void CFileEx::IsOpenThrows()
{
	if( ! IsOpen() ) {
		ThrowErr( FILEEX_ERR_INVALID_OPCLOSEDFILE, TRUE );
	}
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Opens a standard win32 file using the same type of parameters as
//		the CreateFile API however, the template file is assumed to be null
//		and the flags are locked ( not selectable by user ) because me MUST
//		ensure the FILE_FLAG_OVERLAPPED flag is properly set.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::Open( LPCTSTR lpFile, DWORD dwCreateDisp, DWORD dwAccess, DWORD dwShare, LPSECURITY_ATTRIBUTES lpSec )
{
	if( IsOpen() ) {
		Close();
	}

	if( ( lpFile == NULL ) || ( lstrlen( lpFile ) > _MAX_PATH ) ) {
		return FALSE;
	}

//	m_hFile = ::CreateFile( lpFile, dwAccess, dwShare, lpSec, dwCreateDisp, FILE_FLAG_OVERLAPPED, NULL );
	m_hFile = ::CreateFile(lpFile, dwAccess, dwShare, lpSec, dwCreateDisp, 0, NULL );
			
	if( m_hFile == INVALID_HANDLE_VALUE ) {
			// file open failed.  Throw error back to the user with the
			// windows error code.
		ThrowErr( ::GetLastError(), FALSE );
		m_hFile = NULL;		
	}else{
		::_tcscpy( m_lpFile, lpFile );
		m_bOpen = TRUE;
	}

	return m_bOpen;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Shuts down all current IO and closes the file.  Calling this rountine
//		is necessary for overlapped IO operations because unpredicatable
//		behavior will occurr if CloseHandle is used directly on m_hFile while
//		an Overlapped I/O operation is in progress.
///////////////////////////////////////////////////////////////////////////////
void CFileEx::Close()
{
	if( m_bActiveOvIO ) {
			// just set the stop event.  The IO routine will recall this function
			// on its way out to do the proper close.
		::SetEvent( m_hStop );
		m_bShouldClose = TRUE;
	}else{
		::CloseHandle( m_hFile );
		m_hFile			= NULL;
		m_bOpen			= FALSE;
		m_bShouldClose	= FALSE;
	}

	return;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Reads raw data in from the file if this file is opened.  While
//		the file is always opened as available for overlapped I/O this function
//		proceeds to read the function and just wait for the result.  Mimics
//		behaviour of CFile::Read
///////////////////////////////////////////////////////////////////////////////
DWORD CFileEx::Read( BYTE* pBuffer, DWORD dwSize )
{
	DWORD		dwRead	= 0;
	BOOL		bRet	= FALSE;
	OVERLAPPED	ov;

	ZeroMemory( &ov, sizeof( OVERLAPPED ) );
	IsOpenThrows();
	
	if( pBuffer ) {
		//bRet = ::ReadFile( m_hFile, pBuffer, dwSize, &dwRead, &ov );
		bRet = ::ReadFile( m_hFile, pBuffer, dwSize, &dwRead, 0);
		if( ! bRet && ::GetLastError() == ERROR_IO_PENDING ) {
			::GetOverlappedResult( m_hFile, &ov, &dwRead, TRUE );
		}else{
				// some API call failed. To Determine reason throw error
				// back to user with the API call's error from GetLastError
			if( ! bRet ) {
				dwRead = 0;
				ThrowErr( ::GetLastError(), FALSE );
			}
		}
	}else{
		ThrowErr( FILEEX_ERR_INVALIDPARAM, TRUE );
	}

	return dwRead;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Writes raw data to the file if this file is opened.  While
//		the file is always opened as available for overlapped I/O this function
//		proceeds to read the function and just wait for the result.  Mimics
//		behaviour of CFile::Write
///////////////////////////////////////////////////////////////////////////////
DWORD CFileEx::Write( BYTE* pBuffer, DWORD dwSize )
{
	DWORD		dwWrote	= 0;
	BOOL		bRet	= FALSE;
	DWORD		dwErr	= 0;
	OVERLAPPED	ov;
	
	ZeroMemory( &ov, sizeof( OVERLAPPED ) );
	
	IsOpenThrows();

	if( pBuffer ) {
		//bRet	= ::WriteFile( m_hFile, pBuffer, dwSize, &dwWrote, &ov );
		bRet	= ::WriteFile( m_hFile, pBuffer, dwSize, &dwWrote, NULL );
		dwErr	= ::GetLastError();
		if( ! bRet && ( dwErr == ERROR_IO_PENDING ) ) {
			::GetOverlappedResult( m_hFile, &ov, &dwWrote, TRUE );
		}else{
				// some API call failed. To Determine reason throw error
				// back to user with the API call's error from GetLastError
			if( ! bRet ) {
				dwWrote = 0;
				ThrowErr( ::GetLastError(), FALSE );
			}
		}
	}else{
		ThrowErr( FILEEX_ERR_INVALIDPARAM, TRUE );
	}

	return dwWrote;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		This function will read data in from a file and call a user defined
//		callback function after each segment is read. The segment size
//		can be set with SetOvSegReadWriteSize.
///////////////////////////////////////////////////////////////////////////////
DWORD CFileEx::ReadOv( BYTE* pBuffer, DWORD dwSize, LPFN_LRGFILEOP_PROGCALLBACK lpCallback, LPVOID pParam, BOOL bUseMsgPump )
{
	DWORD dwRead = 0;
	
	IsOpenThrows();

	if( pBuffer ) {
		m_bActiveOvIO = TRUE;
		if( bUseMsgPump ) {
			dwRead = DoFileOperationWithMsgPump( FALSE, pBuffer, dwSize, lpCallback, pParam );
		}else{
			dwRead = DoFileOperation( FALSE, pBuffer, dwSize, lpCallback, pParam );
		}
		m_bActiveOvIO = FALSE;

		if( m_bShouldClose ) {
				// this close rountine will now actually shut the file because 
				// the m_bActiveOvIo flag is false.
			Close();
		}

	}else{
		ThrowErr( FILEEX_ERR_INVALIDPARAM, TRUE );
	}

	return dwRead;
}


///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		This function will write data to from a file and call a user defined
//		callback function after each segment is written. The segment size
//		can be set with SetOvSegReadWriteSize.
///////////////////////////////////////////////////////////////////////////////
DWORD CFileEx::WriteOv( BYTE* pBuffer, DWORD dwSize, LPFN_LRGFILEOP_PROGCALLBACK lpCallback, LPVOID pParam, BOOL bUseMsgPump )
{
	DWORD dwWrote = 0;
	IsOpenThrows();

	if( pBuffer ) {
		m_bActiveOvIO = TRUE;
		if( bUseMsgPump ) {
			dwWrote = DoFileOperationWithMsgPump( TRUE, pBuffer, dwSize, lpCallback, pParam );
		}else{
			dwWrote = DoFileOperation( TRUE, pBuffer, dwSize, lpCallback, pParam );
		}
		m_bActiveOvIO = FALSE;

		if( m_bShouldClose ) {
				// this close rountine will now actually shut the file because 
				// the m_bActiveOvIo flag is false.
			Close();
		}

	}else{
		ThrowErr( FILEEX_ERR_INVALIDPARAM, TRUE );
	}

	return dwWrote;
}


///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Set the size of the segments of data that are read or written to/from
//		the file during overlapped reads and writes.  This is how much data
//		is transferred in between each call to the user defined callback.
//		This can not be changed when overlapped I/O operations are in progress.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::SetOvSegReadWriteSize( DWORD dwSegmentSize  )
{
	BOOL bSet = FALSE;
	if( dwSegmentSize <= 0 ) {
		ThrowErr( FILEEX_ERR_PARAM_OUTOFRANGE, 0 );
	}
	
	if( ! m_bActiveOvIO ) {
		bSet		= TRUE;
		m_dwSegSize = dwSegmentSize;
	}else{
		ThrowErr( FILEEX_ERR_OTHERIO_CURRENTLY_RUNNG, 0 );
	}

	return bSet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		return segment size.  see SetOvSegReadWriteSize() 
//		comment for description.
///////////////////////////////////////////////////////////////////////////////
DWORD CFileEx::GetOvSegReadWriteSize()
{
	return m_dwSegSize;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Attempts to stop an active overlapped I/O operation by signaling
//		the stop event.  This action stops the operation but does not
//		close the file.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::AbortOverlappedOperation()
{
	BOOL bRet = FALSE;
	if( m_bActiveOvIO ) {
		::SetEvent( m_hStop );
		bRet = TRUE;
	}
	return bRet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Returns the size of the file that this object is wrapping. If the
//		file is closed 0 is returned.
///////////////////////////////////////////////////////////////////////////////
ULONGLONG CFileEx::GetFileSize()
{
	ULONGLONG ulSize = 0;
	LARGE_INTEGER lInt;
	
	IsOpenThrows();
    
	if( ::GetFileSizeEx( m_hFile, &lInt ) ) {
		ulSize = lInt.QuadPart;
	}else{
		ThrowErr( ::GetLastError(), FALSE );
	}

	return ulSize;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Attempts to move the file pointer to a given location in the file
//		by shifting from a given position a requested number of bytes.
///////////////////////////////////////////////////////////////////////////////
ULONGLONG CFileEx::Seek( LONGLONG lToMove, DWORD dwMoveFrom  )
{
	ULONGLONG		ulPos = 0;
	LARGE_INTEGER	lIn;
	LARGE_INTEGER	lOut;
	
	IsOpenThrows();

	if( ( lToMove <= ( LONGLONG )GetFileSize() ) ) {
		if( ( dwMoveFrom == FILE_BEGIN ) ||
			( dwMoveFrom == FILE_END ) ||
			( dwMoveFrom == FILE_CURRENT ) )
		{
			lIn.QuadPart = lToMove;

			if( ::SetFilePointerEx( m_hFile, lIn, &lOut, dwMoveFrom ) ) {
				ulPos = lOut.QuadPart;
			}else{
				ThrowErr( ::GetLastError(), FALSE );
			}
			
		}else{
			ThrowErr( FILEEX_ERR_PARAM_OUTOFRANGE, TRUE );
		}
	}	

	return ulPos;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		uses the seek function to move the file pointer to the 
//		beginning of the file.
///////////////////////////////////////////////////////////////////////////////
ULONGLONG CFileEx::SeekToBegin()
{
	return Seek( 0, FILE_BEGIN );
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		uses the seek function to move the file pointer to the 
//		end of the file.
///////////////////////////////////////////////////////////////////////////////
ULONGLONG CFileEx::SeekToEnd()
{
	return Seek( 0, FILE_END );
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Causes all unwritten data to be flushed out to the file
//		on disk.
///////////////////////////////////////////////////////////////////////////////
void CFileEx::Flush()
{
	IsOpenThrows();
    if( ! ::FlushFileBuffers( m_hFile ) ) {
		ThrowErr( ::GetLastError(), FALSE );
	}
	
	return;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Returns the name of the current file. The name returned is the full
//		path name of the file.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::GetFileName( TCHAR* lpName, DWORD dwBufLen )
{
	BOOL bRet = FALSE;
	

	if( IsOpen() && lpName && ( dwBufLen > 0 ) ){
		//FIXTHIS:
	}

	return bRet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Returns the time this file was last accessed.  This is just a shortcut
//		for the static version of this function and only works if this file
//		is open.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::GetTimeLastAccessed( SYSTEMTIME& sys )
{
	BOOL bRet = FALSE;
	if( IsOpen() ) {
		bRet = CFileEx::GetTimeLastAccessed( m_lpFile, sys );
	}
	return bRet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Returns the time this file was last modified.  This is just a shortcut
//		for the static version of this function and only works if this file
//		is open.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::GetTimeLastModified( SYSTEMTIME& sys )
{
	BOOL bRet = FALSE;
	if( IsOpen() ) {
		bRet = CFileEx::GetTimeLastModified( m_lpFile, sys );
	}
	return bRet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Returns the time this file was created.  This is just a shortcut
//		for the static version of this function and only works if this file
//		is open.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::GetTimeCreated( SYSTEMTIME& sys )
{
	BOOL bRet = FALSE;
	if( IsOpen() ) {
		bRet = CFileEx::GetTimeCreated( m_lpFile, sys );
	}
	return bRet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Returns the time this file was last accessed. 
///////////////////////////////////////////////////////////////////////////////
/*static*/
BOOL CFileEx::GetTimeLastAccessed( LPCTSTR lpFile, SYSTEMTIME& sys )
{
	BOOL bRet = FALSE;
	WIN32_FILE_ATTRIBUTE_DATA fd;
	ZeroMemory( &fd, sizeof( WIN32_FILE_ATTRIBUTE_DATA ) );
	if( lpFile ) {
		if( ::GetFileAttributesEx( lpFile, GetFileExInfoStandard, ( void* ) &fd ) ) {
			::FileTimeToSystemTime( &fd.ftLastAccessTime, &sys );
			bRet = TRUE;
		}
	}

	return bRet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Returns the time this file was last modified.
///////////////////////////////////////////////////////////////////////////////
/*static*/
BOOL CFileEx::GetTimeLastModified( LPCTSTR lpFile, SYSTEMTIME& sys )
{
	BOOL bRet = FALSE;
	WIN32_FILE_ATTRIBUTE_DATA fd;
	ZeroMemory( &fd, sizeof( WIN32_FILE_ATTRIBUTE_DATA ) );
	if( lpFile ) {
		if( ::GetFileAttributesEx( lpFile, GetFileExInfoStandard, ( void* ) &fd ) ) {
			::FileTimeToSystemTime( &fd.ftLastWriteTime, &sys );
			bRet = TRUE;
		}
	}

	return bRet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Returns the time this file was created.
///////////////////////////////////////////////////////////////////////////////
/*static*/
BOOL CFileEx::GetTimeCreated( LPCTSTR lpFile, SYSTEMTIME& sys )
{
	BOOL bRet = FALSE;
	WIN32_FILE_ATTRIBUTE_DATA fd;
	ZeroMemory( &fd, sizeof( WIN32_FILE_ATTRIBUTE_DATA ) );
	if( lpFile ) {
		if( ::GetFileAttributesEx( lpFile, GetFileExInfoStandard, ( void* ) &fd ) ) {
			::FileTimeToSystemTime( &fd.ftCreationTime, &sys );
			bRet = TRUE;
		}
	}

	return bRet;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Reads or writes the next segment of input/output.
///////////////////////////////////////////////////////////////////////////////
BOOL CFileEx::NextIoSegment( BOOL bWrite, RDWROVERLAPPEDPLUS& ovp, BYTE* pBuffer, DWORD dwTtlSegs, DWORD dwCurSeg )
{
	BOOL	bSuccess	= FALSE;
	BYTE*	pOffBuf		= NULL;
	DWORD	dwTransfer	= 0;


	pOffBuf			= ( BYTE* ) POINTEROFFSET( pBuffer, ovp.dwTotalSoFar );
	dwTransfer		= ( dwCurSeg == dwTtlSegs ) ? ( ovp.dwTotalSizeToTransfer % m_dwSegSize ) : m_dwSegSize;
	ovp.ov.Offset	= ovp.dwTotalSoFar;

	if( bWrite ) {
		bSuccess = ::WriteFileEx(	m_hFile, pOffBuf, dwTransfer, &ovp.ov, 
									CFileEx::FileIoCompletionRoutine );
	}else{
		bSuccess = ::ReadFileEx(	m_hFile, pOffBuf, dwTransfer, &ovp.ov, 
									CFileEx::FileIoCompletionRoutine );
	}		

	return bSuccess;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Handles all outstanding messages in the queue.  This is used during
//		idle time between io completions.
///////////////////////////////////////////////////////////////////////////////
void CFileEx::PumpMsgs()
{
	MSG	msg;
	while( ::PeekMessage( &msg, NULL, 0, 0, PM_NOREMOVE ) ) {
		::GetMessage( &msg, NULL, 0, 0 );
		::TranslateMessage( &msg );
		::DispatchMessage( &msg );
	}

	return;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		Reads/Writes a buffer to/from a file that has already been opened.  In doing this 
//		it allows for peice by peice file read/writes which can notify the the caller
//		after each segment has been read/written. A message pump is also provided
//		in this function for handling messages while waiting to service the
//		next I/O completion.
///////////////////////////////////////////////////////////////////////////////
DWORD CFileEx::DoFileOperationWithMsgPump( BOOL bWrite, BYTE* pBuffer, DWORD dwSize, LPFN_LRGFILEOP_PROGCALLBACK lpCallback, LPVOID pParam )
{
	RDWROVERLAPPEDPLUS	ovp;
	DWORD				dwNumSegs	= 0,						
						dwCurSeg	= 1;
	BOOL				bDone		= FALSE,
						bQuit		= 0;
	int					nRet		= 0;
	
	ZeroMemory( &ovp.ov, sizeof( OVERLAPPED ) );
	ovp.lpCallback				= lpCallback;
	ovp.pParam					= pParam;
	ovp.dwTotalSizeToTransfer	= dwSize;
	ovp.dwTotalSoFar			= 0;
	ovp.dwError					= 0;
	ovp.bContinue				= TRUE;

	dwNumSegs = ( dwSize + m_dwSegSize - 1 ) / m_dwSegSize;	// <-- eliviates need for floating point lib calc.	
	
	if( ! NextIoSegment( bWrite, ovp, pBuffer, dwNumSegs, dwCurSeg ) ) {
			// something fouled up in our NextIoSegment routine ( ReadFileEx or WriteFilEx failed )
			// so we set the error in the ovp structure and set quit and fake the nRet code.  By 
			// doing this an exception will be thrown on the way out of this function. setting bDone
			// to TRUE makes sure the loop never runs.
		ovp.dwError = ::GetLastError();
		bQuit		= TRUE;
		nRet		= WAIT_IO_COMPLETION;
		bDone		= TRUE;
	}

	while( ! bDone ) {
		nRet = ::MsgWaitForMultipleObjectsEx( 1, &m_hStop, INFINITE, QS_ALLEVENTS, MWMO_ALERTABLE );
		switch( nRet ) 
		{
			case WAIT_OBJECT_0:			bQuit = TRUE;	::ResetEvent( m_hStop );	break;
			case WAIT_OBJECT_0 + 1:		PumpMsgs();									break;
			case WAIT_IO_COMPLETION:
				{
					bDone = ( ovp.dwTotalSoFar == ovp.dwTotalSizeToTransfer );
	
					if( bDone || bQuit) {
						break;
					}
							// this signals either an error happened on the last I/O
							// compeletion rountine or the user returned FALSE from their
							// callback signaling to stop the IO process.
					if( ! ovp.bContinue ) {							
						bQuit = TRUE;
					}else{
						dwCurSeg++;
						if( ! NextIoSegment( bWrite, ovp, pBuffer, dwNumSegs, dwCurSeg ) ) {
							// something failed with our read/write call. This is an API error
							// so we need to handle it accordingly. Setting the ovp.dwError 
							// and setting Quit to TRUE will force an exception to be thrown
							// back to the user notifying them of the failure.
							ovp.dwError = ::GetLastError();
							bQuit		= TRUE;
						}
					}
				}
				break;
		};	

			// For Some reason we are now dropping out of this loop. This is mostly
			// likely a kill event that got signaled but we need to check for an
			// actual API error just in case.
		if( ( nRet == WAIT_IO_COMPLETION ) && bQuit ) {
			if( ovp.dwError != 0 ) {
				ThrowErr( ovp.dwError, FALSE );
			}
			break;
		}
	}

	return ovp.dwTotalSoFar;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		This function does pretty much the exact same thing as the function
//		above ( DoFileOperationWithMsgPump ). In fact it is 90% the same code.
//		The only difference is that this version of the function does not
//		support a message pump for the idle time between IO compeltions.
///////////////////////////////////////////////////////////////////////////////
DWORD CFileEx::DoFileOperation( BOOL bWrite, BYTE* pBuffer, DWORD dwSize, LPFN_LRGFILEOP_PROGCALLBACK lpCallback, LPVOID pParam )
{
	RDWROVERLAPPEDPLUS	ovp;
	DWORD				dwNumSegs	= 0,						
						dwCurSeg	= 1;
	BOOL				bDone		= FALSE,
						bQuit		= 0;
	int					nRet		= 0;

	ZeroMemory( &ovp.ov, sizeof( OVERLAPPED ) );
	ovp.lpCallback				= lpCallback;
	ovp.pParam					= pParam;
	ovp.dwTotalSizeToTransfer	= dwSize;
	ovp.dwTotalSoFar			= 0;
	ovp.dwError					= 0;
	ovp.bContinue				= TRUE;

	dwNumSegs = ( dwSize + m_dwSegSize - 1 ) / m_dwSegSize;	// <-- eliviates need for floating point lib calc.		
	
	if( ! NextIoSegment( bWrite, ovp, pBuffer, dwNumSegs, dwCurSeg ) ) {
			// something fouled up in our NextIoSegment routine ( ReadFileEx or WriteFilEx failed )
			// so we set the error in the ovp structure and set quit and fake the nRet code.  By 
			// doing this an exception will be thrown on the way out of this function. setting bDone
			// to TRUE makes sure the loop never runs.
		ovp.dwError = ::GetLastError();
		bQuit		= TRUE;
		nRet		= WAIT_IO_COMPLETION;
		bDone		= TRUE;
	}

	while( ! bDone ) {
		nRet = ::WaitForMultipleObjectsEx( 1, &m_hStop, FALSE, INFINITE, TRUE );
		switch( nRet ) 
		{
			case WAIT_OBJECT_0:			bQuit = TRUE;	::ResetEvent( m_hStop );	break;
			case WAIT_IO_COMPLETION:	
				{
					bDone = ( ovp.dwTotalSoFar == ovp.dwTotalSizeToTransfer );
	
					if( bDone || bQuit ) {
						break;
					}

							// this signals either an error happened on the last I/O
							// compeletion rountine or the user returned FALSE from their
							// callback signaling to stop the IO process.
					if( ! ovp.bContinue ) {							
						bQuit = TRUE;
					}else{
						dwCurSeg++;						
						if( ! NextIoSegment( bWrite, ovp, pBuffer, dwNumSegs, dwCurSeg ) ) {
							// something failed with our read/write call. This is an API error
							// so we need to handle it accordingly. Setting the ovp.dwError 
							// and setting Quit to TRUE will force an exception to be thrown
							// back to the user notifying them of the failure.
							ovp.dwError = ::GetLastError();
							bQuit		= TRUE;
						}
					}
				}
				break;
		};	

			// For Some reason we are now dropping out of this loop. This is mostly
			// likely a kill event that got signaled but we need to check for an
			// actual API error just in case.
		if( ( nRet == WAIT_IO_COMPLETION ) && bQuit ) {
			if( ovp.dwError != 0 ) {
				ThrowErr( ovp.dwError, FALSE );
			}
			break;
		}
	}

	return ovp.dwTotalSoFar;
}

///////////////////////////////////////////////////////////////////////////////
//	Desc:
//		This is the compeltion routine used in the DoFileOperationWithMsgPump
//		and DoFileOperation functions to service an IO completion.
//		It updates the total amount transferred so far and calls the user's
//		callback routine if one was provided.
///////////////////////////////////////////////////////////////////////////////
/*static*/
void CALLBACK CFileEx::FileIoCompletionRoutine( DWORD dwErrorCode, DWORD dwNumTrans, LPOVERLAPPED lpOverlapped )
{
	LPRDWROVERLAPPEDPLUS lpRwStruct = NULL;

	if( ! lpOverlapped ) {
		return;
	}

	lpRwStruct = CONTAINING_RECORD( lpOverlapped, RDWROVERLAPPEDPLUS, ov );
	if( lpRwStruct ) {
		lpRwStruct->dwTotalSoFar += dwNumTrans;
		if( lpRwStruct->lpCallback != NULL ) {
			lpRwStruct->bContinue = 
				lpRwStruct->lpCallback( lpRwStruct->dwTotalSoFar, lpRwStruct->dwTotalSizeToTransfer, lpRwStruct->pParam );
		}
		
	
		lpRwStruct->dwError		= dwErrorCode;	// will be zero on no error.
		lpRwStruct->bContinue	= ( ( dwErrorCode == 0 ) && lpRwStruct->bContinue );	//force quit on error.

	}	

	return;
}