#if defined(_WIN32)
//#include <winsock.h>
#include <winsock2.h>
#include <ws2tcpip.h>

struct _ChatData
{
	fd_set MASTER_SET, SELECT_RESULT;
};

#define master_set (((_ChatData*)(data))->MASTER_SET)
#define select_result (((_ChatData*)(data))->SELECT_RESULT)

#endif
#include "Chat.h" // on windows winsock2.h has to be included before windows.h -.-

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>

//#ifndef _MSC_VER


#define SERVER_PORT  5001
#define TRUE             1
#define FALSE            0
#ifndef _WIN32
#define SOCKET_ERROR -1
#define INVALID_SOCKET -1
#endif
ChatClient::ChatClient(bool useStdIn, const char* server_addr) 
{

#ifdef _WIN32
	data=(void*) new _ChatData();
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;

	/* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
	wVersionRequested = MAKEWORD(2, 2);

	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		/* Tell the user that we could not find a usable */
		/* Winsock DLL.                                  */
		printf("WSAStartup failed with error: %d\n", err);
		return;
	}
#endif
	struct sockaddr_in   serv_addr;
	int    len, rc;
	connected=false;
	if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) ==SOCKET_ERROR)
	{
		msg="socket creation error";
		return;
	}

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family      = AF_INET;
	serv_addr.sin_port        = htons(SERVER_PORT);

	if(inet_pton(AF_INET, server_addr, &serv_addr.sin_addr)==SOCKET_ERROR) 
	{
		msg="ip address error";
		return;
	}

	if(connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) ==SOCKET_ERROR)
	{
		msg="connection error";
		return;
	}

	FD_ZERO(&master_set);
	FD_SET(sockfd, & master_set);
	_useStdIn=useStdIn;
	if (useStdIn)
	{
		FD_SET(fileno(stdin), & master_set);
		max_sd=MAX(fileno(stdin), sockfd);
	}
	else
		max_sd=sockfd;

	connected=true;
}
std::string ChatClient::getMessage(int tv_sec, int tv_usec)
{
	struct timeval       timeout;
	int    len, rc;

	fd_set select_result;

	timeout.tv_sec  = tv_sec;
	timeout.tv_usec = tv_usec;

	memcpy(&select_result, &master_set, sizeof(master_set));
	rc = select(max_sd + 1, &select_result, NULL, NULL, &timeout);
	if(_useStdIn && FD_ISSET(fileno(stdin), & select_result)	)
	{
		fgets(send_buf, 4096,stdin );
		send(sockfd, send_buf, strlen(send_buf), 0);
	}
	else if(FD_ISSET(sockfd, & select_result)	)
	{
		rc = recv(sockfd, recv_buf, 4096, 0);
		if (rc == SOCKET_ERROR ) 
		{
			msg="recv failed";
#ifdef _WIN32
			closesocket(sockfd);
#else
			close(sockfd);
#endif
			connected=false;
			return std::string("!");
		}
		if (rc == 0) 
		{
			msg="Server disconnected";
#ifdef _WIN32
			closesocket(sockfd);
#else
			close(sockfd);
#endif
			connected=false;
			return std::string("!");
		}
		recv_buf[rc]=0; // make null terminated string
		return std::string(recv_buf);
	}
	return std::string("???");
}
ChatClient::~ChatClient() { 
#ifdef _WIN32
	if (connected) closesocket(sockfd);
#else
	if(connected) close(sockfd);
#endif

#ifdef _WIN32
	WSACleanup();
	delete (_ChatData*)data;
#endif
}


void ChatClient::sendMessage(const char* msg)
{
	//printf("send: %d %s\n", connected, msg);
	if(connected)
		send(sockfd, msg, strlen(msg), 0);
}



void ChatServer::init_set()
{
	FD_ZERO(&master_set);
	max_sd=0;
}

void ChatServer::add_to_set(int new_sd)
{
	FD_SET(new_sd, &master_set);
	max_sd=MAX(new_sd, max_sd);
}

void ChatServer::remove_from_set(int sd)
{
	FD_CLR(sd, &master_set);
	if (sd == max_sd)
	{
		while (FD_ISSET(max_sd, &master_set) == FALSE)
			max_sd -= 1;
	}
}

void ChatServer::broadcast(int sender, char* msg)
{
	int j;
	int len=strlen(msg);
	for (j=0; j <= max_sd  ; ++j)
	{
		if (FD_ISSET(j, &master_set) && j!=listen_sd && j!=sender)
		{
			send(j, msg, len, 0);
		}
	}
}
ChatServer::ChatServer()
{

#ifdef _WIN32
	data=(void*) new _ChatData();
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;

	/* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
	wVersionRequested = MAKEWORD(2, 2);

	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		/* Tell the user that we could not find a usable */
		/* Winsock DLL.                                  */
		printf("WSAStartup failed with error: %d\n", err);
		return;
	}
#endif
	waiting =false;
	int    i, j, len, rc, on = 1;
	int    new_sd;
	int    close_conn;
	char   buffer[4096];
	char   buffer2[4096];
	struct sockaddr_in   addr;
	fd_set select_result;

	listen_sd = socket(AF_INET, SOCK_STREAM, 0);

	if (listen_sd == SOCKET_ERROR ) 
	{
#ifdef _WIN32
		printf( "Listen failed with error: %ld\n", WSAGetLastError() );
#endif
		msg="listen_failed";
		return;
	}

	/* Allow socket descriptor to be reuseable                   */
	rc = setsockopt(listen_sd, SOL_SOCKET,  SO_REUSEADDR, (char *)&on, sizeof(on));
	if (rc == SOCKET_ERROR) 
	{
		msg="setsockopt";
		return;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family      = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port        = htons(SERVER_PORT);
	rc = bind(listen_sd, (struct sockaddr *)&addr, sizeof(addr));
	if (rc == SOCKET_ERROR ) 
	{
		msg="bind";
		return;
	}

	rc = listen(listen_sd, 32);
	if (rc == SOCKET_ERROR ) 
	{
		msg="listen";
		return;
	}

	printf("Chat server started on port %d\n", SERVER_PORT);

	init_set();
	add_to_set(listen_sd);
	add_to_set(fileno(stdin));

	waiting=true;
}

bool ChatServer::singleStep(int tv_sec, int tv_usec)
{
	struct timeval       timeout;
	int len,rc;

	char   buffer[4096];
	char   buffer2[4096];
	fd_set select_result;
	/* Initialize the timeval struct to 3 minutes.          */
	timeout.tv_sec  =tv_sec;
	timeout.tv_usec = tv_usec;

	int    new_sd;
	{
		memcpy(&select_result, &master_set, sizeof(master_set));

		rc = select(max_sd + 1, &select_result, NULL, NULL, &timeout);

		if (rc == SOCKET_ERROR) 
		{
#ifdef _WIN32
		printf( "select failed with error: %ld\n", WSAGetLastError() );
#endif
			msg="select failed";
			return false;
		}
		if (rc == 0) { 
			msg="timeout";
			return false;
		}

		// rc descriptors are readable now.  
		for (int i=0; i <= max_sd  ; ++i)
		{
			if (FD_ISSET(i, &select_result))
			{
				if (i == listen_sd)
				{
					new_sd = accept(listen_sd, NULL, NULL);
					if (new_sd ==INVALID_SOCKET) exit(-8);

					printf("Client %d connected\n", new_sd);
					add_to_set(new_sd);
					snprintf(buffer, 4096, "[%d] entered room", new_sd);
					broadcast(new_sd, buffer);
				}
				else if(i==fileno(stdin))
				{
					char   send_buf[4096];
					fgets(send_buf, 4096, stdin);
					broadcast(i, send_buf);
					fflush(stdout);
				}
				else
				{
					rc = recv(i, buffer, 4096, 0);
					if (rc ==SOCKET_ERROR) exit(-9); // recv failed
					if (rc == 0)  // disconnected
					{
#ifdef _WIN32
						closesocket(i);
#else
						close(i);
#endif
						remove_from_set(i);
						snprintf(buffer2, 4096, "Client %d disconnected\n", i);
					}
					else
					{
						buffer[rc]=0; // make null terminated string
						snprintf(buffer2, 4096, "%s", buffer);
					}

					// broad cast to other clients
					broadcast(i, buffer2);
					printf("%s", buffer2);
					fflush(stdout);
				} 
			} 
		} 
	} 

	return true;
}

ChatServer::~ChatServer()
{
	int i;
	for (i=0; i <= max_sd; ++i)
	{
		if (FD_ISSET(i, &master_set))
#ifdef _WIN32
			closesocket(i);
#else
			close(i);
#endif
	}
#ifdef _WIN32
	WSACleanup();
	delete (_ChatData*)data;
#endif
}
//#endif
