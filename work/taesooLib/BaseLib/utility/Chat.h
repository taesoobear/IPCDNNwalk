#pragma once
#include <string>
//##ifndef _MSC_VER
#if defined(_WIN32)
#else
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <errno.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#endif

class ChatClient
{

	int    sockfd;
	char   send_buf[4096];
	char   recv_buf[4096];
#ifdef _WIN32
	void* data;
#else
	fd_set master_set, select_result;
#endif
	int max_sd;

	bool connected;
	std::string msg;
	bool _useStdIn;
	public:
	ChatClient(bool useStdIn, const char* server_addr="127.0.0.1");
	~ChatClient();

	inline bool isConnected() { return connected;}
	inline std::string getErrorMessage() { return msg;}

	std::string getMessage(int tv_sec=3*60, int tv_usec=0);
	void sendMessage(const char* msg);
};


class ChatServer
{
#ifdef _WIN32
	void* data;
#else
	fd_set master_set;
#endif
	int listen_sd;
	int max_sd;
	void init_set();
	void add_to_set(int new_sd);
	void remove_from_set(int sd);
	void broadcast(int sender, char* msg);

	std::string msg;
	bool waiting;
	public:
	ChatServer();

	inline bool isWaiting() { return waiting;}
	inline std::string getErrorMessage() { return msg;}
	bool singleStep(int tv_sec=3*60, int tv_usec=0);
	~ChatServer();
};
//#endif
