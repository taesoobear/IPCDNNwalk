#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <pthread.h>
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Text_Editor.H>
#include <FL/Fl_Button.H>
#include <string>	// std::string
#include <signal.h>	// kill()
#include <sys/wait.h>	// waitpid()

#define READ	0
#define WRITE	1
#define BUFSIZE 1024

//
// Simple terminal simulator (PROOF OF CONCEPT)
// 1.00 erco 07/30/2009 -- initial implementation
// 1.10 erco 09/06/2009 -- use Fl::event_text() to handle non-ascii codes
// 1.20 erco 08/27/2011 -- added bidirectional pipes, threads (UNIX ONLY!)
//
// TODO: reap child if it exits, avoid SIGPIPE
//

//// Class to manage buffering with locks for safe interaction between threads
////    Allows one thread to pass data to another without concern
////    for race conditions. All locking is done on non-blocking
////    operations to avoid deadlocks.			-- erco 8/28/11
////
class Buffer {
    pthread_mutex_t lock;		// thread lock to buffer + cnt
    char buf[BUFSIZE];			// buffer
    int cnt;				// count of bytes in buf
    // Lock private data
    void Lock() {
        pthread_mutex_lock(&lock);
    }
    // Unlock private data
    void Unlock() {
        pthread_mutex_unlock(&lock);
    }
public:
    // Ctor
    Buffer() {
	pthread_mutex_init(&lock, NULL);
        cnt = 0;
    }
    // Clear the buffer
    void Clear() {
        Lock();
/*LOCK*/    cnt = 0;
	Unlock();
    }
    // Returns #bytes waiting in buffer.
    int Peek() {
        int lcnt;
        Lock();
/*LOCK*/    lcnt = cnt;
	Unlock();
	return(lcnt);
    }
    // Write buf to fd
    //    May block if fd blocks, but won't block during lock.
    //
    void WriteBufToFD(int fd) {
        char lbuf[BUFSIZE];			// local buffer (avoids write() during Lock())
	int nbytes = Read(lbuf, sizeof(lbuf));	// read bytes, handles locking
	if ( nbytes > 0 )
	    write(fd, lbuf, nbytes);		// may block
    }
    // Write bytes to buf, returns bytes written (can be less).
    int Write(const char *wbuf, int wbufsz) {
        int avail = sizeof(buf) - cnt;
	if ( wbufsz > avail ) wbufsz = avail;
        Lock();
/*LOCK*/    memcpy(buf+cnt, wbuf, wbufsz);
/*LOCK*/    cnt += wbufsz;
	Unlock();
	return(wbufsz);
    }
    // Read bytes from buf, returns bytes read (never more than rbufsz).
    int Read(char *rbuf, int rbufsz) {
        int rbytes = cnt;		// try to return all bytes
	int diff = 0;
	if ( rbufsz < rbytes ) {	// return buffer smaller?
	    rbytes = rbufsz;		// return as much as we can
	    diff = cnt - rbytes;	// keep track of what's left
	}
	Lock();
/*LOCK*/    memcpy(rbuf, buf, rbytes);	// xsfr buf to caller's buffer
/*LOCK*/    if ( diff > 0 ) memmove(buf, buf+rbytes, diff);	// shuffle buffer
/*LOCK*/    cnt -= rbytes;
	Unlock();
	return(rbytes);
    }
};
//// Class to make a "dumb terminal" out of a text editor
////    Implements simple character cooking (Backspace, ^U, etc)
////
class MyTerminal : public Fl_Text_Editor {
    Fl_Text_Buffer *buff;
    int child_wfd;		// child's stdin file descriptor
    int child_rfd;		// child's stdout/err file descriptor
    pid_t child_pid;		// child's pid
    pthread_t rtid;		// thread to handle reading stdout_buf from child's stdout/err
    pthread_t wtid;		// thread to handle writing stdin_buf to child's stdin
    Buffer stdin_buf;		// buffer to handle child's stdin
    Buffer stdout_buf;		// buffer to handle child's stdout/err
    char line[4000];		// line input buffer for rawmode=0 (flushed when user hits return)
    int rawmode;		// 0:'cook' input from user (handle backspace, ^U, ^W, etc), 1:raw, uncooked
    // Thread handler: Read stdout/err from child asynchronously, send to parent
    static void* ChildReadThread(void *cbdata) {
        MyTerminal *o = (MyTerminal*)cbdata;
	int fd = o->child_rfd;	// read child's stdout/err
	while (1) {
	    char c;
	    if ( read(fd, &c, 1) == 1 ) {
	        while ( o->stdout_buf.Write(&c, 1) != 1 ) {
		    usleep(5000);
		}
	    }
	}
	return(0);
    }
    // Thread handler: Write FLTK keystrokes asynchronously to child
    static void* ChildWriteThread(void *cbdata) {
        MyTerminal *o = (MyTerminal*)cbdata;
	while (1) {
	    int peek = o->stdin_buf.Peek();
	    if ( peek > 0 ) {
		o->stdin_buf.WriteBufToFD(o->child_wfd);	// write entire buf to fd
	    }
	    usleep(5000);
	}
	return(0);
    }
    // FLTK timer handler
    static void IOTimer_CB(void *cbdata) {
        MyTerminal *o = (MyTerminal*)cbdata;
	char lbuf[1025];
	int nbytes;
	if ( ( nbytes = o->stdout_buf.Read(lbuf, sizeof(lbuf)-1) ) > 0 ) {
	    lbuf[nbytes] = 0;		// ensure null terminated, before..
	    o->append(lbuf);		// ..append to text editor
	}
	Fl::repeat_timeout(.01, IOTimer_CB, cbdata);
    }
public:
    // Ctor
    MyTerminal(int X,int Y,int W,int H,const char* L=0) : Fl_Text_Editor(X,Y,W,H,L) {
        buff = new Fl_Text_Buffer();
        buffer(buff);
        textfont(FL_COURIER);
        textsize(12);
	child_pid = 0;
	rtid      = 0;
	wtid      = 0;
	rawmode   = 0;
	line[0]   = 0;
    }
    // Dtor
    ~MyTerminal() {
        StopChild();
    }
    // Append to buffer, keep cursor at end
    void append(const char*s) {
        buff->append(s);
        // Move cursor to eol
        insert_position(buffer()->length());
        scroll(count_lines(0, buffer()->length(), 1), 0);
    }
    // Flush the line input buffer
    void FlushLine() {
        if ( rawmode ) return;
        char *linep = line;
        int linelen = strlen(line);
	int wcnt;
	while ( 1 ) {
	    if ( ( wcnt = stdin_buf.Write(linep, linelen) ) > 0 ) {
		linep += wcnt;				// adjust pointer based on bytes written
		linelen -= wcnt;			// adjust len
		if ( linelen == 0 ) break;		// all bytes sent? done
	    }
	    usleep(5000);				// wait for buffer to empty
	}
	line[0] = 0;					// all written, empty
    }
    // Handle events in the Fl_Text_Editor
    int handle(int e) {
        switch (e) {
            case FL_PASTE: {				// make sure pastes get sent to child
		const char *pbuf = Fl::event_text();
		int pbuflen = strlen(pbuf);
		if ( rawmode ) {
		    stdin_buf.Write(pbuf, pbuflen);
		} else {
		    if ( ( strlen(line) + pbuflen ) < sizeof(line) ) {
			strcat(line, pbuf);
			if ( strchr(line, '\n') ) {	// at least one CRLF?
			    FlushLine();		// flush entire line[] buffer
			}
		    }
		}
		break;
	    }
            case FL_KEYUP: {
		if ( rawmode ) {			// no character cooking?
		    return(1);				// don't echo
		}
		break;
            }
            case FL_KEYDOWN: {
                int key = Fl::event_key();		// key pressed
		const char *text = Fl::event_text();	// ascii version of key
		int textlen = Fl::event_length();
		// No character cooking?
		if ( rawmode ) {
		    stdin_buf.Write(text, textlen);	// Write directly to child
		    return(1);				// don't let editor see it
		}
		switch(key) {
		    case FL_Enter:			// Enter
		    case FL_KP_Enter:			// Enter on keypad
			strcat(line, "\n");		// use \n instead of \r
			FlushLine();
			break;
		    case FL_BackSpace:			// backspace? remove char from end of buffer
			if ( line[0] ) {		// buffer has contents?
			    line[strlen(line)-1] = 0;	// remove last char from our buffer
			    break;			// let editor handle bs
			} else {
			    return(1);			// buffer empty? don't let user continue to bs
			}
			break;
		    default:
			if (text[0] == 0x15 ) {		// ^U: line cancel
			    for ( int t=0; t<(int)strlen(line); t++ ) {
				kf_backspace(0, this);
			    }
			    line[0] = 0;
			    return(1);
			}
			if (text[0] == 0x17 ) {		// ^W: word cancel
			    // Delete word left
			    int t = (int)strlen(line)-1;
			    int trailwhite = 1;		// delete trailing whitespace
			    while ( t >= 0 ) {
			        if ( trailwhite && line[t] != ' ' ) trailwhite = 0;
				if ( !trailwhite && line[t] == ' ' ) break;
				kf_backspace(0, this);
				line[t--] = 0;
			    }
			    return(1);
			}
			// Otherwise, append to line buffer, pass on to text editor
			strncat(line, text, sizeof(line)-1);
			line[sizeof(line)-1] = 0;
			break;
		}
            }
        }
        return(Fl_Text_Editor::handle(e));
    }
    // Stop child (if any), close pipes, cancel threads, timers.
    void StopChild() {
        // Kill child
	if (child_pid > 0 ) { kill(child_pid, SIGKILL); waitpid(child_pid,0,0); child_pid = 0; }
	// Cancel threads
	if (rtid > 0 ) { pthread_cancel(rtid); rtid = 0; }
	if (wtid > 0 ) { pthread_cancel(wtid); wtid = 0; }
	// Close pipes
	if (child_rfd > 0 ) { close(child_rfd); child_rfd = 0; }
	if (child_wfd > 0 ) { close(child_wfd); child_wfd = 0; }
	// Dump line buffer, remove timeout
	line[0] = 0;
	Fl::remove_timeout(IOTimer_CB, (void*)this);
    }
    // Fork off a child and exec() the cmd_args [NULL terminated ptr array]
    // Create the pipes for redirected stdio to child.
    // Returns 0 on success, -1 on error, errmsg has reason.
    //
    int StartChild(char **cmd_args, std::string &errmsg) {
        // Stop previous children (if any)
        if ( child_pid > 0 ) StopChild();
	// Make two pipes
	//    p1 will be stdin for child (parent writes)
	//    p2 will be stdout/err for child (parent reads)
	//
	int p1[2], p2[2];
	if (pipe(p1) < 0) {
	    errmsg = "pipe1: ";
	    errmsg += strerror(errno);
	    return(-1);
	}
	if (pipe(p2) < 0) {
	    errmsg = "pipe2: ";
	    errmsg += strerror(errno);
	    return(-1);
	}
	// FORK A CHILD
	switch(child_pid = fork()) {
	    case -1: // ERROR
		errmsg = "fork() failed: ";
		errmsg += strerror(errno);
		return(-1);
	    case 0: // CHILD
		// redirect stdio of child to pipes
		dup2(p1[READ],  0);
		dup2(p2[WRITE], 1);
		dup2(p2[WRITE], 2);
		// close unused descriptors
		close(p1[READ]);
		close(p1[WRITE]);
		close(p2[READ]);
		close(p2[WRITE]);
		// Start child
		execvp(cmd_args[0], cmd_args);	// exec's child (should not return)
		perror("exec() failed");
		_exit(1);
	    default: // PARENT
	        // Save ends of pipe we need to interact with shell
	        child_wfd = p1[WRITE];
	        child_rfd = p2[READ];
		// close unused ends of pipe
		close(p1[READ]);
		close(p2[WRITE]);
		break;
	}
	// Start two threads to asynchronously pump stdin/out
	// between child and FLTK parent via in/out buffers.
	void *vp = (void*)this;
	int err;
	if ((err = pthread_create(&rtid, NULL, ChildReadThread, vp)) != 0) {
	    errmsg = "pthread_create(1) failed"; errmsg += strerror(err);
	    StopChild();
	    return(-1);
	}
	if ((err = pthread_create(&wtid, NULL, ChildWriteThread, vp)) != 0) {
	    errmsg = "pthread_create(2) failed"; errmsg += strerror(err);
	    StopChild();
	    return(-1);
	}
	Fl::add_timeout(.05, IOTimer_CB, (void*)this);
	return(0);
    }
    pid_t GetChildPid() {
        return(child_pid);
    }
    void SetRawMode(int val) {
        rawmode = val;
    }
};
// TEST BUTTON: START TCSH SHELL
void TcshButtonCB(Fl_Widget*, void *cbdata) {
    MyTerminal *term = (MyTerminal*)cbdata;
    std::string errmsg;
    char *cmd_argv[10];
    int i = 0;
    cmd_argv[i++] = "/bin/tcsh";
    cmd_argv[i++] = "-is";
    cmd_argv[i++] = 0;
    if ( term->StartChild(cmd_argv, errmsg) < 0 ) {
	term->append("ERROR: ");
	term->append(errmsg.c_str());
    } else {
        term->append("\n\n*** tcsh(1) started\n");
    }
    Fl::focus(term);		// return focus to editor
}
// TEST BUTTON: START NSLOOKUP INTERPRETER
void NslookupButtonCB(Fl_Widget*, void *cbdata) {
    MyTerminal *term = (MyTerminal*)cbdata;
    std::string errmsg;
    char *cmd_argv[10];
    int i = 0;
    cmd_argv[i++] = "nslookup";
    cmd_argv[i++] = 0;
    if ( term->StartChild(cmd_argv, errmsg) < 0 ) {
	term->append("ERROR: ");
	term->append(errmsg.c_str());
    } else {
        term->append("\n\n*** nslookup(1) started\n");
    }
    Fl::focus(term);		// return focus to editor
}
// TEST BUTTON: START PYTHON
void PythonButtonCB(Fl_Widget*, void *cbdata) {
    MyTerminal *term = (MyTerminal*)cbdata;
    std::string errmsg;
    char *cmd_argv[10];
    int i = 0;
    cmd_argv[i++] = "python";
    cmd_argv[i++] = "-iu";
    cmd_argv[i++] = 0;
    if ( term->StartChild(cmd_argv, errmsg) < 0 ) {
	term->append("ERROR: ");
	term->append(errmsg.c_str());
    } else {
        term->append("\n\n*** python(1) started\n");
    }
    Fl::focus(term);		// return focus to editor
}
int main() {
    Fl_Double_Window win(620,520,"Terminal Test");
    MyTerminal edit(10,10,win.w()-20,win.h()-50);
    // Test various child processes
    Fl_Button b1(10,490,100,25,"tcsh");
    b1.callback(TcshButtonCB, (void*)&edit);
    b1.tooltip("Press here to start a new tcsh child");
    Fl_Button b2(120,490,100,25,"nslookup");
    b2.callback(NslookupButtonCB, (void*)&edit);
    b2.tooltip("Press here to start nslookup");
    Fl_Button b3(230,490,100,25,"python");
    b3.callback(PythonButtonCB, (void*)&edit);
    b3.tooltip("Press here to start python");
    // Setup window
    win.resizable(win);
    win.show();
    edit.append("*** ^U: line cancel\n"
                "*** ^W: word delete\n"
                "*** No other special characters supported\n");
    // Start with a tcsh
    TcshButtonCB(0, &edit);
    return(Fl::run());
}
