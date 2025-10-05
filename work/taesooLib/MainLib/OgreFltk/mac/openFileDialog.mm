#include <stdio.h>
#include <string>
#import <Cocoa/Cocoa.h>
#import "appDelegate.h"

std::string chooseFileMac(const char* path, const char* mask);
/*int main(int argc, char* argv[])
{
	printf("%s\n", chooseFileMac(argv[1], argv[2]).c_str());
	return 0;
}
*/
std::string chooseFileMac(const char* path, const char* mask){
	// WORKING :)
	NSWindow *keyWindow = [NSApp keyWindow];
	NSOpenPanel *panel;
	NSArray* fileTypes = [[NSArray alloc] initWithObjects:[[NSString alloc] initWithCString:mask], @"PDF", nil];
	panel = [NSOpenPanel openPanel];
	[panel setFloatingPanel:YES];
	[panel setCanChooseDirectories:NO];
	[panel setCanChooseFiles:YES];
	[panel setAllowsMultipleSelection:NO];
	[panel setAllowedFileTypes:fileTypes];
	[panel setDirectoryURL:[NSURL fileURLWithPath:[[[NSString alloc] initWithCString:path] stringByExpandingTildeInPath]]];
	int i = [panel runModal];
	if(i == NSOKButton){
		NSURL *nsurl = [[panel URLs] objectAtIndex:0];
		std::string url2([[nsurl path] UTF8String]);
		return url2;
		//NSPrint(@"%s", [panel URL]);
		//return std::string([[[panel URLs] objectAtIndex:0] UTF8String]);
		//printf("%s\n",[[[panel URL] absoluteString] UTF8String]);
		//std::string url([[[panel URL] absoluteString] UTF8String]);
		//[keyWindow makeKeyWindow];
		//return url.substr( 7) ; // removing "file://"
	}
	//[keyWindow makeKeyWindow];
	return std::string("");
}
static char **g_argv;
@implementation MyApplicationDelegate : NSObject
- (id)init {
    if (self = [super init]) {
        // allocate and initialize window and stuff here ..
    }

    return self;
}

- (void)applicationWillFinishLaunching:(NSNotification *)notification {
    [window makeKeyAndOrderFront:self];
	printf("%s\n", chooseFileMac(g_argv[1], g_argv[2]).c_str());
}

- (void)dealloc {
    [window release];
    [super dealloc];
}

@end
int main(int argc, char * argv[]) {
g_argv=argv;
    NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];
    NSApplication * application = [NSApplication sharedApplication];

    MyApplicationDelegate * appDelegate = [[[MyApplicationDelegate alloc] init] autorelease];

    [application setDelegate:appDelegate];
    [application run];

    [pool drain];

    return EXIT_SUCCESS;
}
