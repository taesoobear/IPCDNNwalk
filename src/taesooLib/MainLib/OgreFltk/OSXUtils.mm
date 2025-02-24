#include "OSXUtils.h"

#import <Cocoa/Cocoa.h>
#import <AppKit/NSWindow.h>
#import <Ogre.h>


unsigned long WindowContentViewHandleFltk(void* handle)
{
    NSWindow *window = (NSWindow*)handle;
    NSView *view = [window contentView];
    return (unsigned long)view;
}

CGRect createCGrect(void* handle)
{

    NSWindow *win= (NSWindow*)handle;

    NSScreen *screen = NSScreen.mainScreen;
    NSRect rect = screen.frame;
	return CGRectMake(
	win.frame.origin.x,
	rect.size.height-win.frame.origin.y-win.frame.size.height,
	//win.frame.origin.y,
	win.frame.size.width,
	win.frame.size.height);
}



/*
bool CanCapture(void) {
    for(;;) {
        if (CGPreflightScreenCaptureAccess()) {
            return true;
        }
        if (!CGRequestScreenCaptureAccess()) {
            return false;
        }
    }
    return false;
}

#import <Foundation/Foundation.h>
#import <ScreenCaptureKit/ScreenCaptureKit.h>
#import <CoreGraphics/CoreGraphics.h>
#import <AVFoundation/AVFoundation.h>
//https://github.com/Fidetro/CapturingScreenContentInMacOS?tab=readme-ov-file
// Create a content filter that includes a single window.
//filter = SCContentFilter(desktopIndependentWindow: window)


CGImageRef SCScreenshotManager_captureImage(SCContentFilter* filter, SCStreamConfiguration* config) {
    __block CGImageRef capturedImage = NULL;
    dispatch_semaphore_t semaphore = dispatch_semaphore_create(0);

    NSLog(@"Initializing SCStream...");
    SCStream* stream = [[SCStream alloc] initWithFilter:filter configuration:config delegate:nil];

    NSLog(@"Starting capture...");
    [stream startCaptureWithCompletionHandler:^(NSError * _Nullable error) {
        if (error) {
            NSLog(@"Error starting capture: %@", error.localizedDescription);
            NSLog(@"Error domain: %@", error.domain);
            NSLog(@"Error code: %ld", (long)error.code);
            NSLog(@"Error user info: %@", error.userInfo);
            dispatch_semaphore_signal(semaphore);
        } else {
            NSLog(@"Capture started successfully, attempting to capture screenshot...");
            [stream stopCaptureWithCompletionHandler:^(NSError * _Nullable error) {
                if (error) {
                    NSLog(@"Error stopping capture: %@", error.localizedDescription);
                    NSLog(@"Error domain: %@", error.domain);
                    NSLog(@"Error code: %ld", (long)error.code);
                    NSLog(@"Error user info: %@", error.userInfo);
                } else {
                    NSLog(@"Capture stopped successfully");
                }
                dispatch_semaphore_signal(semaphore);
            }];
        }
    }];

    dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
    [stream release];
    return capturedImage;
}

CGImageRef takeScreenShot(void* _hWnd)
{
	NSWindow *win= (NSWindow*)_hWnd;
	CGRect r=createCGrect(_hWnd);
	CGWindowID windowID = (CGWindowID)[win windowNumber];
	NSError *error = nil;
	printf("a\n");
	SCShareableContent *shareableContent = [SCShareableContent excludingDesktopWindows:NO 
		onScreenWindowsOnly:YES 
		error:&error];

	if (error) {
		printf("err1\n");
		NSLog(@"Error fetching shareable content: %@", error.localizedDescription);
	}
	printf("b\n");

	SCWindow *targetWindow = nil;
	// Find the SCWindow that matches the CGWindowID
	for (SCWindow *window in shareableContent.windows) {
		if (window.windowID == windowID) {
			targetWindow = window;
		}
	}
	printf("c\n");
	if (!targetWindow) {
		printf("err2\n");
		NSLog(@"SCWindow not found for CGWindowID: %u", windowID);
	}
    @autoreleasepool {
        if (CanCapture()) {
            NSLog(@"Here\n");
            [SCShareableContent getShareableContentExcludingDesktopWindows: true
                                                       onScreenWindowsOnly: true
                                                         completionHandler: ^(SCShareableContent *shareableContent, NSError *error) {
                NSLog(@"But not Here\n");
				SCWindow *targetWindow = nil;
				// Find the SCWindow that matches the CGWindowID
				for (SCWindow *window in shareableContent.windows) {
					if (window.windowID == windowID) {
						targetWindow = window;
					}
				}
				if (!targetWindow) {
					NSLog(@"SCWindow not found for CGWindowID: %u", windowID);
					return;
				}

				// Create an SCContentFilter with the selected window
				SCContentFilter *filter = [[SCContentFilter alloc] initWithDesktopIndependentWindow:targetWindow];
				SCStreamConfiguration* config=[[SCStreamConfiguration alloc] init];
				CGImageRef image= SCScreenshotManager_captureImage(filter, config) ;
				[config release];
				[filter release];

            }];
			

        }
    }
	return NULL;
}
*/
