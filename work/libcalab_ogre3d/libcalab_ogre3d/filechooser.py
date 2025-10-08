#!/usr/bin/env python

# example filechooser.py

import pygtk
pygtk.require('2.0')

import gtk, os,sys

# Check for new pygtk: this is new class in PyGtk 2.4
if gtk.pygtk_version < (2,3,90):
   print "PyGtk 2.3.90 or later required for this example"
   raise SystemExit

dialog = None
if len(sys.argv)<5:
	print "Usage: python filechooser.py 'message' '*.jpg' . OPEN"
	raise SystemExit

if sys.argv[4]=='OPEN':
	dialog = gtk.FileChooserDialog(sys.argv[1],
								   None,
								   gtk.FILE_CHOOSER_ACTION_OPEN,
								   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
									gtk.STOCK_OPEN, gtk.RESPONSE_OK))
else:
	dialog = gtk.FileChooserDialog(sys.argv[1],
								   None,
								   gtk.FILE_CHOOSER_ACTION_SAVE,
								   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
									gtk.STOCK_OPEN, gtk.RESPONSE_OK))
dialog.set_default_response(gtk.RESPONSE_OK)
dialog.set_current_folder(sys.argv[3])
pattern=sys.argv[2]
patterns=[pattern]
filter = gtk.FileFilter()
filter.set_name(pattern)
if pattern.find('{')>0:
	patterns=pattern.split(',')
	for i in xrange(len(patterns)):
		pattern=patterns[i]
		if pattern.find('{')>0:
			idx=pattern.find('{')
			patterns[i]=pattern[:idx]+pattern[idx+1:]
		elif pattern.find('}')>0:
			idx=pattern.find('}')
			patterns[i]=pattern[:idx]+pattern[idx+1:]

for i in xrange(len(patterns)):
	pattern=patterns[i]
	if pattern.find('*')<0:
		pattern='*.'+pattern
	filter.add_pattern(pattern)
dialog.add_filter(filter)


response = dialog.run()
if response == gtk.RESPONSE_OK:
    print dialog.get_filename()
else:
    print 'Closed, no files selected'
dialog.destroy()
