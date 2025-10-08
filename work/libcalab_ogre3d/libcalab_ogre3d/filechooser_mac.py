#!/usr/bin/env python

# example filechooser.py
import Tkinter, tkFileDialog
import os,sys
if len(sys.argv)<5:
	print "Usage: python filechooser.py 'message' '*.jpg' . OPEN"
	raise SystemExit

root = Tkinter.Tk()
root.withdraw()

# Make it almost invisible - no decorations, 0 size, top left corner.
#root.overrideredirect(True)
#root.geometry('0x0+0+0')
#
## Show window again and lift it to top so it can get focus,
## otherwise dialogs will end up behind the terminal.
#root.deiconify()
#root.lift()
#root.focus_force()

file_opt = options =  {}
pattern=sys.argv[2]
mode=sys.argv[4]
patterns=[pattern]
options['filetypes'] = [('all files', '.*')]
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
    if pattern.count('.')>=2 :
        pattern=pattern[pattern.rfind('.'):]
    options['filetypes'].append((pattern[2:],pattern[1:]))
options['initialdir'] = os.path.abspath(sys.argv[3])
options['title'] = sys.argv[1]
options['parent'] = root
os.system('''/usr/bin/osascript -e 'tell app "Finder" to set frontmost of process "Python" to true' ''')
if mode=='OPEN':
    file_path = tkFileDialog.askopenfilename(**options)
else:
    file_path = tkFileDialog.asksaveasfile(mode='w', **options)
if file_path=="":
    print 'Closed, no files selected'
else:
    print(file_path)
