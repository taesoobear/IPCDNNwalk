import wx
import os, sys, pdb
if len(sys.argv)<5:
	print("Usage: python filechooser.py 'message' '*.jpg' . OPEN")
	raise SystemExit

def printPath(path):
    if path==None:
        print('Closed, no files selected')
    else:
        print(path)
def get_path(wildcard, s, d):
    app = wx.App(None)
    if s==wx.FD_OPEN:
        style = s | wx.FD_FILE_MUST_EXIST
    else:
        style = s 
    dialog = wx.FileDialog(None, 'Open', wildcard=wildcard, style=style, defaultDir=d)
    if dialog.ShowModal() == wx.ID_OK:
        path = dialog.GetPath()
    else:
        path = None
    dialog.Destroy()
    return path

pattern=sys.argv[2]
patterns=[pattern]
npattern=''
if pattern.find('{')>0:
        patterns=pattern.split(',')
        for i in range(len(patterns)):
                pattern=patterns[i]
                if pattern.find('{')>0:
                        idx=pattern.find('{')
                        patterns[i]=pattern[:idx]+pattern[idx+1:]
                elif pattern.find('}')>0:
                        idx=pattern.find('}')
                        patterns[i]=pattern[:idx]+pattern[idx+1:]


for i in range(len(patterns)):
        pattern=patterns[i]
        if pattern.find('*')<0:
                pattern='*.'+pattern

        if npattern=='':
            npattern=pattern
        else:
            npattern=npattern+"|"+pattern
if sys.argv[4]=='OPEN':
    printPath(get_path(npattern,wx.FD_OPEN, sys.argv[3]))
else:
    printPath(get_path(npattern,wx.FD_SAVE, sys.argv[3]))

