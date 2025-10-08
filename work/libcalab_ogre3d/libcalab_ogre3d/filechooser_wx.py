# uses wx or tkinter
import os, sys, pdb
if len(sys.argv)<5:
	print("Usage: python filechooser.py 'message' '*.jpg' . OPEN")
	raise SystemExit

try:
    import wx
except:
    # use tkinter instead
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()

    file_opt = options = {}
    pattern = sys.argv[2]
    mode = sys.argv[4]
    patterns = [pattern]
    options['filetypes'] = [('all files', '.*')]

    if '{' in pattern:
        patterns = pattern.split(',')
        for i in range(len(patterns)):
            pattern = patterns[i]
            if '{' in pattern:
                idx = pattern.find('{')
                patterns[i] = pattern[:idx] + pattern[idx+1:]
            elif '}' in pattern:
                idx = pattern.find('}')
                patterns[i] = pattern[:idx] + pattern[idx+1:]

    for i in range(len(patterns)):
        pattern = patterns[i]
        if '*' not in pattern:
            pattern = '*.' + pattern
        if pattern.count('.') >= 2:
            pattern = pattern[pattern.rfind('.'):]
        options['filetypes'].append((pattern[2:], pattern[1:]))

    options['initialdir'] = os.path.abspath(sys.argv[3])
    options['title'] = sys.argv[1]
    options['parent'] = root

    if mode == 'OPEN':
        file_path = filedialog.askopenfilename(**options)
    else:
        file_path = filedialog.asksaveasfilename(**options)

    if not file_path:
        print('Closed, no files selected')
    else:
        print(file_path)
    exit()


def printPath(path):
    if path==None:
        print('Closed, no files selected')
    else:
        print(path)
def get_path(title, wildcard, s, d):
    app = wx.App(None)
    if s==wx.FD_OPEN:
        style = s | wx.FD_FILE_MUST_EXIST
    else:
        style = s 
    dialog = wx.FileDialog(None, title, wildcard=wildcard, style=style, defaultDir=d)
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
        pattern=pattern.split('.')[-1] # work-around a bug in wxWidget.
        if pattern.find('*')<0:
                pattern='*.'+pattern

        if npattern=='':
            npattern=pattern
        else:
            npattern=npattern+";"+pattern
if sys.argv[4]=='OPEN':
    printPath(get_path(sys.argv[1]+' '+sys.argv[2], npattern,wx.FD_OPEN, sys.argv[3]))
else:
    printPath(get_path(sys.argv[1]+' '+sys.argv[2], npattern,wx.FD_SAVE, sys.argv[3]))

