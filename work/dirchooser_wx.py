import wx
import os, sys, pdb
if len(sys.argv)<3:
	print("Usage: python dirchooser.py 'message' '../src'")
	raise SystemExit

def printPath(path):
    if path==None:
        print('Closed, no files selected')
    else:
        print(path)
def get_path(title,  d):
    app = wx.App(None)
    d=os.path.abspath(d)
    dialog = wx.DirDialog (None, title, d,
                    wx.DD_DEFAULT_STYLE | wx.DD_DIR_MUST_EXIST)

    if dialog.ShowModal() == wx.ID_OK:
        path = dialog.GetPath()
    else:
        path = None
    dialog.Destroy()
    return path

printPath(get_path(sys.argv[1], sys.argv[2]))

