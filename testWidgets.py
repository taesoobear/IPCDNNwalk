# this file contains a line-by-line python port of testWidgets.lua (taesooLib/Samples/classification/lua/)
import os,sys, pdb, math, random, copy, code
from libcalab_ogre3d import *

def ctor(this):
    this.create("Button", "a", "a")
    this.create("Check_Button", "b", "b")
    this.widget(0).checkButtonValue(False)

    this.create("Light_Button", "c", "c")
    this.create("Check_Button", "cc", "cc")
    this.create("Value_Slider", "hip angle", "hip angle",1);
    this.widget(0).sliderRange(-60,60);
    this.widget(0).sliderValue(0);
    this.create("Choice", "n_th triangle","", 0)
    if True:
        # low-level api
        this.widget(0).menuSize(3)
        this.widget(0).menuItem(0, "0_th triangle", "FL_ALT+0")
        this.widget(0).menuItem(1, "1_th triangle")
        this.widget(0).menuItem(2, "2_th triangle")
        this.widget(0).menuValue(0)

    this.create("Box", "asdf", "widget position and size")
    this.setUniformGuidelines(10) # total 10 slots
    this.create("Button", "e", "e", 0,5)
    this.widget(0).buttonShortcut("FL_ALT+e")
    this.create("Button", "f", "f", 5,7)
    this.widget(0).buttonShortcut("FL_ALT+f")
    this.create("Button", "g", "g", 7)
    this.widget(0).buttonShortcut("FL_ALT+g")
    this.setWidgetHeight(40)
    this.create("Button", "h", "h")
    this.create("Button", "i", "i", 0)

    this.setWidgetHeight(100);

    maxSlider=5 # numkey
    this.setUniformGuidelines(maxSlider)
    for i in range(0, maxSlider):
        slider_id="slider"+str(i)
        this.create('Vert_Slider', slider_id, '', i, i+1);
    this.resetToDefault()
    this.addButton("test progress bar") # same as create("Button", "test progress bar", "test progress bar")
    this.create("Progress", "percent")
    this.widget(0).clearVisible()
    this.create("Input", "input1", "")
    this.widget(0).inputValue('input1 text')
    this.setWidgetHeight(30)
    this.create("Multiline_Input", "input2", "")
    this.widget(0).inputValue('input2 text\nsecond line ')
    this.resetToDefault()
    this.addButton('print input2')


    this.setWidgetHeight(100)
    this.create("Multi_Browser", "body parts", "body parts") # or you can instead use Select_Browser for single select.
    if True:
        mBrowser=this.widget(0)
        mBrowser.browserClear()
        for i in range(1,11):
            mBrowser.browserAdd(str(i))

    this.resetToDefault()
    this.updateLayout()




def drawText(a):
    textheight=30
    RE.draw("Text", m.vector3(0,textheight+10, 0), "textfield0", m.vector3(0,0,0), textheight, a)

def onCallback(w, userData):
    this=m.getPythonWin()
    if w.id()=="b" or w.id()=="c":
        drawText(w.id()+ " "+str(w.checkButtonValue()))
    elif w.id()=="test progress bar":
        w=this.findWidget('percent')
        w.setVisible()
        for i in range(100) :
            w.progressValue(i)
            RE.renderOneFrame(True) # necessary. 이때 frameMove, onFrameChanged도 호출됨!
            print(i)
            m.usleep(20000)
    elif w.id()=="hip angle" :
        drawText("hip angle "+ str(w.sliderValue()))
    elif w.id()=="input1" :
        drawText(w.inputValue())
    elif w.id()=='print input2':
        drawText(this.findWidget('input2').inputValue())
    elif w.id()[0:6]=="slider" :
        drawText(f"{w.id()} {w.sliderValue()}")
    elif w.id()=="n_th triangle" :
        drawText("choice "+ str(w.menuValue())+" "+w.menuText())
    elif w.id()=="body parts" :
        drawText("browser "+str(w.browserValue())+w.id())
        # you can also you use bool w.browserSelected(int i), where
        # i>=0 and i< w.browserSize()
    else:
        drawText(w.id())

            
def main():

    RE.createMainWin(sys.argv)
    ctor(m.getPythonWin())
    print('ctor finished')
    m.startMainLoop() # this finishes when program finishes

if __name__=="__main__":
    main()
