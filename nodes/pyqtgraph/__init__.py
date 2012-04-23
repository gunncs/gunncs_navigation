# -*- coding: utf-8 -*-
### import all the goodies and add some helper functions for easy CLI use

## 'Qt' is a local module; it is intended mainly to cover up the differences
## between PyQt4 and PySide.
from Qt import QtGui 

## not really safe.
#if QtGui.QApplication.instance() is None:
    #app = QtGui.QApplication([])

## in general openGL is poorly supported in Qt. 
## we only enable it where the performance benefit is critical.
## Note this only applies to 2D graphics; 3D graphics always use OpenGL.
import sys

## check python version
if sys.version_info[0] != 2 or sys.version_info[1] != 7:
    raise Exception("Pyqtgraph requires Python version 2.7 (this is %d.%d)" % (sys.version_info[0], sys.version_info[1]))

if 'linux' in sys.platform:  ## linux has numerous bugs in opengl implementation
    useOpenGL = False
elif 'darwin' in sys.platform: ## openGL greatly speeds up display on mac
    useOpenGL = True
else:
    useOpenGL = False  ## on windows there's a more even performance / bugginess tradeoff. 
                
CONFIG_OPTIONS = {
    'useOpenGL': useOpenGL,   ## by default, this is platform-dependent (see widgets/GraphicsView). Set to True or False to explicitly enable/disable opengl.
    'leftButtonPan': True  ## if false, left button drags a rubber band for zooming in viewbox
}

def setConfigOption(opt, value):
    CONFIG_OPTIONS[opt] = value

def getConfigOption(opt):
    return CONFIG_OPTIONS[opt]


## Rename orphaned .pyc files. This is *probably* safe :)

def renamePyc(startDir):
    ### Used to rename orphaned .pyc files
    ### When a python file changes its location in the repository, usually the .pyc file
    ### is left behind, possibly causing mysterious and difficult to track bugs. 
    
    printed = False
    startDir = os.path.abspath(startDir)
    for path, dirs, files in os.walk(startDir):
        for f in files:
            fileName = os.path.join(path, f)
            base, ext = os.path.splitext(fileName)
            py = base + ".py"
            if ext == '.pyc' and not os.path.isfile(py):
                if not printed:
                    print "NOTE: Renaming orphaned .pyc files:"
                    printed = True
                n = 1
                while True:
                    name2 = fileName + ".renamed%d" % n
                    if not os.path.exists(name2):
                        break
                    n += 1
                print "  " + fileName + "  ==>"
                print "  " + name2
                os.rename(fileName, name2)
                
import os
path = os.path.split(__file__)[0]
renamePyc(path)


## Import almost everything to make it available from a single namespace
## don't import the more complex systems--canvas, parametertree, flowchart, dockarea
## these must be imported separately.

def importAll(path, excludes=()):
    d = os.path.join(os.path.split(__file__)[0], path)
    files = []
    for f in os.listdir(d):
        if os.path.isdir(os.path.join(d, f)):
            files.append(f)
        elif f[-3:] == '.py' and f != '__init__.py':
            files.append(f[:-3])
        
    for modName in files:
        if modName in excludes:
            continue
        mod = __import__(path+"."+modName, globals(), locals(), fromlist=['*'])
        if hasattr(mod, '__all__'):
            names = mod.__all__
        else:
            names = [n for n in dir(mod) if n[0] != '_']
        for k in names:
            if hasattr(mod, k):
                globals()[k] = getattr(mod, k)

importAll('graphicsItems')
importAll('widgets', excludes=['MatplotlibWidget'])

from imageview import *
from WidgetGroup import *
from Point import Point
from Transform import Transform
from functions import *
from graphicsWindows import *
from SignalProxy import *




## Convenience functions for command-line use



plots = []
images = []
QAPP = None

def plot(*args, **kargs):
    """
    Create and return a :class:`PlotWindow <pyqtgraph.PlotWindow>` 
    (this is just a window with :class:`PlotWidget <pyqtgraph.PlotWidget>` inside), plot data in it.
    Accepts a *title* argument to set the title of the window.
    All other arguments are used to plot data. (see :func:`PlotItem.plot() <pyqtgraph.PlotItem.plot>`)
    """
    mkQApp()
    #if 'title' in kargs:
        #w = PlotWindow(title=kargs['title'])
        #del kargs['title']
    #else:
        #w = PlotWindow()
    #if len(args)+len(kargs) > 0:
        #w.plot(*args, **kargs)
        
    pwArgList = ['title', 'label', 'name', 'left', 'right', 'top', 'bottom']
    pwArgs = {}
    dataArgs = {}
    for k in kargs:
        if k in pwArgList:
            pwArgs[k] = kargs[k]
        else:
            dataArgs[k] = kargs[k]
        
    w = PlotWindow(**pwArgs)
    w.plot(*args, **dataArgs)
    plots.append(w)
    w.show()
    return w
    
def image(*args, **kargs):
    """
    Create and return an :class:`ImageWindow <pyqtgraph.ImageWindow>` 
    (this is just a window with :class:`ImageView <pyqtgraph.ImageView>` widget inside), show image data inside.
    Will show 2D or 3D image data.
    Accepts a *title* argument to set the title of the window.
    All other arguments are used to show data. (see :func:`ImageView.setImage() <pyqtgraph.ImageView.setImage>`)
    """
    mkQApp()
    w = ImageWindow(*args, **kargs)
    images.append(w)
    w.show()
    return w
show = image  ## for backward compatibility
    
    
def mkQApp():
    if QtGui.QApplication.instance() is None:
        global QAPP
        QAPP = QtGui.QApplication([])
        
        
