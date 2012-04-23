# -*- coding: utf-8 -*-
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.SignalProxy import SignalProxy

import pyqtgraph.functions as fn
from math import log
from decimal import Decimal as D  ## Use decimal to avoid accumulating floating-point errors
from decimal import *
import weakref

__all__ = ['SpinBox']
class SpinBox(QtGui.QAbstractSpinBox):
    """QSpinBox widget on steroids. Allows selection of numerical value, with extra features:
      - SI prefix notation
      - Float values with linear and decimal stepping (1-9, 10-90, 100-900, etc.)
      - Option for unbounded values
      - Delayed signals (allows multiple rapid changes with only one change signal)
    """
    
    ## There's a PyQt bug that leaks a reference to the 
    ## QLineEdit returned from QAbstractSpinBox.lineEdit()
    ## This makes it possible to crash the entire program 
    ## by making accesses to the LineEdit after the spinBox has been deleted.
    ## I have no idea how to get around this..
    
    
    valueChanged = QtCore.Signal(object)     # (value)  for compatibility with QSpinBox
    sigValueChanged = QtCore.Signal(object)  # (self)
    sigValueChanging = QtCore.Signal(object, object)  # (self, value)  sent immediately; no delay.
    
    def __init__(self, parent=None, value=0.0, **kwargs):
        QtGui.QAbstractSpinBox.__init__(self, parent)
        self.lastValEmitted = None
        self.lastText = ''
        self.textValid = True  ## If false, we draw a red border
        self.setMinimumWidth(0)
        self.setMaximumHeight(20)
        self.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
        self.opts = {
            'bounds': [None, None],
            
            ## Log scaling options   #### Log mode is no longer supported.
            #'step': 0.1,
            #'minStep': 0.001,
            #'log': True,
            #'dec': False,
            
            ## decimal scaling option - example
            #'step': 0.1,    
            #'minStep': .001,    
            #'log': False,
            #'dec': True,
           
            ## normal arithmetic step
            'step': D('0.01'),  ## if 'dec' is false, the spinBox steps by 'step' every time
                                ## if 'dec' is True, the step size is relative to the value
                                ## 'step' needs to be an integral divisor of ten, ie 'step'*n=10 for some integer value of n (but only if dec is True)
            'log': False,
            'dec': False,   ## if true, does decimal stepping. ie from 1-10 it steps by 'step', from 10 to 100 it steps by 10*'step', etc. 
                            ## if true, minStep must be set in order to cross zero.
            
            
            'int': False, ## Set True to force value to be integer
            
            'suffix': '',
            'siPrefix': False,   ## Set to True to display numbers with SI prefix (ie, 100pA instead of 1e-10A)
            
            'delayUntilEditFinished': True,   ## do not send signals until text editing has finished
            
            ## for compatibility with QDoubleSpinBox and QSpinBox
            'decimals': 2
        }
        
        self.decOpts = ['step', 'minStep']
        
        self.val = D(unicode(value))  ## Value is precise decimal. Ordinary math not allowed.
        self.updateText()
        self.skipValidate = False
        self.setCorrectionMode(self.CorrectToPreviousValue)
        self.setKeyboardTracking(False)
        self.setOpts(**kwargs)
        
        
        self.editingFinished.connect(self.editingFinishedEvent)
        self.proxy = SignalProxy(self.sigValueChanging, slot=self.delayedChange)
        
    ##lots of config options, just gonna stuff 'em all in here rather than do the get/set crap.
    def setOpts(self, **opts):
        for k in opts:
            if k == 'bounds':
                #print opts[k]
                self.setMinimum(opts[k][0], update=False)
                self.setMaximum(opts[k][1], update=False)
                #for i in [0,1]:
                    #if opts[k][i] is None:
                        #self.opts[k][i] = None
                    #else:
                        #self.opts[k][i] = D(unicode(opts[k][i]))
            elif k in ['step', 'minStep']:
                self.opts[k] = D(unicode(opts[k]))
            elif k == 'value':
                pass   ## don't set value until bounds have been set
            else:
                self.opts[k] = opts[k]
        if 'value' in opts:
            self.setValue(opts['value'])
            
        ## If bounds have changed, update value to match
        if 'bounds' in opts and 'value' not in opts:
            self.setValue()   
            
        ## sanity checks:
        if self.opts['int']:
            step = self.opts['step']
            mStep = self.opts['minStep']
            if (int(step) != step) or (self.opts['dec'] and (int(mStep) != mStep)):
                raise Exception("Integer SpinBox may only have integer step and minStep.")
            
        self.updateText()



    def setMaximum(self, m, update=True):
        if m is not None:
            m = D(unicode(m))
        self.opts['bounds'][1] = m
        if update:
            self.setValue()
    
    def setMinimum(self, m, update=True):
        if m is not None:
            m = D(unicode(m))
        self.opts['bounds'][0] = m
        if update:
            self.setValue()
        
    def setPrefix(self, p):
        self.setOpts(prefix=p)
    
    def setRange(self, r0, r1):
        self.setOpts(bounds = [r0,r1])
        
    def setProperty(self, prop, val):
        """setProperty is just for compatibility with QSpinBox"""
        if prop == 'value':
            #if type(val) is QtCore.QVariant:
                #val = val.toDouble()[0]
            self.setValue(val)
        else:
            print "Warning: SpinBox.setProperty('%s', ..) not supported." % prop

    def setSuffix(self, suf):
        self.setOpts(suffix=suf)

    def setSingleStep(self, step):
        self.setOpts(step=step)
        
    def setDecimals(self, decimals):
        self.setOpts(decimals=decimals)

    def value(self):
        if self.opts['int']:
            return int(self.val)
        else:
            return float(self.val)

    def setValue(self, value=None, update=True, delaySignal=False):
        """
        Set the value of this spin. 
        If the value is out of bounds, it will be moved to the nearest boundary
        If the spin is integer type, the value will be coerced to int
        Returns the actual value set.
        
        If value is None, then the current value is used (this is for resetting
        the value after bounds, etc. have changed)
        """
        
        if value is None:
            value = self.value()
        
        bounds = self.opts['bounds']
        if bounds[0] is not None and value < bounds[0]:
            value = bounds[0]
        if bounds[1] is not None and value > bounds[1]:
            value = bounds[1]

        if self.opts['int']:
            value = int(value)

        value = D(unicode(value))
        if value == self.val:
            return
        prev = self.val
        
        self.val = value
        if update:
            self.updateText(prev=prev)
            
        self.sigValueChanging.emit(self, float(self.val))  ## change will be emitted in 300ms if there are no subsequent changes.
        if not delaySignal:
            self.emitChanged()
        
        return value

            
    def emitChanged(self):
        self.lastValEmitted = self.val
        self.valueChanged.emit(float(self.val))
        self.sigValueChanged.emit(self)
        
    def delayedChange(self):
        try:
            if self.val != self.lastValEmitted:
                self.emitChanged()
        except RuntimeError:
            pass  ## This can happen if we try to handle a delayed signal after someone else has already deleted the underlying C++ object.
        
    def widgetGroupInterface(self):
        return (self.valueChanged, SpinBox.value, SpinBox.setValue)
        
    def sizeHint(self):
        return QtCore.QSize(120, 0)
        
    
    def stepEnabled(self):
        return self.StepUpEnabled | self.StepDownEnabled        
    
    #def fixup(self, *args):
        #print "fixup:", args
    
    def stepBy(self, n):
        n = D(int(n))   ## n must be integral number of steps.
        s = [D(-1), D(1)][n >= 0]  ## determine sign of step
        val = self.val
        
        for i in range(abs(n)):
            
            if self.opts['log']:
                raise Exception("Log mode no longer supported.")
            #    step = abs(val) * self.opts['step']
            #    if 'minStep' in self.opts:
            #        step = max(step, self.opts['minStep'])
            #    val += step * s
            if self.opts['dec']:
                if val == 0:
                    step = self.opts['minStep']
                    exp = None
                else:
                    vs = [D(-1), D(1)][val >= 0]
                    #exp = D(int(abs(val*(D('1.01')**(s*vs))).log10()))
                    fudge = D('1.01')**(s*vs) ## fudge factor. at some places, the step size depends on the step sign.
                    exp = abs(val * fudge).log10().quantize(1, ROUND_FLOOR)
                    step = self.opts['step'] * D(10)**exp
                if 'minStep' in self.opts:
                    step = max(step, self.opts['minStep'])
                val += s * step
                #print "Exp:", exp, "step", step, "val", val
            else:
                val += s*self.opts['step']
                
            if 'minStep' in self.opts and abs(val) < self.opts['minStep']:
                val = D(0)
        self.setValue(val, delaySignal=True)  ## note all steps (arrow buttons, wheel, up/down keys..) emit delayed signals only.
        

    def valueInRange(self, value):
        bounds = self.opts['bounds']
        if bounds[0] is not None and value < bounds[0]:
            return False
        if bounds[1] is not None and value > bounds[1]:
            return False
        if self.opts.get('int', False):
            if int(value) != value:
                return False
        return True
        

    def updateText(self, prev=None):
        #print "Update text."
        self.skipValidate = True
        if self.opts['siPrefix']:
            if self.val == 0 and prev is not None:
                (s, p) = fn.siScale(prev)
                txt = "0.0 %s%s" % (p, self.opts['suffix'])
            else:
                txt = fn.siFormat(float(self.val), suffix=self.opts['suffix'])
        else:
            txt = '%g%s' % (self.val , self.opts['suffix'])
        self.lineEdit().setText(txt)
        self.lastText = txt
        self.skipValidate = False
        
    def validate(self, strn, pos):
        if self.skipValidate:
            #print "skip validate"
            #self.textValid = False
            ret = QtGui.QValidator.Acceptable
        else:
            try:
                ## first make sure we didn't mess with the suffix
                suff = self.opts.get('suffix', '')
                if len(suff) > 0 and unicode(strn)[-len(suff):] != suff:
                    #print '"%s" != "%s"' % (unicode(strn)[-len(suff):], suff)
                    ret = QtGui.QValidator.Invalid
                    
                ## next see if we actually have an interpretable value
                else:
                    val = self.interpret()
                    if val is False:
                        #print "can't interpret"
                        #self.setStyleSheet('SpinBox {border: 2px solid #C55;}')
                        #self.textValid = False
                        ret = QtGui.QValidator.Intermediate
                    else:
                        if self.valueInRange(val):
                            if not self.opts['delayUntilEditFinished']:
                                self.setValue(val, update=False)
                            #print "  OK:", self.val
                            #self.setStyleSheet('')
                            #self.textValid = True
                            
                            ret = QtGui.QValidator.Acceptable
                        else:
                            ret = QtGui.QValidator.Intermediate
                        
            except:
                #print "  BAD"
                #import sys
                #sys.excepthook(*sys.exc_info())
                #self.textValid = False
                #self.setStyleSheet('SpinBox {border: 2px solid #C55;}')
                ret = QtGui.QValidator.Intermediate
            
        ## draw / clear border
        if ret == QtGui.QValidator.Intermediate:
            self.textValid = False
        elif ret == QtGui.QValidator.Acceptable:
            self.textValid = True
        ## note: if text is invalid, we don't change the textValid flag 
        ## since the text will be forced to its previous state anyway
        self.update()
        
        ## support 2 different pyqt APIs. Bleh.
        if hasattr(QtCore, 'QString'):
            return (ret, pos)
        else:
            return (ret, strn, pos)
        
    def paintEvent(self, ev):
        QtGui.QAbstractSpinBox.paintEvent(self, ev)
        
        ## draw red border if text is invalid
        if not self.textValid:
            p = QtGui.QPainter(self)
            p.setRenderHint(p.Antialiasing)
            p.setPen(fn.mkPen((200,50,50), width=2))
            p.drawRoundedRect(self.rect().adjusted(2, 2, -2, -2), 4, 4)
            p.end()


    def interpret(self):
        """Return value of text. Return False if text is invalid, raise exception if text is intermediate"""
        strn = self.lineEdit().text()
        suf = self.opts['suffix']
        if len(suf) > 0:
            if strn[-len(suf):] != suf:
                return False
            #raise Exception("Units are invalid.")
            strn = strn[:-len(suf)]
        try:
            val = fn.siEval(strn)
        except:
            #sys.excepthook(*sys.exc_info())
            #print "invalid"
            return False
        #print val
        return val
        
    #def interpretText(self, strn=None):
        #print "Interpret:", strn
        #if strn is None:
            #strn = self.lineEdit().text()
        #self.setValue(siEval(strn), update=False)
        ##QtGui.QAbstractSpinBox.interpretText(self)
        
        
    def editingFinishedEvent(self):
        """Edit has finished; set value."""
        #print "Edit finished."
        if unicode(self.lineEdit().text()) == self.lastText:
            #print "no text change."
            return
        try:
            val = self.interpret()
        except:
            return
        
        if val is False:
            #print "value invalid:", str(self.lineEdit().text())
            return
        if val == self.val:
            #print "no value change:", val, self.val
            return
        self.setValue(val, delaySignal=False)  ## allow text update so that values are reformatted pretty-like
        
    #def textChanged(self):
        #print "Text changed."
        
        
### Drop-in replacement for SpinBox; just for crash-testing
#class SpinBox(QtGui.QDoubleSpinBox):
    #valueChanged = QtCore.Signal(object)     # (value)  for compatibility with QSpinBox
    #sigValueChanged = QtCore.Signal(object)  # (self)
    #sigValueChanging = QtCore.Signal(object)  # (value)
    #def __init__(self, parent=None, *args, **kargs):
        #QtGui.QSpinBox.__init__(self, parent)
    
    #def  __getattr__(self, attr):
        #return lambda *args, **kargs: None
        
    #def widgetGroupInterface(self):
        #return (self.valueChanged, SpinBox.value, SpinBox.setValue)
    
        
if __name__ == '__main__':
    import sys
    app = QtGui.QApplication([])
    
    def valueChanged(sb):
        #sb = QtCore.QObject.sender()
        print str(sb) + " valueChanged: %s" % str(sb.value())
    
    def valueChanging(sb, value):
        #sb = QtCore.QObject.sender()
        print str(sb) + " valueChanging: %s" % str(sb.value())
    
    def mkWin():
        win = QtGui.QMainWindow()
        g = QtGui.QFormLayout()
        w = QtGui.QWidget()
        w.setLayout(g)
        win.setCentralWidget(w)
        s1 = SpinBox(value=5, step=0.1, bounds=[-1.5, None], suffix='units')
        t1 = QtGui.QLineEdit()
        g.addRow(s1, t1)
        s2 = SpinBox(value=10e-6, dec=True, step=0.1, minStep=1e-6, suffix='A', siPrefix=True)
        t2 = QtGui.QLineEdit()
        g.addRow(s2, t2)
        s3 = SpinBox(value=1000, dec=True, step=0.5, minStep=1e-6, bounds=[1, 1e9], suffix='Hz', siPrefix=True)
        t3 = QtGui.QLineEdit()
        g.addRow(s3, t3)
        s4 = SpinBox(int=True, dec=True, step=1, minStep=1, bounds=[-10, 1000])
        t4 = QtGui.QLineEdit()
        g.addRow(s4, t4)

        win.show()
        
        import sys
        for sb in [s1, s2, s3,s4]:
            
            #QtCore.QObject.connect(sb, QtCore.SIGNAL('valueChanged(double)'), lambda v: sys.stdout.write(str(sb) + " valueChanged\n"))
            #QtCore.QObject.connect(sb, QtCore.SIGNAL('editingFinished()'), lambda: sys.stdout.write(str(sb) + " editingFinished\n"))
            sb.sigValueChanged.connect(valueChanged)
            sb.sigValueChanging.connect(valueChanging)
            sb.editingFinished.connect(lambda: sys.stdout.write(str(sb) + " editingFinished\n"))
        return win, w, [s1, s2, s3, s4]
    a = mkWin()
    
        
    def test(n=100):
        for i in range(n):
            win, w, sb = mkWin()
            for s in sb:
                w.setParent(None)
                s.setParent(None)
                s.valueChanged.disconnect()
                s.editingFinished.disconnect()
                
    ## Start Qt event loop unless running in interactive mode.
    if sys.flags.interactive != 1:
        app.exec_()
