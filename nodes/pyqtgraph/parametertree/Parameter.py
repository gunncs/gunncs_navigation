from pyqtgraph.Qt import QtGui, QtCore
import collections, os, weakref, re
from ParameterItem import ParameterItem

PARAM_TYPES = {}


def registerParameterType(name, cls, override=False):
    global PARAM_TYPES
    if name in PARAM_TYPES and not override:
        raise Exception("Parameter type '%s' already exists (use override=True to replace)" % name)
    PARAM_TYPES[name] = cls



class Parameter(QtCore.QObject):
    """Tree of name=value pairs (modifiable or not)
       - Value may be integer, float, string, bool, color, or list selection
       - Optionally, a custom widget may be specified for a property
       - Any number of extra columns may be added for other purposes
       - Any values may be reset to a default value
       - Parameters may be grouped / nested
       
    For more Parameter types, see ParameterTree.parameterTypes module.
    """
    ## name, type, limits, etc.
    ## can also carry UI hints (slider vs spinbox, etc.)
    
    sigValueChanged = QtCore.Signal(object, object)  ## self, value   emitted when value is finished being edited
    sigValueChanging = QtCore.Signal(object, object)  ## self, value  emitted as value is being edited
    
    sigChildAdded = QtCore.Signal(object, object, object)  ## self, child, index
    sigChildRemoved = QtCore.Signal(object, object)  ## self, child
    sigParentChanged = QtCore.Signal(object, object)  ## self, parent
    sigLimitsChanged = QtCore.Signal(object, object)  ## self, limits
    sigDefaultChanged = QtCore.Signal(object, object)  ## self, default
    sigNameChanged = QtCore.Signal(object, object)  ## self, name
    sigOptionsChanged = QtCore.Signal(object, object)  ## self, {opt:val, ...}
    
    ## Emitted when anything changes about this parameter at all.
    ## The second argument is a string indicating what changed ('value', 'childAdded', etc..)
    ## The third argument can be any extra information about the change
    sigStateChanged = QtCore.Signal(object, object, object) ## self, change, info
    
    ## emitted when any child in the tree changes state
    ## (but only if monitorChildren() is called)
    sigTreeStateChanged = QtCore.Signal(object, object)  # self, changes
                                                         # changes = [(param, change, info), ...]
    
    # bad planning.
    #def __new__(cls, *args, **opts):
        #try:
            #cls = PARAM_TYPES[opts['type']]
        #except KeyError:
            #pass
        #return QtCore.QObject.__new__(cls, *args, **opts)
    
    @staticmethod
    def create(**opts):
        """
        Create a new Parameter (or subclass) instance using opts['type'] to select the 
        appropriate class.
        
        Use registerParameterType() to add new class types.
        """
        cls = PARAM_TYPES[opts['type']]
        return cls(**opts)
    
    def __init__(self, **opts):
        QtCore.QObject.__init__(self)
        
        self.opts = {
            'readonly': False,
            'visible': True,
            'enabled': True,
            'renamable': False,
            'removable': False,
            'strictNaming': False,  # forces name to be usable as a python variable
        }
        self.opts.update(opts)
        
        self.childs = []
        self.names = {}   ## map name:child
        self.items = weakref.WeakKeyDictionary()  ## keeps track of tree items representing this parameter
        self._parent = None
        self.treeStateChanges = []  ## cache of tree state changes to be delivered on next emit
        self.blockTreeChangeEmit = 0
        #self.monitoringChildren = False  ## prevent calling monitorChildren more than once
        
        if 'value' not in self.opts:
            self.opts['value'] = None
        
        if 'name' not in self.opts or not isinstance(self.opts['name'], basestring):
            raise Exception("Parameter must have a string name specified in opts.")
        self.setName(opts['name'])
        
        for chOpts in self.opts.get('children', []):
            #print self, "Add child:", type(chOpts), id(chOpts)
            self.addChild(chOpts)
            
        if 'value' in self.opts and 'default' not in self.opts:
            self.opts['default'] = self.opts['value']
    
        ## Connect all state changed signals to the general sigStateChanged
        self.sigValueChanged.connect(lambda param, data: self.emitStateChanged('value', data))
        self.sigChildAdded.connect(lambda param, *data: self.emitStateChanged('childAdded', data))
        self.sigChildRemoved.connect(lambda param, data: self.emitStateChanged('childRemoved', data))
        self.sigParentChanged.connect(lambda param, data: self.emitStateChanged('parent', data))
        self.sigLimitsChanged.connect(lambda param, data: self.emitStateChanged('limits', data))
        self.sigDefaultChanged.connect(lambda param, data: self.emitStateChanged('default', data))
        self.sigNameChanged.connect(lambda param, data: self.emitStateChanged('name', data))
        self.sigOptionsChanged.connect(lambda param, data: self.emitStateChanged('options', data))
        
        #self.watchParam(self)  ## emit treechange signals if our own state changes
        
    def name(self):
        return self.opts['name']

    def setName(self, name):
        """Attempt to change the name of this parameter; return the actual name. 
        (The parameter may reject the name change or automatically pick a different name)"""
        if self.opts['strictNaming']:
            if len(name) < 1 or re.search(r'\W', name) or re.match(r'\d', name[0]):
                raise Exception("Parameter name '%s' is invalid. (Must contain only alphanumeric and underscore characters and may not start with a number)" % name)
        parent = self.parent()
        if parent is not None:
            name = parent._renameChild(self, name)  ## first ask parent if it's ok to rename
        if self.opts['name'] != name:
            self.opts['name'] = name
            self.sigNameChanged.emit(self, name)
        return name

    def childPath(self, child):
        """Return the path of parameter names from self to child."""
        path = []
        while child is not self:
            path.insert(0, child.name())
            child = child.parent()
        return path

    def setValue(self, value, blockSignal=None):
        ## return the actual value that was set
        ## (this may be different from the value that was requested)
        #print self, "Set value:", value, self.opts['value'], self.opts['value'] == value
        try:
            if blockSignal is not None:
                self.sigValueChanged.disconnect(blockSignal)
            if self.opts['value'] == value:
                return value
            self.opts['value'] = value
            self.sigValueChanged.emit(self, value)
        finally:
            if blockSignal is not None:
                self.sigValueChanged.connect(blockSignal)
            
        return value

    def value(self):
        return self.opts['value']

    def getValues(self):
        """Return a tree of all values that are children of this parameter"""
        vals = collections.OrderedDict()
        for ch in self:
            vals[ch.name()] = (ch.value(), ch.getValues())
        return vals
    
    def saveState(self):
        """Return a structure representing the entire state of the parameter tree."""
        state = self.opts.copy()
        state['children'] = {ch.name(): ch.saveState() for ch in self}
        return state

    def defaultValue(self):
        return self.opts['default']
        
    def setDefault(self, val):
        self.opts['default'] = val
        self.sigDefaultChanged.emit(self, val)

    def setToDefault(self):
        if self.hasDefault():
            self.setValue(self.defaultValue())

    def hasDefault(self):
        return 'default' in self.opts
        
    def valueIsDefault(self):
        return self.value() == self.defaultValue()
        
    def setLimits(self, limits):
        if 'limits' in self.opts and self.opts['limits'] == limits:
            return
        self.opts['limits'] = limits
        self.sigLimitsChanged.emit(self, limits)
        return limits

    def writable(self):
        return not self.opts.get('readonly', False)

    def setOpts(self, **opts):
        """For setting any arbitrary options."""
        changed = collections.OrderedDict()
        for k in opts:
            if k == 'value':
                self.setValue(opts[k])
            elif k == 'name':
                self.setName(opts[k])
            elif k == 'limits':
                self.setLimits(opts[k])
            elif k == 'default':
                self.setDefault(opts[k])
            elif k not in self.opts or self.opts[k] != opts[k]:
                self.opts[k] = opts[k]
                changed[k] = opts[k]
                
        if len(changed) > 0:
            self.sigOptionsChanged.emit(self, changed)
        
    def emitStateChanged(self, changeDesc, data):
        ## Emits stateChanged signal and 
        ## requests emission of new treeStateChanged signal
        self.sigStateChanged.emit(self, changeDesc, data)
        #self.treeStateChanged(self, changeDesc, data)
        self.treeStateChanges.append((self, changeDesc, data))
        self.emitTreeChanges()

    def makeTreeItem(self, depth):
        """Return a TreeWidgetItem suitable for displaying/controlling the content of this parameter.
        Most subclasses will want to override this function.
        """
        if hasattr(self, 'itemClass'):
            #print "Param:", self, "Make item from itemClass:", self.itemClass
            return self.itemClass(self, depth)
        else:
            return ParameterItem(self, depth=depth)


    def addChild(self, child):
        """Add another parameter to the end of this parameter's child list."""
        return self.insertChild(len(self.childs), child)
        
    def insertChild(self, pos, child):
        """Insert a new child at pos.
        If pos is a Parameter, then insert at the position of that Parameter.
        If child is a dict, then a parameter is constructed as Parameter(**child)
        """
        if isinstance(child, dict):
            child = Parameter.create(**child)
        
        name = child.name()
        if name in self.names:
            if child.opts.get('autoIncrementName', False):
                name = self.incrementName(name)
                child.setName(name)
            else:
                raise Exception("Already have child named %s" % str(name))
        if isinstance(pos, Parameter):
            pos = self.childs.index(pos)
            
        if child.parent() is not None:
            child.remove()
            
        self.names[name] = child
        self.childs.insert(pos, child)
        
        child.parentChanged(self)
        self.sigChildAdded.emit(self, child, pos)
        child.sigTreeStateChanged.connect(self.treeStateChanged)
        return child
        
    def removeChild(self, child):
        name = child.name()
        if name not in self.names or self.names[name] is not child:
            raise Exception("Parameter %s is not my child; can't remove." % str(child))
        
        del self.names[name]
        self.childs.pop(self.childs.index(child))
        child.parentChanged(None)
        self.sigChildRemoved.emit(self, child)
        child.sigTreeStateChanged.disconnect(self.treeStateChanged)

    def clearChildren(self):
        for ch in self.childs[:]:
            self.removeChild(ch)

    def children(self):  
        ## warning -- this overrides QObject.children
        return self.childs[:]

    def parentChanged(self, parent):
        self._parent = parent
        self.sigParentChanged.emit(self, parent)
        
    def parent(self):
        return self._parent
        
    def remove(self):
        """Remove self from parent's child list"""
        parent = self.parent()
        if parent is None:
            raise Exception("Cannot remove; no parent.")
        parent.removeChild(self)

    def incrementName(self, name):
        ## return an unused name by adding a number to the name given
        base, num = re.match('(.*)(\d*)', name).groups()
        numLen = len(num)
        if numLen == 0:
            num = 2
            numLen = 1
        else:
            num = int(num)
        while True:
            newName = base + ("%%0%dd"%numLen) % num
            if newName not in self.childs:
                return newName
            num += 1

    def __iter__(self):
        for ch in self.childs:
            yield ch

    def __getitem__(self, names):
        """Get the value of a child parameter"""
        if not isinstance(names, tuple):
            names = (names,)
        return self.param(*names).value()

    def __setitem__(self, names, value):
        """Set the value of a child parameter"""
        if isinstance(names, basestring):
            names = (names,)
        return self.param(*names).setValue(value)

    def param(self, *names):
        """Return a child parameter. 
        Accepts the name of the child or a tuple (path, to, child)"""
        try:
            param = self.names[names[0]]
        except KeyError:
            raise Exception("Parameter %s has no child named %s" % (self.name(), names[0]))
        
        if len(names) > 1:
            return param.param(*names[1:])
        else:
            return param
        
    def __repr__(self):
        return "<%s '%s' at 0x%x>" % (self.__class__.__name__, self.name(), id(self))
       
    def __getattr__(self, attr):
        #print type(self), attr
        if attr in self.names:
            return self.param(attr)
        else:
            raise AttributeError(attr)
       
    def _renameChild(self, child, name):
        ## Only to be called from Parameter.rename
        if name in self.names:
            return child.name()
        self.names[name] = child
        del self.names[child.name()]
        return name

    def registerItem(self, item):
        self.items[item] = None
        
    def hide(self):
        self.show(False)
        
    def show(self, s=True):
        self.opts['visible'] = s
        self.sigOptionsChanged.emit(self, {'visible': s})


    #def monitorChildren(self):
        #if self.monitoringChildren:
            #raise Exception("Already monitoring children.")
        #self.watchParam(self)
        #self.monitoringChildren = True

    #def watchParam(self, param):
        #param.sigChildAdded.connect(self.grandchildAdded)
        #param.sigChildRemoved.connect(self.grandchildRemoved)
        #param.sigStateChanged.connect(self.grandchildChanged)
        #for ch in param:
            #self.watchParam(ch)

    #def unwatchParam(self, param):
        #param.sigChildAdded.disconnect(self.grandchildAdded)
        #param.sigChildRemoved.disconnect(self.grandchildRemoved)
        #param.sigStateChanged.disconnect(self.grandchildChanged)
        #for ch in param:
            #self.unwatchParam(ch)

    #def grandchildAdded(self, parent, child):
        #self.watchParam(child)
        
    #def grandchildRemoved(self, parent, child):
        #self.unwatchParam(child)
        
    #def grandchildChanged(self, param, change, data):
        ##self.sigTreeStateChanged.emit(self, param, change, data)
        #self.emitTreeChange((param, change, data))

    def treeChangeBlocker(self):
        """
        Return an object that can be used to temporarily block and accumulate
        sigTreeStateChanged signals. This is meant to be used when numerous changes are 
        about to be made to the tree and only one change signal should be
        emitted at the end.
        
        Example:
            with param.treeChangeBlocker():
                param.addChild(...)
                param.removeChild(...)
                param.setValue(...)
        """
        return SignalBlocker(self.blockTreeChangeSignal, self.unblockTreeChangeSignal)

    def blockTreeChangeSignal(self):
        """
        Used to temporarily block and accumulate tree change signals.
        *You must remember to unblock*, so it is advisable to use treeChangeBlocker() instead.
        """
        self.blockTreeChangeEmit += 1

    def unblockTreeChangeSignal(self):
        """Unblocks enission of sigTreeStateChanged and flushes the changes out through a single signal."""
        self.blockTreeChangeEmit -= 1
        self.emitTreeChanges()
        
        
    def treeStateChanged(self, param, changes):
        """
        Called when the state of any sub-parameter has changed. 
        Arguments:
            param: the immediate child whose tree state has changed.
                   note that the change may have originated from a grandchild.
            changes: list of tuples describing all changes that have been made
                     in this event: (param, changeDescr, data)
                     
        This function can be extended to react to tree state changes.
        """
        self.treeStateChanges.extend(changes)
        self.emitTreeChanges()
    
    def emitTreeChanges(self):
        if self.blockTreeChangeEmit == 0:
            changes = self.treeStateChanges
            self.treeStateChanges = []
            self.sigTreeStateChanged.emit(self, changes)


class SignalBlocker:
    def __init__(self, enterFn, exitFn):
        self.enterFn = enterFn
        self.exitFn = exitFn
        
    def __enter__(self):
        self.enterFn()
        
    def __exit__(self, exc_type, exc_value, tb):
        self.exitFn()
    
    
    