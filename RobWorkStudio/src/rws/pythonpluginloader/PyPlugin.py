from sdurws import *
import sys
from PySide2 import QtCore

class cpp_link(QtCore.QObject):
    def __init__(self):
        self.plugin=None
    def register_plugin(self,aPlugin):
        if self.plugin == None:
            print("Plugin registered as: ", sys.argv[0])
            self.plugin=aPlugin
        else:
            raise Exception("Only One Plugin can be registered")       
    def stateChanged(self):
        stateChangedListener = getattr(self.plugin,"stateChangedListener",None)
        if callable(stateChangedListener):
            stateChangedListener(getRobWorkStudioFromQt().getState())
    def openWorkCell(self):
        open = getattr(self.plugin,"open",None)
        if callable(open):
            open(getRobWorkStudioFromQt().getWorkCell())
        pass
    def closeWorkCell(self):
        close = getattr(self.plugin,"close",None)
        if callable(close):
            close()

rws_cpp_link = cpp_link()

class rwsplugin(QtCore.QObject):
    def __init__(self,link):
        print("Initializing RWS python plugin")
        self.rws_cpp_link = link
        self.rwstudio = getRobWorkStudioFromQt()
    def getWidget(self):
        all_widgets = QtWidgets.QApplication.allWidgets()
        window = None
        for widget in all_widgets:
            if str(widget.objectName()) == sys.argv[0]:
                window = widget
                break  
        return window
    def getRobWorkStudio(self):
        return self.rwstudio
    def getWorkCell(self):
        return getRobWorkStudio().getWorkCell() 
    def log(self):
        return self.rwstudio.log()