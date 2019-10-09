from PySide2 import QtWidgets
import sys

#find widget
def findWidget():
    all_widgets = QtWidgets.QApplication.allWidgets()
    window = None
    for widget in all_widgets:
        if str(widget.objectName()) == sys.argv[0]:
            window = widget
            break  
    return window
#done finding widget

def message_box():
    QtWidgets.QMessageBox.information(None, 'Test', 'Hello!',QtWidgets.QMessageBox.Ok)
    QtWidgets.QApplication.instance().quit()

#argv[0]= widgetName
#argv[1]= pluginName
def main():
    window = findWidget()                       # find the plugin widget
    l = window.layout()                         # get the layout
    button = QtWidgets.QPushButton('Press Me')  # make btn
    button.clicked.connect(message_box)         # connect btn with function
    l.addWidget(button,0,0)                     # add btn to layout

main()

#Adde the following lines to RobworkStudio.ini to load this plugin
#pluginName\DockArea=1
#pluginName\Filename=PythonPlugin
#pluginName\Path=/path/to/RobWork/RobWorkStudio/example/python
#pluginName\Visible=false