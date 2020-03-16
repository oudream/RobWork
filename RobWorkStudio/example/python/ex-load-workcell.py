from sdurws import *
from sdurw import *
import sys

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print("Usage : python3 "+sys.argv[0]+" <workcell> [OPTION]")
        print("Option: -t      for test run")    
    WC_FILE = str(sys.argv[1]) + "/scenes/SinglePA10Demo/SinglePA10Demo.wc.xml"


    rwstudio = getRobWorkStudioInstance()
    rwstudio.setWorkCell(WorkCellLoaderFactory.load(WC_FILE))

    while isRunning():
        if len(sys.argv) == 3 and sys.argv[2] == "-t":
            exit(0)