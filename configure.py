#!/usr/bin/env python
import subprocess

class bcolors:    
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def main():

    print bcolors.OKBLUE + bcolors.BOLD + """   ____ ______     ______    _   _   _    _     
  / ___|  _ \\ \\   / / ___|  | | | | / \\  | |    
 | |  _| |_) \\ \\ / / |      | | | |/ _ \\ | |    
 | |_| |  _ < \\ V /| |___   | |_| / ___ \\| |___ 
  \\____|_| \\_\\ \\_/  \\____|   \\___/_/   \\_\\_____|                                      
    """ + bcolors.ENDC
    print bcolors.BOLD + "> ual_backend_mavros installated in ros melodic docker image:" + bcolors.ENDC
    
    subprocess.call("sudo apt-get update", shell=True)
    subprocess.call("sudo apt-get install -y libeigen3-dev", shell=True)
    subprocess.call("sudo apt-get install -y ros-melodic-joy", shell=True)
    subprocess.call("sudo apt-get install -y ros-melodic-geodesy", shell=True)
    subprocess.call("sudo apt-get install -y ros-melodic-mavros", shell=True)
    subprocess.call("sudo apt-get install -y ros-melodic-mavros-extras", shell=True)
    subprocess.call("sudo apt-get install -y ros-melodic-tf2-geometry-msgs", shell=True)
    subprocess.call("sudo geographiclib-get-geoids egm96-5", shell=True)
    subprocess.call("sudo usermod -a -G dialout $USER", shell=True)
    subprocess.call("sudo apt-get remove modemmanager", shell=True)
    subprocess.call("sudo apt-get install -y libyaml-cpp-dev", shell=True)
    subprocess.call("sudo apt-get install -y net-tools", shell=True)


if __name__ == "__main__":
    main()
