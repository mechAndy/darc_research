import os
from subprocess import call, STDOUT

rootDir = '/home/nailbunny/catkin_ws/src'
dirs = os.listdir(rootDir)

for file in dirs:
    tempPath = os.path.join(rootDir,file)
    if os.path.isdir(tempPath):
        gitPath = os.path.join(tempPath,".git")
        if os.path.isdir(gitPath):
            print "---------------"
            print file
            print "---------------"
            os.chdir(tempPath)
            os.system("git status -s")
#    tempPath = os.path.join(rootDir,file)   
#    if os.path.isdir(tempPath):
#        print tempPath
#        os.chdir(tempPath)
#        os.system("git status -s")

os.chdir("/home/nailbunny/catkin_ws")


