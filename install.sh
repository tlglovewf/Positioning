#!/usr/bin/env sh 
#check if root user
CurUser=$(id -u)
if test "$CurUser" != "0" ; then
        echo "To run script as administrator (user "root"), use sudo ./install.sh"
        exit
fi

commandtype=$1
commandinstall=install
commanduninstall=uninstall
echo "command:${commandtype}"

echo "Check the program complile status!"

if [ ! -d "build" ];then
    echo "compile sources first."
    sh build.sh
else
    echo "build dir found!!! begin to un/install."
fi

doInstall(){
    echo "install program."
    cd build
    sudo make install
}

doUninstall(){
    echo "uninstall program"
    cd build
    sudo make uninstall
}


if [ -z $commandtype ];then
    doInstall
else
    if [ $commandtype = $commandinstall ] ; then
        doInstall
    elif [ $commandtype = $commanduninstall ] ; then
        doUninstall
    else
        doInstall
    fi
fi