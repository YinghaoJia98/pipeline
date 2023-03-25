#!/bin/bash

#init
function_package_name="pipeline"
backup_package="backup_package"
scripts_current_path=$(dirname `readlink -f $0`)
package_path=$(dirname $scripts_current_path)
package_name=$(basename $package_path)
user_path=/home/$(whoami)

#installation path
relative_path="deeprobotics_perception/user_development_ws/src"
installation_path=~/$relative_path

#version
if [ "$package_name" != "$function_package_name" ]
then
  package_name=$function_package_name
fi
version=$package_path/version
pipeline_version=$(sed -n 1p $version)
package_version=$package_name"-"$pipeline_version


#main
if [ ! -e $installation_path ]
then
  echo -e "\033[31m Error:\033[0m $installation_path path does not exist!"
  exit
fi

if [ "$package_path" == "$installation_path/$package_name" ]
then
  echo -e "\033[31m Error:\033[0m 'install.sh' user path error!"
  echo ""
  exit
fi

if [ ! -e $user_path/$backup_package ]
then
    mkdir $user_path/$backup_package
fi

echo "Start installing the <$package_name> package."
echo "<$package_name> installation path: $installation_path"
echo ""
if [ -e "$installation_path/$package_name" ]
then
    echo "<$package_name> already exists and will be backuped..."
    mv "$installation_path/$package_name" "$user_path/$backup_package/$package_name".bak.$(date +"%F_%T")
    echo "backup succeeded."
    echo " "
fi

echo "<$package_name> is installing..."
cp -r $package_path $installation_path
chmod -R 777 $installation_path/$package_name/*

cd $(dirname $installation_path)
catkin_make --pkg  ${package_name}


echo "<$package_name> was installed successfully." 
echo ""
echo $package_version