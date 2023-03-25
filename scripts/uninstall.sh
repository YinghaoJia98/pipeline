#init
scripts_current_path=$(dirname `readlink -f $0`)
package_path=$(dirname $scripts_current_path)
package_name=$(basename $package_path)
user_path=/home/$(whoami)

#installation path
relative_path="deeprobotics_perception/user_development_ws/src"
installation_path=~/$relative_path

#main
if [ "$package_path" != "$installation_path/$package_name" ]
then
  echo -e "\033[31m Error:\033[0m 'uninstall.sh' use path error!"
  echo ""
  exit
fi
rm -rf $installation_path/$package_name
echo "<$package_name> was uninstalled successfully." 