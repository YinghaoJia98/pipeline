#! /bin/bash

#init
function_package_name="pipeline"
scripts_current_path=$(dirname `readlink -f $0`)
package_path=$(dirname $scripts_current_path)
package_name=$(basename $package_path)
temp_path=$(dirname $package_path)

#pack path
user_path=/home/$(whoami)
pack_path=$user_path

#version
if [ "$package_name" != "$function_package_name" ]
then
  package_name=$function_package_name
fi
version=$package_path/version
pipeline_version=$(sed -n 1p $version)
package_version=$package_name"-"$pipeline_version

#unpack.sh package name init
unpack_sh="$package_version.tar.gz-unpack.sh"
unpack_sh_path=$scripts_current_path/$unpack_sh


#mian
function creat_unpack() {
  cd $pack_path
  touch $unpack_sh
  chmod +x $unpack_sh
  cat > $unpack_sh << EOF
#! /bin/bash

#mian
if [ ! -e $package_version.tar.gz]
then
  echo "<$package_version.tar.gz> does not exist!"
fi
tar zxvf $package_version.tar.gz 
rm -rf $package_version.tar.gz-unpack.sh
EOF
}


tar zcvf $pack_path/$package_version.tar.gz  -C $temp_path $package_name

creat_unpack

echo "<$package_name> packed successfully."
echo ""
echo $package_version
