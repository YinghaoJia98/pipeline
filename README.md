# 当前版本:
V1.3.5

**使用事项**
​    1. 使用升级包时请关闭所有的终端，以防止出现意料之外的错误
    2. 在pipeline/scripts目录下找到install.sh脚本，打开新终端运行该脚本即可实现更新
    3. 本次pipeline更新对使用方法做出了调整，详情请见pipeline使用手册
    运行install.sh出现报错处理：
    请检查感知环境是否为COS_V1i，否则升级脚本install.sh会出现报错，当前版本COS_V1i的建图主目录为：/home/ysc/deeprobotics_perception/deeprobotics_mapping_localization_ws
    如果无法找到上述建图目录或无法区分感知系统版本，可以通过复制粘贴手动替换更新包，手动更新时请注意备份。





# 一、install.sh使用说明
# 使用条件：当用户需要对pipeline进行安装或升级时使用，运行install.sh能够实现功能包的安装或升级。
# install.sh运行方法：
进入到pipeline更新包scripts目录下，打开终端运行以下命令
./install.sh

# 二、uninstall.sh使用说明
# 使用条件：当用户需要对已有的pipeline功能包进行卸载时使用，运行uninstall.shh能够实现对pipeline功能包的卸载。
# pack.sh运行方法：
进入到pipeline更新包scripts目录下，打开终端运行以下命令
./uninstall.sh

# 三、pack.sh使用说明
# 使用条件：当用户需要对已有的pipeline功能包进行打包时使用，运行pack.sh能够实现将pipeline功能包的完成打包至/home/ysc/目录下，同时生成对应的解包工具。
# pack.sh运行方法：
进入到pipeline更新包scripts目录下，打开终端运行以下命令
./pack.sh


