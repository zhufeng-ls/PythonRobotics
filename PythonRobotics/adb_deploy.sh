#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 工作目录
WORKSPACE_DIR="$HOME/code/mower_ws_rv1126_develop"
INSTALL_PACKAGE="$WORKSPACE_DIR/install.tgz"
INSTALL_DIR="$WORKSPACE_DIR/install"
DEVICE_INSTALL_DIR="/userdata/install"
TARGET_INSTALL_DIR="/oem/bin/install"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ADB 设备安装包推送脚本${NC}"
echo -e "${GREEN}========================================${NC}"

# 1. 拉取最新代码
echo -e "\n${YELLOW}[1/7] 拉取最新代码...${NC}"
cd "$WORKSPACE_DIR"/src/mower || exit 1
git pull origin develop
if [ $? -ne 0 ]; then
    echo -e "${RED}Git pull 失败！${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 代码拉取完成${NC}"

cd ..
cd ..

# 2. 编译项目
echo -e "\n${YELLOW}[2/7] 编译项目...${NC}"
./jim/build.sh build rv1126 -j4 mower_x9
if [ $? -ne 0 ]; then
    echo -e "${RED}编译失败！${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 编译完成${NC}"

# 3. 打包安装包
echo -e "\n${YELLOW}[3/7] 打包安装包...${NC}"
if [ -d "$INSTALL_DIR" ]; then
    rm -f "$INSTALL_PACKAGE"
    tar -zcvf "$INSTALL_PACKAGE" -C "$WORKSPACE_DIR" install
    if [ $? -ne 0 ]; then
        echo -e "${RED}打包失败！${NC}"
        exit 1
    fi
    PACKAGE_SIZE=$(du -h "$INSTALL_PACKAGE" | cut -f1)
    echo -e "${GREEN}✓ 打包完成 (大小: $PACKAGE_SIZE)${NC}"
else
    echo -e "${RED}install 目录不存在！${NC}"
    exit 1
fi

# 4. 检查 adb 设备连接
echo -e "\n${YELLOW}[4/7] 检查 ADB 设备连接...${NC}"
adb devices
DEVICE_COUNT=$(adb devices | grep -w "device" | wc -l)
if [ "$DEVICE_COUNT" -eq 0 ]; then
    echo -e "${RED}未检测到 ADB 设备！${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ADB 设备已连接${NC}"

# 5. 推送安装包到设备
echo -e "\n${YELLOW}[5/7] 推送安装包到设备...${NC}"
adb push "$INSTALL_PACKAGE" /userdata/
if [ $? -ne 0 ]; then
    echo -e "${RED}推送失败！${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 安装包推送完成${NC}"

# 6. 解压安装包
echo -e "\n${YELLOW}[6/7] 解压安装包...${NC}"
adb shell "cd /userdata && tar -zxvf install.tgz"
if [ $? -ne 0 ]; then
    echo -e "${RED}解压失败！${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 安装包解压完成${NC}"

# 7. 关闭旧进程并安装
echo -e "\n${YELLOW}[7/7] 关闭旧进程并安装...${NC}"
adb shell "tmux kill-server"
adb shell "rm -rf $TARGET_INSTALL_DIR && mv $DEVICE_INSTALL_DIR $TARGET_INSTALL_DIR"
if [ $? -ne 0 ]; then
    echo -e "${RED}安装失败！${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 安装完成${NC}"

# 清理临时文件
echo -e "\n${YELLOW}清理临时文件...${NC}"
adb shell "rm -f /userdata/install.tgz"

# 完成
echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}  所有操作完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}安装路径: $TARGET_INSTALL_DIR${NC}"
