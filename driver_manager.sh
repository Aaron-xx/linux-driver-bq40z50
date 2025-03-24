#!/bin/bash

# 加载 bq40z50 驱动模块
modprobe_bq40z50() {
    echo "正在加载 bq40z50 驱动模块..."
    insmod ./bq40z50.ko
    if [ $? -eq 0 ]; then
        echo "模块加载成功"
    else
        echo "模块加载失败"
        exit 1
    fi
}

# 卸载 bq40z50 驱动模块
rmmod_bq40z50() {
    echo "正在卸载 bq40z50 驱动模块..."
    rmmod bq40z50
    if [ $? -eq 0 ]; then
        echo "模块卸载成功"
    else
        echo "模块卸载失败"
        exit 1
    fi
}

# 根据命令行参数执行相应操作
case "$1" in
    load)
        modprobe_bq40z50
        ;;
    unload)
        rmmod_bq40z50
        ;;
    reload)
        rmmod_bq40z50 || true
        modprobe_bq40z50
        ;;
    *)
        echo "用法: $0 {load|unload|reload}"
        exit 1
        ;;
esac

