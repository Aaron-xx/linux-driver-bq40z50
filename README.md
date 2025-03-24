以下是适合你的 `README.md` 和 `.gitignore` 文件内容：  

### `README.md`
```markdown
# bq40z50 Linux 驱动

本仓库包含适用于 Linux 内核的 bq40z50 电池管理 IC 驱动代码。

## 目录结构
```
```
.
├── bq40z50_fg.c        # 主要的驱动源代码
├── bq40z50-overlay.dts # 设备树覆盖文件
├── Makefile            # 编译驱动的 Makefile
└── test.sh             # 测试脚本
```
```

## 编译和安装
### 1. 获取 Linux 内核源码
确保你已经安装了对应的 Linux 内核源码，并在 `Makefile` 中正确设置了 `KDIR` 变量。

### 2. 编译驱动
```sh
make && sudo cp bq40z50.ko /lib/modules/$(uname -r)/kernel/drivers/power/ && sudo depmod
```

### 3. 加载驱动
```sh
sudo modprobe bq40z50_fg.ko
```

### 4. 卸载驱动
```sh
sudo rmmod bq40z50_fg
```

## 设备树配置
如果你的系统使用设备树，请将 `bq40z50-overlay.dts` 编译为 `.dtbo` 并加载：

```sh
dtc -@ -I dts -O dtb -o bq40z50-overlay.dtbo bq40z50-overlay.dts
sudo cp bq40z50-overlay.dtbo /boot/overlays/
echo "dtoverlay=bq40z50-overlay" | sudo tee -a /boot/config.txt
```

## 测试
运行测试脚本：
```sh
chmod +x test.sh
./test.sh
```

## 许可证
本项目基于 **GPLv2** 许可证，详情请查看 LICENSE 文件。
```
