# 🔤 matplotlib中文字体安装指南

## 🐛 问题现象

运行LQR学习示例时出现大量字体警告：
```
UserWarning: Glyph 29366 (\N{CJK UNIFIED IDEOGRAPH-72B6}) missing from font(s) DejaVu Sans.
```

图表中的中文显示为方框□或问号？

## 🔧 解决方案

### 方案1: 安装中文字体 (推荐)

#### Ubuntu/Debian系统:
```bash
# 安装文泉驿字体
sudo apt update
sudo apt install fonts-wqy-microhei fonts-wqy-zenhei

# 或者安装Noto中文字体
sudo apt install fonts-noto-cjk

# 清除matplotlib字体缓存
rm -rf ~/.cache/matplotlib
```

#### CentOS/RHEL/Fedora系统:
```bash
# Fedora/CentOS 8+
sudo dnf install wqy-microhei-fonts wqy-zenhei-fonts

# 或者安装Google Noto字体
sudo dnf install google-noto-sans-cjk-ttc-fonts

# 清除matplotlib字体缓存
rm -rf ~/.cache/matplotlib
```

#### Arch Linux:
```bash
# 安装文泉驿字体
sudo pacman -S wqy-microhei wqy-zenhei

# 或者安装Noto字体
sudo pacman -S noto-fonts-cjk

# 清除matplotlib字体缓存
rm -rf ~/.cache/matplotlib
```

### 方案2: 手动下载字体

如果无法通过包管理器安装，可以手动下载：

```bash
# 创建字体目录
mkdir -p ~/.fonts

# 下载文泉驿微米黑字体
wget https://github.com/googlefonts/noto-cjk/releases/download/Sans2.004/04_NotoSansCJKsc.zip
unzip 04_NotoSansCJKsc.zip -d ~/.fonts/

# 或者下载文泉驿字体
wget http://sourceforge.net/projects/wqy/files/wqy-microhei/0.2.0-beta/wqy-microhei-0.2.0-beta.tar.gz
tar -xzf wqy-microhei-0.2.0-beta.tar.gz
cp wqy-microhei/wqy-microhei.ttc ~/.fonts/

# 更新字体缓存
fc-cache -fv
rm -rf ~/.cache/matplotlib
```

### 方案3: 使用conda安装

```bash
# 在conda环境中安装字体包
conda activate python_robotics
conda install -c conda-forge fonts-anaconda
```

## 🧪 验证安装

安装完成后，运行以下脚本验证：

```python
import matplotlib.pyplot as plt
import matplotlib
print("Available fonts:")
for font in matplotlib.font_manager.findSystemFonts():
    if 'wqy' in font.lower() or 'noto' in font.lower():
        print(f"  {font}")

# 测试中文显示
plt.figure(figsize=(6, 4))
plt.text(0.5, 0.5, '中文字体测试 - LQR控制器', 
         fontsize=16, ha='center', va='center')
plt.title('字体测试')
plt.show()
```

## 🚀 快速测试修复

重新运行LQR示例：

```bash
cd PathTracking/lqr_steer_control
conda activate python_robotics
python lqr_example_simple.py
```

如果看到：
- ✅ `中文字体设置成功: WenQuanYi Micro Hei` - 字体安装成功
- ⚠️ `未找到支持中文的字体，将使用英文标签` - 程序将自动使用英文标签

## 🔄 故障排除

### 问题1: 安装后仍显示英文
```bash
# 清除所有matplotlib缓存
rm -rf ~/.cache/matplotlib
rm -rf ~/.matplotlib

# 重启Python
python -c "import matplotlib.pyplot; print('Cache cleared')"
```

### 问题2: 权限问题
```bash
# 如果遇到权限问题，使用用户目录
mkdir -p ~/.local/share/fonts
cp your-font.ttf ~/.local/share/fonts/
fc-cache -fv ~/.local/share/fonts
```

### 问题3: Docker/容器环境
```dockerfile
# 在Dockerfile中添加
RUN apt-get update && apt-get install -y \
    fonts-wqy-microhei \
    fonts-wqy-zenhei \
    && rm -rf /var/lib/apt/lists/*
```

## 📋 支持的字体列表

### Linux推荐字体:
- **WenQuanYi Micro Hei** (文泉驿微米黑) - 最佳选择
- **WenQuanYi Zen Hei** (文泉驿正黑)
- **Noto Sans CJK SC** (Google Noto简体中文)
- **Source Han Sans CN** (思源黑体)

### macOS推荐字体:
- **PingFang SC** (苹方简体)
- **Hiragino Sans GB** (冬青黑体简体中文)
- **STHeiti** (华文黑体)

### Windows推荐字体:
- **Microsoft YaHei** (微软雅黑)
- **SimHei** (黑体)
- **SimSun** (宋体)

## 💡 最佳实践

1. **优先使用系统包管理器安装字体** - 更稳定可靠
2. **安装后清除matplotlib缓存** - 确保字体被识别
3. **验证安装效果** - 运行测试脚本确认
4. **如果仍有问题** - 程序会自动切换到英文标签，不影响学习

---
🎨 **安装完成后，您就可以愉快地使用中文界面学习LQR了！** 🎨 