# 手动中文字体配置说明

## 🎯 简单配置方式

现在使用简单的手动配置方式，在需要中文显示的Python脚本中直接添加字体配置代码。

## 📝 配置方法

### 在Python脚本开头添加以下代码：

```python
import matplotlib.pyplot as plt

# 配置中文字体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
```

## 🚀 使用方法

### 方法1：直接添加到脚本中
```python
import matplotlib.pyplot as plt
import numpy as np

# 配置中文字体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 你的绘图代码
fig, ax = plt.subplots()
ax.plot([1, 2, 3], [1, 4, 2])
ax.set_xlabel('X坐标')
ax.set_ylabel('Y坐标')
ax.set_title('中文标题')
plt.show()
```

### 方法2：创建配置函数
```python
def setup_chinese_font():
    """配置中文字体"""
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

# 在脚本开头调用
setup_chinese_font()
```

## 📋 字体优先级

1. **Microsoft YaHei** (微软雅黑) - 优先选择
2. **Noto Sans CJK SC** - Google Noto字体
3. **WenQuanYi Micro Hei** (文泉驿微米黑) - 开源字体
4. **DejaVu Sans** - 备用字体

## ✅ 验证配置

运行测试脚本验证配置：
```bash
python3 bresenham_visualization.py
```

## 🔧 故障排除

### 问题1：仍然显示乱码
**解决方案**：
1. 检查字体是否安装：`fc-list :lang=zh`
2. 尝试其他字体：`plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans']`

### 问题2：负号显示异常
**解决方案**：
```python
plt.rcParams['axes.unicode_minus'] = False
```

### 问题3：图形显示异常
**解决方案**：
```python
# 清除matplotlib缓存
import matplotlib
matplotlib.font_manager._rebuild()
```

## 📝 示例脚本

### 完整示例
```python
#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

# 配置中文字体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def main():
    # 创建测试数据
    x = np.linspace(0, 2*np.pi, 100)
    y = np.sin(x)
    
    # 创建图形
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(x, y, 'b-', linewidth=2, label='正弦波')
    ax.set_xlabel('X坐标')
    ax.set_ylabel('Y坐标')
    ax.set_title('中文字体测试')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 添加中文注释
    ax.annotate('这是中文注释', xy=(np.pi/2, 1), xytext=(np.pi, 0.5),
                arrowprops=dict(arrowstyle='->', color='red'),
                fontsize=12, color='red')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
```

## 🎯 优势

- ✅ **简单直接**：只需添加几行代码
- ✅ **灵活控制**：可以针对不同脚本使用不同配置
- ✅ **易于理解**：配置逻辑清晰明了
- ✅ **无副作用**：不影响其他项目或系统配置

---

*配置方式：手动配置*  
*支持系统：Linux (Ubuntu/Debian)*  
*Python版本：3.x*  
*matplotlib版本：3.x*
