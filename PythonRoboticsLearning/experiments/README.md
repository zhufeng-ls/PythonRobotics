# PythonRobotics 实验目录

本目录包含算法学习和实验的代码文件。

## 📁 目录结构

```
experiments/
├── hybrid_astar_param_demo.py         # Hybrid A* 参数测试工具
├── hybrid_astar_interactive_demo.py   # Hybrid A* 交互式界面
├── HYBRID_ASTAR_README.md             # Hybrid A* 使用说明
├── rrt_vs_rrt_star/                    # RRT vs RRT* 对比实验
├── rrt_comparison/                     # RRT 参数研究
└── ...                                 # 其他实验
```

## 🔬 可用的实验工具

### Hybrid A* 算法

#### 1. 参数测试工具 ⭐
```bash
# 使用默认参数
python3 hybrid_astar_param_demo.py

# 自定义参数
python3 hybrid_astar_param_demo.py --sb 100 --back 5 --steer 5

# 批量测试
python3 hybrid_astar_param_demo.py --batch
```

**功能**: 测试 SB_COST, BACK_COST, STEER_CHANGE_COST 对路径规划的影响

**详细文档**: [HYBRID_ASTAR_README.md](HYBRID_ASTAR_README.md)

---

### RRT 系列算法

#### RRT 参数研究
```bash
cd rrt_comparison
python3 rrt_parameter_study.py
```

#### RRT vs RRT* 对比
```bash
cd rrt_vs_rrt_star
python3 comparison.py
```

---

## 📝 实验记录模板

创建新实验时,建议包含:

1. **实验目的**: 要验证什么?
2. **测试参数**: 哪些变量?
3. **评估指标**: 如何衡量?
4. **结果分析**: 数据说明了什么?
5. **结论**: 有什么发现?

---

## 🚀 创建新实验

参考现有实验的结构:

1. 导入必要的模块
2. 定义测试场景
3. 实现参数调节
4. 运行算法
5. 记录结果
6. 可视化输出
7. 编写说明文档

---

**更多实验陆续添加中...** 🧪
