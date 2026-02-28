# PythonRobotics 学习笔记

> **学习开始日期**: 2026-01-08
> **目标**: 系统学习机器人算法（路径规划、定位、SLAM、控制）

---

## 📂 目录结构

```
/home/zhufeng/code/PythonRobotics/
├── PythonRobotics/              # 原始算法库项目（只读）
│   ├── PathPlanning/
│   ├── PathTracking/
│   └── ...
│
└── PythonRoboticsLearning/      # 本学习目录（读写）
    ├── notes/                   # 学习笔记（Markdown 格式）
    │   ├── 01_astar_algorithm.md
    │   ├── 02_dijkstra.md
    │   └── ...
    ├── experiments/             # 实验代码和结果
    │   ├── astar_variants/
    │   ├── rrt_comparison/
    │   └── ...
    ├── code_snippets/           # 有用代码片段
    │   ├── visualization_tools.py
    │   └── test_utils.py
    └── README.md                # 本文件
```

---

## 📚 学习进度

### ✅ 已完成

| 算法 | 笔记 | 实验 | 理解程度 | 日期 |
|------|------|------|----------|------|
| A* | ✅ | ⏳ | ⭐⭐⭐⭐ | 2026-01-08 |
| RRT | ✅ | ✅ | ⭐⭐⭐⭐ | 2026-01-20 |
| RRT* | ✅ | ✅ | ⭐⭐⭐⭐⭐ | 2026-01-21 |
| Hybrid A* | ✅ | ✅ | ⭐⭐⭐⭐⭐ | 2026-01-23 |
| Dijkstra | ✅ | ⏳ | ⭐⭐⭐⭐⭐ | 2026-01-23 |
| BFS/DFS | ✅ | ⏳ | ⭐⭐⭐⭐⭐ | 2026-01-23 |
| Greedy Best First | ✅ | ⏳ | ⭐⭐⭐⭐⭐ | 2026-01-23 |
| LQR Steering Control | ✅ | ⏳ | ⭐⭐⭐⭐⭐ | 2026-01-24 |
| Pure Pursuit | ✅ | ⏳ | ⭐⭐⭐⭐ | 2026-01-24 |
| **MPC** | ✅ | ⏳ | ⭐⭐⭐⭐⭐ | **2026-02-09** |

### 🚧 进行中

| 算法 | 当前任务 | 下一步 |
|------|---------|--------|
| - | - | - |

### 📅 待学习

| 优先级 | 算法分类 | 算法列表 |
|--------|---------|---------|
| 🔴 高 | 状态估计 | Kalman Filter, Extended Kalman Filter, Particle Filter |
| 🟡 中 | SLAM | EKF-SLAM, FastSLAM |
| 🟢 低 | 高级路径规划 | Informed RRT*, Anytime RRT*, Bi-RRT |

---

## 🎯 学习目标

### 短期目标（1 个月）
- [ ] 完成基础网格搜索算法学习（5 个算法）
- [ ] 理解启发式搜索原理
- [ ] 能修改代码适配新场景

### 中期目标（3 个月）
- [ ] 掌握采样算法（RRT 系列）
- [ ] 理解路径跟踪控制
- [ ] 学习状态估计滤波器

### 长期目标（6 个月）
- [ ] 实现 SLAM 完整流程
- [ ] 能组合多个算法解决实际问题
- [ ] 优化算法性能

---

## 📖 学习资源

### 项目资源
- **主项目**: `/home/zhufeng/code/PythonRobotics/PythonRobotics/`
- **在线文档**: https://atsushisakai.github.io/PythonRobotics/
- **算法总览**: `PythonRobotics/README.md`
- **父目录说明**: `../CLAUDE.md` (重要！阅读本目录结构的完整说明)

### 推荐书籍
- 《Probabilistic Robotics》- Thrun et al.
- 《Planning Algorithms》- LaValle
- 《Autonomous Mobile Robots》- Siegwart

### 在线资源
- [Red Blob Games - Pathfinding](https://www.redblobgames.com/)
- [Amit's A* Pages](https://www.redblobgames.com/pathfinding/a-star/)

---

## 🎓 交互式学习指南

### 开始学习

直接告诉 Claude 你想学什么：

```
"我想学习 [算法名] 算法"
```

**Claude 会：**
1. 通过问答确认你的基础知识
2. 分步讲解算法原理
3. 结合代码分析实现细节
4. 确保每个知识点都理解了再继续

### 创建笔记

学习讨论结束后，你可以说：

```
"请根据我们上面关于[算法名]的讨论，创建笔记"
```

**笔记特点：**
- 位置：`notes/[算法名]_algorithm.md`
- 同一算法只有一个文件
- 支持多次会话累积更新
- 包含讨论总结和核心要点

---

## 📊 学习统计

### 代码阅读量
- 本周: 1 个算法（RRT）
- 累计: 2 个算法（A*, RRT）
- 目标: 每周 2-3 个算法

### 实验完成度
- 已完成: 1 个（RRT 参数实验）
- 进行中: 0 个
- 计划中: 5 个

### 笔记质量
- 详细度: ⭐⭐⭐⭐⭐
- 代码分析: ⭐⭐⭐⭐
- 实验记录: ⭐⭐⭐

---

## 💡 交互式学习技巧

### 1. 高效问答方式
```
✅ 明确学习目标："我想理解 A* 算法的启发式函数"
✅ 主动参与讨论：用自己的话解释概念
✅ 及时澄清疑问："我不太理解这个地方"
❌ 避免："告诉我一切" 或 只说"好的"
```

### 2. 笔记创建时机
- ✅ **讨论深入后**：理解了核心概念
- ✅ **对比完成后**：比较了多个算法
- ✅ **代码分析后**：理解了实现细节
- ❌ **刚开始就要**：避免过早创建空洞笔记

---

## 🔗 快速链接

### 已创建的笔记
- [A* 算法笔记](./notes/01_astar_algorithm.md)
- [RRT 算法笔记](./notes/03_rrt_algorithm.md)
- [RRT* 算法笔记](./notes/04_rrt_star_algorithm.md)
- [Hybrid A* 算法笔记](./notes/05_hybrid_a_star_algorithm.md)
- [LQR 转向控制笔记](./notes/03_lqr_steering_control.md)
- [Pure Pursuit 跟踪笔记](./notes/06_pure_pursuit_tracking.md)
- [**MPC 模型预测控制笔记**](./notes/07_model_predictive_control.md) ← **最新**

---

## 📝 学习日志

### 2026-01-08
- ✅ 创建学习目录结构
- ✅ 运行 A* 算法演示
- ✅ 完成 A* 算法详细笔记
- 📌 理解了启发式搜索的核心思想
- 📌 掌握了 A* 的代码结构

### 2026-01-20
- ✅ 完成 RRT 算法学习
- ✅ 创建详细的算法笔记（原理、代码分析、实验）
- ✅ 编写参数影响研究实验
- 📌 发现 expand_dis 对效率影响显著：步长越大，节点数越少，速度越快
- 📌 发现 goal_sample_rate 对性能影响较小，但 20% 时路径较短
- 📌 理解了采样-based 算法与搜索算法的本质区别

### 2026-01-23
- ✅ Hybrid A* 深度复习
- ✅ 完全理解成本函数（换向 vs 倒车）
- ✅ 理解网格化与收敛的本质
- ✅ 掌握"混合"的完整架构
- ✅ 补全基础搜索算法理解（Dijkstra, BFS, DFS, Greedy）
- ✅ 创建 Hybrid A* 参数影响实验
  - `hybrid_astar_param_demo.py`: 可调参数测试工具
  - `hybrid_astar_quick_test.py`: 快速对比测试
  - `hybrid_astar_interactive_demo.py`: 交互式可视化
  - `HYBRID_ASTAR_README.md`: 详细使用说明
- 📌 **关键洞察**：
  - 离散搜索 ≠ 多邻居扩展
  - 收敛 ≠ 有解
  - Hybrid = 网格去重 + 精确存储
- 📌 **实验发现**：
  - 在简单场景中,参数对路径影响不大
  - 默认参数 (SB=100, BACK=5) 已经很合理
  - 需要复杂场景才能体现参数差异

### 2026-01-21
- ✅ 完成 RRT* 算法学习
- ✅ 创建详细的算法笔记（原理、代码分析、核心机制）
- ✅ 编写 RRT vs RRT* 对比实验
- 📌 **完全理解**了 RRT* 的两大核心机制：
  - Choose Parent：为新节点选择最优父节点
  - Rewire：新节点优化邻近节点（"过继"机制）
- 📌 理解了渐近最优的实现原理
- 📌 掌握了成本传播的递归机制
- 📌 实验验证：RRT* 比 RRT 路径改进 4.4%

### 2026-02-09
- ✅ 完成 MPC（模型预测控制）深度学习
- ✅ 创建超详细的算法笔记（32页，涵盖理论到实现）
- ✅ 通过互动式教学完全掌握 MPC 核心概念
- 📌 **完全理解**了 MPC 三步曲：预测-优化-执行
- 📌 掌握了迭代线性MPC的数学推导和实现机制
- 📌 理解了车辆运动学建模和线性化过程
- 📌 掌握了CVXPY优化框架和约束处理
- 📌 理解了热启动机制和收敛判断
- 📌 **关键洞察**：
  - MPC = 滚动优化 + 约束处理 + 多步预测
  - 线性化精度与计算效率的平衡
  - 权重矩阵调节影响控制性能
  - 迭代机制提高非线性系统控制精度

### 待更新...

---

## 🎓 证书与成果

### 完成的项目
- 待添加

### 获得的技能
- ✅ 理解 A* 启发式搜索
- ✅ 能阅读和分析算法代码
- ⏳ 路径跟踪控制
- ⏳ SLAM 算法实现

---

## 🤝 贡献与分享

### 学习心得
如果你也在学习 PythonRobotics，欢迎：
- 分享你的学习笔记
- 讨论算法原理
- 交换实验心得

### 推荐方式
- GitHub Issues
- 技术博客
- 学习小组

---

## 📞 联系方式

- **学习目录**: `/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/`
- **主项目**: `/home/zhufeng/code/PythonRobotics/PythonRobotics/`
- **GitHub**: https://github.com/AtsushiSakai/PythonRobotics
- **目录结构说明**: 请查看父目录 `/home/zhufeng/code/PythonRobotics/CLAUDE.md`

---

**祝学习愉快！🚀**
