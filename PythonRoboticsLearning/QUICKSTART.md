# 🚀 个人学习参考手册

> **这是你的个人快速参考**，专门给你查阅用，AI 不需要了解

## 📍 学习环境

**位置**：`/home/zhufeng/code/PythonRobotics/PythonRoboticsLearning/`

**两个目录**：
- `../PythonRobotics/` ← 原始算法库（只读参考）
- `./` ← 你的学习目录（笔记和进度）

## ⚡ 快速启动

### 开始学习（3步骤）

```
1. 对 Claude 说："我想学习 [算法名] 算法"
2. 通过问答理解原理和代码
3. 结束时说："请根据我们关于X的讨论，创建笔记"
```

### 常用学习短语

```
# 开始学习
"我想学习 A* 算法"
"我想理解 RRT 的工作原理"
"讲解一下 Kalman Filter"

# 深入了解
"分析 RRT 算法的核心循环逻辑"
"RRT 和 RRT* 的区别是什么？"
"这个参数的作用是什么？"

# 创建笔记
"请根据我们上面关于A*的讨论，创建笔记"
```

## 📂 文件速查

### 重要文件位置
```
../CLAUDE.md               ← AI 指导文档（完整）
README.md                  ← 学习进度追踪
notes/[算法名]_algorithm.md ← 讨论笔记（自动创建）
```

### 算法文件位置
```
../PythonRobotics/PathPlanning/
├── AStar/a_star.py
├── RRT/rrt.py
├── RRTStar/rrt_star.py
├── Dijkstra/dijkstra.py
└── ...

../PythonRobotics/PathTracking/
├── LQRSteerControl/lqr_steer_control.py
├── PurePursuit/pure_pursuit.py
└── ...
```

## 💡 高效学习技巧

### ✅ 好的提问方式
- "我想理解 A* 算法的启发式函数"
- "能举个具体例子吗？"
- "我不太明白这个地方"

### ❌ 避免的提问
- "告诉我 A* 的一切"
- 只说"好的"、"理解了"

### 📝 笔记创建时机
- ✅ 讨论深入理解核心概念后
- ✅ 代码分析完成后
- ✅ 算法对比完成后
- ❌ 刚开始就创建（会很空洞）

## 🔥 学习状态速查

### 当前进度
```bash
# 查看学习进度
cat README.md

# 查看已有笔记
ls notes/

# 查看特定笔记
cat notes/rrt_algorithm.md
```

### 算法分类速查
```
🔴 路径规划：A*, RRT, RRT*, Dijkstra, Hybrid A*
🟡 路径跟踪：LQR, Pure Pursuit, Stanley, MPC
🟢 定位估计：Kalman Filter, EKF, Particle Filter
🔵 SLAM：EKF-SLAM, FastSLAM, Graph-based SLAM
```

## 🆘 快速解决

### 遇到问题时

**Q: 不知道学什么算法？**
```bash
# 查看可用算法
ls ../PythonRobotics/PathPlanning/
```

**Q: 忘了某个算法学到哪了？**
```bash
# 查看学习记录
grep -A3 -B1 "算法名" README.md
```

**Q: 想回顾之前的讨论？**
```bash
# 查看对应笔记
cat notes/[算法名]_algorithm.md
```

**Q: 学习进度混乱？**
```
直接问 Claude："我之前学过哪些算法了？"
```

## 🎯 个人学习偏好

### 我的学习习惯
- [ ] 喜欢从简单算法开始（A*, Dijkstra）
- [ ] 喜欢先理解数学原理再看代码
- [ ] 喜欢算法对比学习
- [ ] 其他：_____________

### 常用算法清单
- [ ] A* - 启发式搜索基础
- [ ] RRT - 采样算法基础
- [ ] RRT* - 最优采样
- [ ] Hybrid A* - 车辆路径规划
- [ ] LQR - 最优控制基础
- [ ] Kalman Filter - 状态估计基础

---

**这是你的个人手册，随时更新！📚**