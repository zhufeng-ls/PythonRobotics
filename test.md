结论与你的笔记的差别
按照上述推导，导数等于 0 的方程应该是这样的： $$\frac{\partial J}{\partial \mathbf{u}} = 2\mathbf{B}^T \mathbf{Q}_f \mathbf{A}\mathbf{x} + 2(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B}) \mathbf{u} = 0$$

你再看你笔记里的公式： $$\frac{\partial J}{\partial \mathbf{u}} = 2\mathbf{x}^T \mathbf{A}^T \mathbf{Q}_f \mathbf{B} + 2(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B}) \mathbf{u} = 0$$

会发现笔记里第一项用的是 $2\mathbf{x}^T \mathbf{A}^T \mathbf{Q}_f \mathbf{B}$，这是行向量（分子布局求导法），而第二项 $2(\mathbf{R} + \dots)\mathbf{u}$ 又是列向量（分母布局）。行向量加列向量在严格的矩阵运算里是不合法的（产生了维度不匹配）。

总结： 笔记里的这一行其实有微小的排版/推导符号疵漏（混用不同矩阵布局规则）。 你可以把第一项修正为 $2\mathbf{B}^T \mathbf{Q}_f \mathbf{A}\mathbf{x}$，一切就完全说得通了。把修正后的第一项移到等号右边： $$2(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B}) \mathbf{u} = -2\mathbf{B}^T \mathbf{Q}_f \mathbf{A}\mathbf{x}$$ 两边除以2，并左乘逆矩阵，就能完美解出下面那行笔记里的 $\mathbf{u}^$ 了： $$\mathbf{u}^ = -(\mathbf{R} + \mathbf{B}^T \mathbf{Q}_f \mathbf{B})^{-1} \mathbf{B}^T \mathbf{Q}_f \mathbf{A} \mathbf{x}$$