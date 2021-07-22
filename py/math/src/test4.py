import numpy as np
import matplotlib.pyplot as plt

plt.style.use('ggplot')
plt.rcParams['figure.figsize'] = (12, 8)

# 特征分解
# https://robot.czxy.com/docs/pcl/chapter03/point_cloud_math/
# 特征分解(旋转矩阵和特征向量矩阵相关，缩放矩阵与特征值矩阵相关)
sigma = np.array([
    [19 / 16, -15 * np.sqrt(3) / 16,],
    [-15 * np.sqrt(3) / 16, 49 / 16]
])

eVa, eVe = np.linalg.eig(sigma)
print("eigen value:\n", eVa)
print("eigen vector:\n", eVe)

# plt.scatter(Y[:, 0], Y[:, 1])
for value, eigen in zip(eVa, eVe.T):
    plt.plot(
        [0, 3 * np.sqrt(value) * eigen[0]],
        [0, 3 * np.sqrt(value) * eigen[1]],
        lw=5)

plt.title('Transformed Data')
plt.axis('equal')
plt.show()

"""
一个多元正态分布的概率密度函数中，其协方差矩阵 Σ 包含了分布的旋转角度和缩放信息，其均值向量 
→
μ
 包含了密度中心的位置信息。
"""