import numpy as np
import matplotlib.pyplot as plt
# %matplotlib inline 缩放

plt.style.use('ggplot')
plt.rcParams['figure.figsize'] = (12, 8)

# Normal distributed x and y vector with mean 0 and standard deviation 1
x = np.random.normal(0, 1, 200)
y = np.random.normal(0, 1, 200)
X = np.vstack((x, y)) # 2xn

# 缩放
sx, sy = 0.5, 2.0
Scale = np.array([[sx, 0], [0, sy]])

Y = Scale.dot(X)
# 原始点集
plt.scatter(X[0, :], X[1, :])
# 缩放后点
plt.scatter(Y[0, :], Y[1, :])
plt.title('Generated Data')
plt.axis('equal')
plt.show()