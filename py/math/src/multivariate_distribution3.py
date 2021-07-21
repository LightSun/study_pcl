import numpy as np
import matplotlib.pyplot as plt

plt.style.use('ggplot')
plt.rcParams['figure.figsize'] = (12, 8)

# 缩放后旋转: https://robot.czxy.com/docs/pcl/chapter03/point_cloud_math/

# Normal distributed x and y vector with mean 0 and standard deviation 1
x = np.random.normal(0, 1, 200)
y = np.random.normal(0, 1, 200)
X = np.vstack((x, y)) # 2xn

sx, sy = 0.5, 2.0
Scale = np.array([[sx, 0], [0, sy]])

# Rotation matrix. 逆时针30度
theta = np.pi / 6
c, s = np.cos(theta), np.sin(theta)
Rot = np.array([[c, -s], [s, c]])

# Transformation matrix
T = Rot.dot(Scale)

# Apply transformation matrix to X
Y = T.dot(X)

# 原始点集
plt.scatter(X[0, :], X[1, :])
# 缩放、旋转后
plt.scatter(Y[0, :], Y[1, :])
plt.title('Generated Data')
plt.axis('equal')
plt.show()

# Covariance
def cov(x, y):
    xbar, ybar = x.mean(), y.mean()
    return np.sum((x - xbar)*(y - ybar))/(len(x) - 1)

# Covariance matrix
def cov_mat(X):
    return np.array([[cov(X[0], X[0]), cov(X[0], X[1])],
                     [cov(X[1], X[0]), cov(X[1], X[1])]])
# Calculate covariance matrix
print(cov_mat(Y)) # (or with np.cov(Y))