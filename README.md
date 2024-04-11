import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

class Robot:
    def __init__(self, hip_length, knee_length):
        self.hip_length = hip_length
        self.knee_length = knee_length

    def bezier_curve(self, t, control_points):
        n = len(control_points) - 1
        curve = np.zeros((len(t), 2))
        for i in range(n + 1):
            curve += self.bernstein_poly(n, i, t)[:, np.newaxis] * control_points[i]
        return curve

    def bernstein_poly(self, n, i, t):
        return np.math.factorial(n) / (np.math.factorial(i) * np.math.factorial(n - i)) * (t ** i) * ((1 - t) ** (n - i))

    def hip_trajectory(self, control_points):
        def objective(t):
            point = self.bezier_curve(t, control_points)[-1]
            return point[1]  # 髋关节高度作为目标函数

        result = minimize(objective, x0=0.5, bounds=[(0, 1)])  # 初始猜测为0.5，t的范围为[0, 1]
        t_optimal = result.x[0]
        hip_height = self.bezier_curve(t_optimal, control_points)[-1][1]
        return hip_height, t_optimal

# 定义路径点
control_points = np.array([
    [0, 0],
    [0.5, 1],
    [1, 0.5],
    [1.5, 0]
])


hip_height, t_optimal = robot.hip_trajectory(control_points)
print("Optimal t:", t_optimal)
print("Hip Height:", hip_height)

# 绘制髋关节高度-时间曲线
t_values = np.linspace(0, 1, 100)
hip_heights = [robot.bezier_curve(t, control_points)[-1][1] for t in t_values]

plt.plot(t_values, hip_heights)
plt.xlabel('Time')
plt.ylabel('Hip Height')
plt.title('Hip Joint Height - Time Curve')
plt.grid(True)
plt.show()
