import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# シグモイド関数の定義
def sigmoid(x, L, x0, k):
    return L / (1 + np.exp(-k * (x - x0)))

# 3点の座標（x, y）のデータ
# x_data = np.array([x1, x2, x3])  # 例: [1, 2, 3]
# y_data = np.array([y1, y2, y3])  # 例: [0.1, 0.5, 0.9]
x_data = np.array([1, 2, 3])  # 例: [1, 2, 3]
y_data = np.array([4, 5, 6])  # 例: [0.1, 0.5, 0.9]

# 初期推定値 (L, x0, k)
initial_guess = [1.0, np.mean(x_data), 1.0]

# 曲線フィッティング
popt, pcov = curve_fit(sigmoid, x_data, y_data, p0=initial_guess)

# フィッティングされたパラメータ
L, x0, k = popt

# フィッティングされたシグモイド曲線のプロット
x_fit = np.linspace(min(x_data), max(x_data), 100)
y_fit = sigmoid(x_fit, *popt)

plt.scatter(x_data, y_data, label="Data", color="red")
plt.plot(x_fit, y_fit, label="Fitted Sigmoid", color="blue")
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Sigmoid Curve Fitting')
plt.show()

print(f"Fitted parameters: L = {L}, x0 = {x0}, k = {k}")
