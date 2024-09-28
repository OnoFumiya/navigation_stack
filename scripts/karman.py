import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
import matplotlib.pyplot as plt

# 3ステップの位置 (非線形な軌道を仮定)
positions = np.array([[0, 0], [1, 2], [3, 5]])

# EKFの初期設定
ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)

# 状態ベクトル [x, y, x速度, y速度]
ekf.x = np.array([positions[2, 0], positions[2, 1], 0, 0])

# 観測誤差共分散行列
ekf.R = np.array([[1, 0], 
                  [0, 1]])

# 予測誤差共分散行列
ekf.P *= 10

# プロセスノイズ共分散行列
ekf.Q = np.array([[1, 0, 0, 0], 
                  [0, 1, 0, 0], 
                  [0, 0, 1, 0], 
                  [0, 0, 0, 1]])

# 状態遷移モデル（非線形）
def fx(x, dt):
    """非線形の状態遷移関数。ここでは放物線運動を仮定。"""
    F = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return np.dot(F, x)

# 観測モデル
def hx(x):
    """観測関数：位置のみを返す。"""
    return np.array([x[0], x[1]])

# ヤコビ行列の計算（線形化）
def jacobian_F(x, dt):
    """状態遷移関数のヤコビ行列。"""
    return np.array([[1, 0, dt, 0],
                     [0, 1, 0, dt],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def jacobian_H(x):
    """観測関数のヤコビ行列。"""
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0]])

# 過去の2つの位置データを用いて速度を計算
delta_t = 0.2  # 時間の間隔
velocity_x = (positions[2, 0] - positions[1, 0]) / delta_t
velocity_y = (positions[2, 1] - positions[1, 1]) / delta_t

# 状態ベクトルを更新（位置と速度）
ekf.x = np.array([positions[2, 0], positions[2, 1], velocity_x, velocity_y])

# 軌道を予測
num_steps = 200  # 予測するステップ数
predicted_positions = []

for _ in range(num_steps):
    ekf.F = jacobian_F(ekf.x, delta_t)  # ヤコビ行列を計算
    ekf.predict()  # 予測ステップ
    ekf.update(np.array([positions[2, 0], positions[2, 1]]), HJacobian=jacobian_H, Hx=hx)  # 観測を更新
    predicted_positions.append(ekf.x[:2])  # 位置のみを取得

# 結果をプロット
predicted_positions = np.array(predicted_positions)

# 実際の位置
actual_positions = positions

# プロット設定
plt.plot(actual_positions[:, 0], actual_positions[:, 1], 'bo-', label='実際の位置')
plt.plot(predicted_positions[:, 0], predicted_positions[:, 1], 'ro--', label='予測された軌道')

plt.xlabel('X座標')
plt.ylabel('Y座標')
plt.title('非線形軌道予測（EKF）')
plt.legend()
plt.grid()
plt.show()
