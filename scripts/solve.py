# import numpy as np
# from scipy.optimize import fsolve


# # 方程式を定義
# def equation(x):
#     # 定数を設定
#     x12 = -3.5
#     y12 = -2.7
#     x32 = 1.6
#     y32 = 2.6
#     term1 = y12 / (1 / (1 + np.exp(-x * x12)) - 0.5)
#     term2 = y32 / (1 / (1 + np.exp(-x * x32)) - 0.5)
#     return term1 - term2

# # 初期推定値を設定
# x_initial_guess = 0.0

# # 解を求める
# x_solution = fsolve(equation, x_initial_guess)

# print(f"解: {x_solution[0]}")



import numpy as np
from scipy.optimize import fsolve

# 方程式を定義
def equation(x):
    y12 = -2.7
    y32 = 2.6
    x12 = -3.5
    x32 = 1.4
    return (y12 / (1 / (1 + np.exp(-x * x12)) - 0.5)) - (y32 / (1 / (1 + np.exp(-x * x32)) - 0.5))

# 初期推定値を設定
x_initial_guess = 0.1
x_solution = fsolve(equation, x_initial_guess)
print(f"解: {x_solution[0]}")