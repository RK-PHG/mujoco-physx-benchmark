import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei']  # Windows系统黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示异常

# 用户可修改参数 ----------------------------------
dt = 0.005  # 时间步长（秒）
file_path1 = "../data/penetrations/0.005/mujoco_666_test_result.csv"  # 第一个文件路径
file_path2 = "../data/penetrations/0.005/physx_666_test_result.csv"  # 第二个文件路径
# ----------------------------------------------

# 读取两个CSV文件
df1 = pd.read_csv(file_path1)
df2 = pd.read_csv(file_path2)

# 验证列结构一致性
if list(df1.columns) != list(df2.columns):
    raise ValueError("CSV文件列结构不一致")

# 生成时间轴（假设两个文件数据长度相同）
time_steps = dt * np.arange(len(df1))

# 创建绘图画布
plt.figure(figsize=(14, 7))

# 绘制双文件数据
for col in df1.columns:
    # 绘制第一个文件数据（实线，橙色）
    plt.plot(time_steps, df1[col],
             color='orange',
             linestyle='-',
             linewidth=2,  # 加粗线条
             label='HW-Sim')

    # 绘制第二个文件数据（实线，蓝色）
    plt.plot(time_steps, df2[col],
             color='b',
             linestyle='-',
             linewidth=2,  # 加粗线条
             label='Physx')

# 设置坐标轴标签
plt.xlabel(f'Time (s) Dt: {dt}', fontsize=12)
plt.ylabel('Penetrations', fontsize=12)

# 将图例放在图表内部右上角
plt.legend(loc='upper right',  # 右上角位置
           frameon=True,       # 添加边框
           fontsize=10,
           edgecolor='gray')   # 边框颜色

# 添加网格线（可选）
plt.grid(True, linestyle='--', alpha=0.5)

# 自动调整布局
plt.tight_layout()
plt.show()