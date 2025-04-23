import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 用户可修改参数 ----------------------------------
dt = 0.001  # 时间步长（秒）
file_path1 = "../data/momentum/0.01/mujoco_rolling_test_result.csv"
file_path2 = "../data/momentum/0.01/physx_rolling_test_result.csv"
window_size = 10  # 重采样窗口大小


# ----------------------------------------------

# 读取并预处理数据
def resample_data(df):
    """每N行计算平均值"""
    group_idx = np.arange(len(df)) // window_size  # 动态分组
    return df.groupby(group_idx).mean()


df1 = resample_data(pd.read_csv(file_path1))
df2 = resample_data(pd.read_csv(file_path2))

# 验证列结构一致性
if list(df1.columns) != list(df2.columns):
    raise ValueError("CSV文件列结构不一致")

# 获取列名
columns = df1.columns
n_cols = len(columns)

# 生成新时间轴 (单位：秒)
resampled_dt = window_size * dt  # 动态时间间隔
time_steps = resampled_dt * np.arange(len(df1))

# 创建绘图画布（垂直排列的子图）
fig, axes = plt.subplots(1, n_cols, figsize=(5*n_cols, 7))

# 如果只有一列，将axes转为数组形式
if n_cols == 1:
    axes = [axes]

# 为每个列创建独立的子图
for idx, col in enumerate(columns):
    ax = axes[idx]

    # 绘制第一个文件数据（实线）
    ax.plot(time_steps, df1[col],
            color='orange',
            linestyle='-',
            label='mujoco')

    # 绘制第二个文件数据（虚线）
    ax.plot(time_steps, df2[col],
            color='b',
            linestyle='--',
            label='physx')

    # 设置子图标题和标签
    ax.set_ylabel(f'Momentum ({window_size}-step Avg)', fontsize=10)
    ax.legend(loc='upper right', frameon=True, fontsize=9)
    ax.grid(True)

    # 添加子图标题（可选）
    ax.set_title(f'Column: {col}', fontsize=11, pad=10)

# 设置共用x轴标签
axes[-1].set_xlabel('Time (s)', fontsize=12)

# 自动调整布局
plt.tight_layout()
plt.show()