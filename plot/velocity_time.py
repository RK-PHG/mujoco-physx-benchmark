import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei']  # Windows系统黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示异常

# --------------------- 文件读取配置 ---------------------
file_path1 = '../data/distance/0.005/physx_sliders_test_result.csv'
file_path2 = '../data/distance/0.005/mujoco_sliders_test_result.csv'
time_step = 0.001

# 读取数据文件
df1 = pd.read_csv(file_path1)
df2 = pd.read_csv(file_path2)

# 验证数据结构
if list(df1.columns) != list(df2.columns):
    raise ValueError("文件列结构不一致")

# 生成时间轴
time_actual = time_step * np.arange(len(df1))

# --------------------- 理论模型计算 ---------------------
a1, a2, a3 = 2.20725, 1.962, 1.962
v1_initial, v2_initial = 10.0, 0.0


def calculate_theory_curves(total_time):
    time = np.arange(0, total_time + time_step, time_step)
    t1 = max((v1_initial - v2_initial) / (a2 + a1), 0)
    v_common = v1_initial - a1 * t1
    t2 = t1 + (v_common / a3 if a3 != 0 else 0)

    v1_th = np.where(time <= t1, v1_initial - a1 * time,
                     np.where(time <= t2, v_common - a3 * (time - t1), 0)).clip(min=0)
    v2_th = np.where(time <= t1, v2_initial + a2 * time,
                     np.where(time <= t2, v_common - a3 * (time - t1), 0)).clip(min=0)
    return time, v1_th, v2_th


time_theory, v1_th, v2_th = calculate_theory_curves(time_actual[-1])

# --------------------- 可视化设置 ---------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6), sharex=True)

# 颜色配置
data_colors = {
    'data1': '#1f77b4',  # 蓝色
    'data2': '#ff7f0e'  # 橙色
}

theory_styles = {
    'linewidth': 1.5,
    'linestyle': '--',
    'alpha': 0.7
}

# 左图：v1对比
ax1.plot(time_actual, df1['v1'],
         color=data_colors['data1'],
         label='PhysX v1')
ax1.plot(time_actual, df2['v1'],
         color=data_colors['data2'],
         label='HW-Sim v1')
ax1.plot(time_theory, v1_th,
         color='r',
         ** theory_styles,
        label = '理论曲线-v1')

# 右图：v2对比
ax2.plot(time_actual, df1['v2'],
         color=data_colors['data1'],
         label='PhysX v2')
ax2.plot(time_actual, df2['v2'],
         color=data_colors['data2'],
         label='HW-Sim v2')
ax2.plot(time_theory, v2_th,
         color='r',
         ** theory_styles,
         label = '理论曲线-v2')

# 通用设置
for ax in [ax1, ax2]:
    ax.set_xlabel('Time (s) Dt：0.005', fontsize=12)
    ax.set_xlim(0, np.ceil(time_actual[-1]))
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', frameon=False)

ax1.set_ylabel('v1 (m/s)', fontsize=12)
ax2.set_ylabel('v2 (m/s)', fontsize=12)

plt.tight_layout()
plt.savefig('dual_plot.png', dpi=300, bbox_inches='tight')
plt.show()