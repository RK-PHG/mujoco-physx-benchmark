import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei']  # Windows系统黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示异常

# --------------------- 文件读取配置 ---------------------
file_path1 = '../data/distance/0.005/physx_sliders_test_result.csv'
file_path2 = '../data/distance/0.005/mujoco_sliders_test_result.csv'
time_step = 0.005

# 读取两个数据文件
df1 = pd.read_csv(file_path1)
df2 = pd.read_csv(file_path2)

# 验证数据结构一致性
if list(df1.columns) != list(df2.columns):
    raise ValueError("文件列结构不一致")

# 生成时间轴（假设两个文件长度相同）
time_actual = time_step * np.arange(len(df1))

# --------------------- 理论模型参数 ---------------------
a1 = 2.20725  # v1减速度
a2 = 1.962  # v2加速度
a3 = 1.962  # 共速后减速度
v1_initial = 10.0
v2_initial = 0.0


# --------------------- 理论位移计算函数 ---------------------
def calculate_theory_displacement(total_time):
    time = np.arange(0, total_time + time_step, time_step)
    t1 = max((v1_initial - v2_initial) / (a2 + a1), 0)
    v_common = v1_initial - a1 * t1
    t2 = t1 + (v_common / a3 if a3 != 0 else 0)

    # 分阶段计算位移
    s1_th = np.zeros_like(time)
    s2_th = np.zeros_like(time)

    # 阶段一：独立运动
    phase1 = time <= t1
    s1_th[phase1] = v1_initial * time[phase1] - 0.5 * a1 * time[phase1] ** 2
    s2_th[phase1] = 0.5 * a2 * time[phase1] ** 2

    # 阶段二：联合减速
    phase2 = (time > t1) & (time <= t2)
    dt_phase1 = t1
    s1_phase1 = v1_initial * dt_phase1 - 0.5 * a1 * dt_phase1 ** 2
    s2_phase1 = 0.5 * a2 * dt_phase1 ** 2

    dt_phase2 = time[phase2] - t1
    s_common = v_common * dt_phase2 - 0.5 * a3 * dt_phase2 ** 2
    s1_th[phase2] = s1_phase1 + s_common
    s2_th[phase2] = s2_phase1 + s_common

    # 阶段三：已停止
    phase3 = time > t2
    dt_phase2 = t2 - t1
    s_common_end = v_common * dt_phase2 - 0.5 * a3 * dt_phase2 ** 2
    s1_th[phase3] = s1_phase1 + s_common_end
    s2_th[phase3] = s2_phase1 + s_common_end

    return time, s1_th, s2_th


# 计算理论曲线
time_theory, s1_th, s2_th = calculate_theory_displacement(time_actual[-1])

# --------------------- 可视化设置 ---------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6), sharex=True)

# 颜色配置
data_colors = {
    'physx': '#1f77b4',  # 蓝色
    'mujoco': '#ff7f0e'  # 橙色
}

theory_style = {
    'linestyle': '--',
    'alpha': 0.7,
    'linewidth': 1.5
}

# 计算最大纵轴范围
y_max = max(df1[['s1', 's2']].max().max(),
            df2[['s1', 's2']].max().max(),
            s1_th.max(),
            s2_th.max()) * 1.1

# 左图：s1对比
ax1.plot(time_actual, df1['s1'], color=data_colors['physx'], label='PhysX s1')
ax1.plot(time_actual, df2['s1'], color=data_colors['mujoco'], label='HW-Sim s1')
ax1.plot(time_theory, s1_th, color='r',  ** theory_style, label = '理论曲线 s1')

# 右图：s2对比
ax2.plot(time_actual, df1['s2'], color=data_colors['physx'], label='PhysX s2')
ax2.plot(time_actual, df2['s2'], color=data_colors['mujoco'], label='HW-Sim s2')
ax2.plot(time_theory, s2_th, color='r',  ** theory_style, label = '理论曲线 s2')

# 统一坐标轴设置
for ax in [ax1, ax2]:
    ax.set_xlabel('Time (s) Dt: 0.005', fontsize=12)
    ax.set_xlim(0, np.ceil(time_actual[-1]))
    ax.set_ylim(0, y_max)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper left')

ax1.set_ylabel('s1 (m)', fontsize=12)
ax2.set_ylabel('s2 (m)', fontsize=12)

plt.tight_layout()
plt.savefig('dual_s_plot.png', dpi=300, bbox_inches='tight')
plt.show()