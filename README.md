# ReadMe

This repository provides a testing platform for comparing MuJoCo and PhysX. It includes a unified benchmarking tool and a GUI interface for easy analysis

##### 目录解释

```
mujoco_and_physx_benchmark
├── sim 用于定义和实现仿真的通用接口
│   ├── common 	定义了仿真测试的通用接口，包括搭建场景，添加物体，获取物体参数等
│   ├── physxSim 	接口的physx实现
│   └── mujocoSim	接口的mujoco实现
├── benchmark	用于实现具体的场景
│   ├── include	头文件、用于设置具体场景的参数
│   ├── mujocoSim	mujoco场景的实现
│   ├── physxSim 	physx场景实现
│   └── res			存放测试运行脚本
├── lib		外部依赖库
│   ├── mujoco	mujoco库
│   ├── physx	physx库
│   ├── raicommon	GUI库
│   ├── raigraphics	GUI库
│   └── yamlcpp		yaml读取库
├── plot	测试折线图绘制
│   ├── distance_time.py		绘制滑块s-t图
│   ├── velocity_time_plot.py	绘制滑块v-t图	
│   ├── energy.py				绘制能量守恒折线图
│   ├── pentrations.py			绘制穿透误差折线图
│   └── momentum.py				绘制动量折线图
├── res		存放模型(urdf等格式)资源
├── data	存放实验测试结果数据	
└── run_build.sh	安装依赖，初始化环境
```

