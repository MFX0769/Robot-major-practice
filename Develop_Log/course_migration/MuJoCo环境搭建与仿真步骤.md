# MuJoCo 环境搭建与仿真步骤

## 目标
从 SolidWorks 模型 → URDF → MuJoCo 仿真，在 Windows 11 上完成。

---

## Step 1: 安装 SolidWorks → URDF 导出插件

### 方法 A（推荐）：SW2URDF
1. 下载 SW2URDF：https://github.com/dfki-ric/sw2urdf/releases
2. 安装：运行安装程序，会自动注册到 SolidWorks 插件
3. 在 SolidWorks 中启用：
   - 打开 SolidWorks → 工具 → 插件 → 勾选 "SW2URDF"
4. 导出：
   - 打开你的机械臂装配体
   - 工具 → SW2URDF → Export
   - 按提示配置：每个关节指定类型（revolute）、旋转轴、限位角度
   - 导出后会生成一个文件夹，内含：
     ```
     my_arm/
     ├── my_arm.urdf
     ├── meshes/
     │   ├── link1.STL
     │   ├── link2.STL
     │   └── ...
     └── config/
     ```

### 方法 B（备选）：onshape-to-robot
如果 SolidWorks 是同学在用，也可以把模型传 Onshape 再用这个转换。
- https://github.com/rhoban/onshape-to-robot

---

## Step 2: 安装 MuJoCo

打开 PowerShell（管理员），执行：
```powershell
pip install mujoco
```

验证安装：
```powershell
python -m mujoco.simulate
```
如果弹出 MuJoCo 自带的仿真 GUI（里面有个人形机器人），说明装成功了。

---

## Step 3: URDF → MuJoCo MJCF 转换

MuJoCo 自带 URDF 转 MJCF 工具。

写一个 Python 脚本 `convert_urdf.py`：

```python
import mujoco

# 转换 URDF 到 MJCF
mjcf_string = mujoco.urdf_to_mjcf("你的文件.urdf")

# 保存为 XML
with open("my_arm.xml", "w") as f:
    f.write(mjcf_string)

# 验证并查看
m = mujoco.MjModel.from_xml_path("my_arm.xml")
print(f"模型加载成功！关节数: {m.nu}")
```

运行：
```powershell
python convert_urdf.py
```

---

## Step 4: 在 MuJoCo 中仿真你的机械臂

### 方法 1：直接用 simulate GUI 打开
```powershell
python -m mujoco.simulate my_arm.xml
```
会用 GUI 打开你的机械臂，可以：
- **拖拽旋转视角**
- Ctrl+右键拖拽关节（手动操控）
- 按 Space 暂停/继续物理仿真

### 方法 2：写 Python 脚本控制
```python
import mujoco
import mujoco.viewer
import time

# 加载模型
m = mujoco.MjModel.from_xml_path("my_arm.xml")
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():
        t = time.time() - start

        # 目标角度（随时间变化，模拟往复运动）
        angle = 0.5 * (1 + mujoco.mju_sin(t * 0.8))
        d.ctrl[0] = angle * 3.14  # 控制第一个关节

        mujoco.mj_step(m, d)
        viewer.sync()
        time.sleep(0.01)
```

运行：
```powershell
python control_arm.py
```

---

## Step 5: 调优（让手臂更真实）

URDF 转 MJCF 后可能需要微调 XML：
- **物理参数**：修改 `<geom>` 中的摩擦力、质量
- **关节限位**：修改 `<joint range="...">`
- **颜色材质**：修改 `<geom rgba="...">`
- **重力补偿**：在 `<worldbody>` 中调整 `<option gravity="...">`

直接编辑 `my_arm.xml`，保存后重新运行就能看到变化。

---

## 文件结构预期

```
RoboticMajor_Pratice_code/
├── mujoco_sim/
│   ├── convert_urdf.py       ← URDF 转 MJCF 脚本
│   ├── control_arm.py        ← Python 控制脚本
│   ├── my_arm.xml            ← MuJoCo 模型
│   ├── meshes/               ← 贴图 STL 文件
│   │   ├── link1.STL
│   │   └── ...
│   └── README.md
├── matlab_sim/
├── develop_log/
└── ...
```

---

## 常见问题

### Q: 打开后模型瘫在地上？
- 关节的 `axis` 或者 `limit` 设置不对，回头检查 URDF 导出时的关节配置
- 或者重力太大，在 XML 里调 `<option gravity="0 0 -5">`

### Q: 视觉 mesh 不显示？
- STL 文件路径需要在 URDF 里用相对路径
- 或者直接在 MJCF 里用 `<geom type="mesh" file="meshes/link1.STL">`

### Q: SW2URDF 插件装不上？
- 检查 SolidWorks 版本（2019+ 支持较好）
- 或者走替代方案：手动写 URDF（不推荐，除非你清楚关节参数）
