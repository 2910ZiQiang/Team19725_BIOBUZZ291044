# 对射光电传感器（Break Beam）使用指南

本文档介绍如何把一颗**对射光电传感器（断裂光束 / Break Beam）**接入 FTC 主控的数字口，
并用 `BreakBeamSensor` 类把它的数字信号滤波后封装成一个**实时布尔开关量**（是否有遮挡）。

---

## 1. 概述

- **目标**：把"光束是否被遮挡"转换为数字开关信号（`true` / `false`），并能通过软件滤波抑制抖动。
- **硬件原理**：对射光电由**发射端**与**接收端**两部分组成，中间形成一束光。物体经过时挡住光束，接收端输出电平翻转。
- **软件封装**：`BreakBeamSensor` 读取数字口电平 → 映射为 0/1 样本 → EMA 滤波 → 与阈值比较 → 实时输出遮挡状态。

---

## 2. 硬件接线

### 2.1 所需元件

- 对射光电传感器（发射端 + 接收端） ×1
- 杜邦线若干
- FTC 主控（REV Control Hub / Expansion Hub） ×1
- 3.3V / 5V 电源（按传感器规格选择）

### 2.2 连接步骤

1. **供电**：把传感器的 VCC 接到主控可用电源（优先 3.3V；若传感器要求 5V 请按规格接线），GND 接主控 GND。
2. **信号**：把传感器的数字输出（DO / Signal）接到主控的任一 **Digital** 口。
3. **在 Robot Configuration 中**把该 Digital 口命名为程序里使用的设备名（例：`breakBeam`）。
4. `BreakBeamSensor` 构造时会自动把该口设为 **INPUT** 模式，无需手动配置。

> ⚠️ 注意：不同型号/接法的传感器，"遮挡"对应的电平可能是**高**也可能是**低**，
> 因此需要正确设置极性参数 `blockedOnState`（见 §3.3）。

---

## 3. 软件使用 —— `BreakBeamSensor` 类

### 3.1 类结构概览

| 方法 / 字段 | 说明 |
|------------|------|
| `BreakBeamSensor(HardwareMap, String)` | 构造，使用默认 alpha / 阈值 / 极性 |
| `BreakBeamSensor(HardwareMap, String, double alpha, double threshold, boolean blockedOnState)` | 构造，全参数 |
| `BreakBeamSensor(DigitalChannel, double, double, boolean)` | 构造，直接注入已获取的数字口（便于测试） |
| `update()` | 采样、滤波并更新遮挡状态，**每帧必须调用** |
| `isBlocked()` | 最新遮挡判定（`true` = 遮挡） |
| `getRawState()` | 原始数字电平（未滤波、未做极性换算） |
| `getRawValue()` | 归一化原始量（0 / 1） |
| `getFilteredValue()` | 归一化滤波量 [0,1]（EMA 输出，"近段时间被遮挡的程度"） |
| `getThreshold()` / `setThreshold(double)` | 归一化遮挡阈值 [0,1] |
| `getAlpha()` / `setAlpha(double)` | EMA 平滑系数 (0,1] |
| `isBlockedOnState()` / `setBlockedOnState(boolean)` | 遮挡对应的原始电平（极性） |
| `reset()` | 重置滤波器状态（不读硬件，判定结果保持到下一帧 `update()`） |

### 3.2 在 OpMode 中使用

```java
import org.firstinspires.ftc.teamcode.Processors.Sensors.BreakBeamSensor;

public class MyOpMode extends LinearOpMode {
    private BreakBeamSensor beam;

    @Override
    public void runOpMode() {
        // 设备名需与 Robot Configuration 中的 Digital 口名称一致
        beam = new BreakBeamSensor(hardwareMap, "breakBeam");
        waitForStart();

        while (opModeIsActive()) {
            beam.update();                  // 必须每帧调用

            if (beam.isBlocked()) {
                // 有球经过 / 触发相应动作
            }

            telemetry.addData("Blocked", beam.isBlocked() ? 1 : 0);
            telemetry.addData("Filtered", beam.getFilteredValue());
            telemetry.update();
        }
    }
}
```

### 3.3 极性参数 `blockedOnState`

| 接法 | 现象 | 应设置 |
|------|------|--------|
| 遮挡 → 电平 1 | 挡光时 `getRawState()` 为 `true` | `blockedOnState = true`（默认） |
| 遮挡 → 电平 0 | 挡光时 `getRawState()` 为 `false` | `blockedOnState = false` |

**现场确认方法**：先看 `getRawState()` 在"无遮挡 / 有遮挡"两种情况下分别是 0 还是 1，再据此设置。
若设置反了，遮挡判定会整体反相。

---

## 4. 滤波与调试

### 4.1 滤波原理

1. 原始数字量按极性映射为样本：**0= 遮挡、1 = 未遮挡**。
2. 用 `utility.filter.EMA` 做指数移动平均：`filtered = alpha * sample + (1 - alpha) * filtered`。
3. `filtered > threshold` 判定为遮挡。

滤波值可理解为"近段时间被遮挡的程度"。由于是数字量，该过程天然起到**去抖**作用：
- `threshold` 取 0.5 时，约等于"半数以上近期样本为遮挡"才判定；
- `alpha` 越接近 1 响应越快、抗抖越弱；越接近 0 越平滑、延迟越大。

### 4.2 参数调优建议

- **阈值（threshold）**：一般固定 **0.5** 即可（0/1 二值量下最直观）。
- **EMA 系数（alpha）**：先取默认 **0.8**；若捕捉到明显抖动（如球滚动时的瞬时误触发），
  可降低到 **0.3 ~ 0.6** 增加抗抖，代价是响应略有延迟；若要求快速响应则提高到 **0.9 ~ 0.95**。

### 4.3 调试信息输出

建议在 telemetry 中至少显示：

- 原始电平 `getRawState()`
- 滤波值 `getFilteredValue()`
- 当前阈值 `getThreshold()`
- 遮挡判定 `isBlocked()`

配合 FTC Dashboard 可实时改参观察响应。

---

## 5. 测试程序

`OpModes/BreakBeamTestOp.java` 提供了现成的测试与调参用 OpMode，功能与 `PressureTestOp` 一致：

每帧调用 `update()`，实时显示 Raw State / Raw Value / Filtered Value / Threshold / Blocked / Alpha / Polarity。

在 **FTC Dashboard → Config → BreakBeamTestOp** 中可调：

| 参数 | 含义 |
|------|------|
| `threshold` | 归一化遮挡阈值 [0,1] |
| `alpha` | EMA 平滑系数 (0,1]，越接近 1 响应越快、抗噪越差 |
| `blockedOnState` | 遮挡对应的原始电平（适配不同接法） |
| `resetFilter` | 置 true 重置一次滤波器状态（仅上升沿生效） |

---

## 6. 注意事项

- **电平匹配**：确认传感器输出电平与主控数字口兼容（3.3V 逻辑）；若传感器输出 5V，直接接入可能损伤主控，需分压或电平转换。
- **环境光干扰**：部分对射光电对强环境光（如阳光）敏感，可能造成误触发；必要时加遮光套管。
- **安装对准**：发射端与接收端需精确对正，机械振动易导致失准，接线与固定需牢固。
- **共用 GND**：传感器与主控必须共地（GND），否则数字口读数不稳定。
- **采样节拍**：`update()` 依赖主循环节拍（约每帧一次），若需要更高的时间分辨率，可考虑改用
  `DigitalChannel` 的硬件中断 / 更高频率采样，但一般情况下主循环节拍足够。

---

## 7. 总结

`BreakBeamSensor` 用极简的接口把对射光电的数字信号变成一个可直接使用的布尔量：
**构造 → 每帧 `update()` → 读 `isBlocked()`**。
先确认极性（`blockedOnState`），再按实际抖动情况调整 `alpha`，即可在比赛中稳定可靠地检测球的通过。
