完成以下测试opmode：
TurretTickTest: 用一操手柄左摇杆的左右方向控制yaw电机功率，用一操左右摇杆的上下方向分别控制仰角舵机旋转，用二操的左右摇杆控制扳机舵机旋转，在telemetry实时显示yaw电机的累计tick数和所有4个舵机的位置，用于标定超参。
TurretYawTest: 用一操手柄方向键控制yaw电机旋转一定角度，用于标定pid
FlywheelTest: 用一操手柄方向键上下和左右分别控制大球和小球飞轮目标速度，用于标定pid和超参。
TurretManualTest: 用二操手柄控制炮台：
左摇杆左右：yaw平转（目标角 = −摇杆 × yawRange，直接位置控制，不经过 updateManual 的速率积分）
大球 / 小球仰角：由 Dashboard 的 targetPitchBig / targetPitchSmall 直接设置，手柄不控制仰角
dpad上下：大球Shooter飞轮目标速度，范围0-最大速
dpad左右：小球Shooter飞轮目标速度，范围0-最大速
左bumper：小球扳机
右bumper：大球扳机
右trigger：intake 电机直通


