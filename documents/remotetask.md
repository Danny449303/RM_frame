## 遥控器的使用和程序编写

### 遥控器的物理档位

首先介绍遥控器上的物理档位，遥控器示意图如下图所示。在本次校内赛中，电控框架给出了如下几种控制方式的编写接口：

1. 三位开关SB
2. 三位开关SC
3. 左右遥杆
4. 自选(可自选其余通道, 配置见下文)

![遥控器正视图](graph\remotePic.png)

### 遥控器配置

#### 1. 遥控器系统设置
长按遥控器开机键开机, 忽略油门警报信息. 按下HOME/MON.进入设置菜单. 按下右侧方向键进入系统设置.
显示设置可根据需求自行设置. 报警电压发射机设置为3.6V. 使用过程中若出现报警请立刻使用附带充电线或底部micro-usb口充电, 否则电池过放会导致报废.
摇杆模式为模式1, 开机警告关.

#### 2. 接收机部署
使用DBUS线连接开发板DBUS接口及接收机通道6
打开开发板供电, 遥控器进入设置菜单.
使用右侧方向键进入通信设置-对码, 长按接收机上的SET按键直到指示灯闪烁.
遥控器按下对码开始, 遥控器显示对码成功, 接收机指示灯常亮.

遥控器返回上级菜单(通信设置), 确认PPM/W.BUS模式为W.BUS, 接收机端口设置中端口6为W.BUS, 接收机指示灯为绿色.

#### 3. 通道设置
框架代码与通道设置对应, 若不进行此步将不能正确获取数据.
进入设置菜单-通道设置, 按下表对应关系进行配置, 其中微调均留空
|通道|控制|
|---|---|
|副翼|J1|
|升降|J3|
|油门|J2|
|方向|J4|
|起落架|SB|
|辅助1|SC|
|辅助2|自选|
|辅助3|自选|

前六个通道请按照如上规范配置.
辅助2及辅助3可由参赛队根据自身需求 选择配置其余两个两档开关(SA/SD)及中部两个旋钮(LD/RD).
程序中这两个通道的信息将赋值给RemoteControl::rcInfo.optional[0]与RemoteControl::rcInfo.optional[1].


### 代码介绍

在介绍完以上几种控制按键后，下面来介绍这几种控制方式的代码，注意，该代码存在于`RM_Frame_C\userCode\tasks\Src`的`Control.cpp`文件中，通过在该程序中对已经写好的其他功能接口（如底盘运动、控制舵机）进行调用即可实现通过遥控器对车辆和其功能的控制：

#### 三位开关SB

位于遥控器左上角内侧开关，该开关对应变量：

```cpp
RemoteControl::rcInfo.sLeft
```

该变量的值包括`UP_POS,MID_POS,DOWN_POS`分别对应了开关的上中下三档。

注意，该变量一般不用更改，仅仅明白这个变量的意义即可

#### 三位开关SC

位于遥控器右上角内侧开关，该开关对应变量：

```cpp
RemoteControl::rcInfo.sRight
```

该变量的值包括`UP_POS,MID_POS,DOWN_POS`分别对应了开关的上中下三档。

注意，该变量已经在框架里面给出：

```cpp
switch (RemoteControl::rcInfo.sLeft) {
    case UP_POS://左侧一档

        break;
    case MID_POS://左侧二档

        break;
    case DOWN_POS:default://左侧三档

        break;
}
```

#### 左右遥杆

左右遥杆分别位于遥控器中间部分的左右两侧。

左摇杆:

```cpp
float RemoteControl::rcInfo.left_col;//上下值，向上返回值大于0、向下小于0
float RemoteControl::rcInfo.left_rol;//左右值，向右返回值大于0、向左小于0
```

右摇杆:

```cpp
float RemoteControl::rcInfo.right_col;//上下值，向上返回值大于0、向下小于0
float RemoteControl::rcInfo.right_rol;//左右值，向右返回值大于0、向左小于0
```

以上值均返回一个-1~1的float类型数。注意到达极限位置返回值可能不为1.

如以下示例：

这里的`ChassisSetVelocity()`为控制底盘速度的接口（具体实现见本教程相关内容），以下函数实现了右摇杆上下拨动控制前后移动（`RemoteControl::rcInfo.right_col`）、右摇杆左右拨动控制左右移动（`RemoteControl::rcInfo.right_rol`）、左摇杆左右拨动控制左右旋转（`RemoteControl::rcInfo.left_rol`）。（注意这里的`4.2`是一个系数，表明移动的速度，需要调一个合适的值）

```cpp
ChassisSetVelocity(RemoteControl::rcInfo.right_col*4.2,                     RemoteControl::rcInfo.right_rol*4.2,RemoteControl::rcInfo.left_rol*60);
```

### 代码示例

将以上代码介绍拼接在一起可以，可以得到如下示例代码

```c++
void CtrlHandle(){
    if (RemoteControl::rcInfo.sRight == DOWN_POS){	//三位开关SC下档，急停模式
        ChassisStop();
        UserStop();
    }else {											//三位开关SC上档或中档
        /**以下内容在三位开关SC上档或中档下运行**/
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS:
                /**以下内容在三位开关SC上档下运行**/
                ChassisSetVelocity(RemoteControl::rcInfo.right_col*4.2, RemoteControl::rcInfo.right_rol*4.2,RemoteControl::rcInfo.left_rol*60);
                
                break;
            case MID_POS:
                /**以下内容在三位开关SC中档下运行**/
                uint8_t clawState;
                if (RemoteControl::rcInfo.sRight == UP_POS){
                    /**以下内容在三位开关SC上档下运行**/
                    clawState = 0;
                }else if(RemoteControl::rcInfo.sRight == MID_POS) {
                     /**以下内容在三位开关SC中档下运行**/
                    clawState = 1;
                }
				break;
			case DOWN_POS:default://左侧二档
                /**以下内容在三位开关SB下档下运行**/
                
                
				break;
        }

    }

}
```

### 安全
代码中包含看门狗, 若未检测到遥控器开机且对码成功将持续重制开发版, 代码不会正常运行, 因此若要测试代码需要保证遥控器与接收机通信正常.
遥控器SC开关拨到最下面三档将进入急停模式, 电机及其他任务将终止运行. 若在调试过程中遇到电机异常转动或机器人冲撞的情况必须及时三档.