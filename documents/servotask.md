## 舵机的使用

舵机的具体实现在`RM_Frame_C\userCode\devices\Inc`的`Servo.h`和`RM_Frame_C\userCode\devices\Src`的`Servo.cpp`中。感兴趣同学自行阅读代码，本教程只对这两个源文件中对应的接口使用进行解释。

在本框架中，由于已经对舵机该设备进行了封装，所以在校内赛中，只用对以下提到的代码在`RM_Frame_C\userCode\tasks\Src`中的C++源文件中相应进行调用即可。

一个舵机的使用流程：定义舵机->填写handle函数->用settarget函数赋值使其运动，除此之外还要使舵机在遥控器急停挡位断电。

### 如何定义舵机

可以对以下参数进行调整来实现舵机的定义：

```c
SERVO_INIT_T Servo_Test_Init ={    	//初始化函数名称
   .servoType = ,    	//舵机类型：POSITION_180：180°舵机（舵机类型自行搜索）
    				 	//POSITION_360：360°舵机（舵机类型自行搜索）
   .servoID =   ,		 //舵机ID：由SERVO_ID_1~SERVO_ID_7组成，与硬件接线有关，不可重复
   .firstAngle = ,       //开机角度：舵机初次上电时转到的角度
   .angleLimit_Min = ,   //最小角度：舵机可以转到的最小角度
   .angleLimit_Max =     //最大角度：舵机可以转到的最大角度
};
Servo TestServo(&Servo_Test_Init);   //声明舵机，调用先前的初始化函数
```

以下代码为定义名为`ClawServo`和`TurnLServo`的舵机的示例代码：

```c++
/*定义舵机ClawServo*/
SERVO_INIT_T Claw_Servo_Init ={
   .servoType = POSITION_180,    //舵机类型：180°舵机（舵机类型自行搜索）
   .servoID = SERVO_ID_1,		 //舵机ID：由SERVO_ID_1，接线如下图所示
   .firstAngle = 0,              //开机角度：舵机初次上电时转到的角度
   .angleLimit_Min = 0,          //最小角度：舵机可以转到的最小角度
   .angleLimit_Max = 180         //最大角度：舵机可以转到的最大角度
};
Servo ClawServo(&Claw_Servo_Init);   //声明舵机
/*定义舵机TurnLServo*/
SERVO_INIT_T TurnL_Servo_Init ={
        .servoType = POSITION_180,
        .servoID = SERVO_ID_2,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo TurnLServo(&TurnL_Servo_Init);
```

![servo_C](graph\servo_C.jpg)

#### 硬件部分

这里介绍舵机有关硬件接法，由于其和初始化代码中的`.servoID`有关，所以这里在单独介绍一下，注意接线时舵机和舵机的ID相对应，并且按照线序正确链接（线序可以参考上图，按颜色连接）。

![servo_C](graph\Cpwm.png)



### 如何驱动舵机

将如下代码放入该舵机所定义`cpp`文件中的`Handle()`函数中。

```cpp
void ServoTestHandle(){
//在这里放入TestServo.Handle()即可使舵机正常运行
}
```

依然用之前的`ClawServo`舵机举例，将`ClawServo.Handle()`函数放入本框架中预先提供的整体Handle函数即可使用，如果使用了其他舵机，均可以放在这个地方来驱动舵机。

```cpp
void ClawHandle(){
    ClawServo.Handle();
    TurnLServo.Handle();

}
```

### 如何使舵机转到相应位置

可以对以下参数进行调整来实现舵机的转动：

```cpp
TestServo.SetTargetAngle(float _targetAngle); //可以将转到_angle位置
```

以下为之前定义示例舵机`ClawServo`的驱动程序

```c++
ClawServo.SetTargetAngle(10); 				//将舵机转到10°
ClawServo.SetTargetAngle(170);				//将舵机转到170°
```

注意该驱动程序可以放在遥控器的功能模块里，有关遥控器的功能模块会讲到在本文档的其他位置讲到。

### 如何使舵机断电

该部分为通过检录而准备在遥控器急停挡位使的舵机断电的程序。

将如下代码放入该舵机所定义`cpp`文件中的`stop()`函数中即可实现该功能。

```cpp
void ServoTestStop(){
//在这里放入TestServo.stop()即可使舵机在遥控器急停挡位断电
}
```

依然用之前的`ClawServo`舵机举例，将`ClawServo.stop()`函数放入本框架中预先提供的整体`Stop`函数即可使用，如果使用了其他舵机，均可以放在这个地方来使其他舵机断电。

```cpp
void ClawStop(){
    ClawServo.stop();
    TurnLServo.stop();
}
```

