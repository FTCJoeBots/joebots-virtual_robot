package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class AlexServo {

    Servo servo = null;

    public void init(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0);
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }
}
