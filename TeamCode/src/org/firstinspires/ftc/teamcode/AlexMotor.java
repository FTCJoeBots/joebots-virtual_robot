package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class AlexMotor {

    DcMotor motor = null;

    public void init(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setPower(0);
    }

    public void stopMotor(){
        motor.setPower(0);
    }

    public void startMotor(double speed){
        motor.setPower(speed);
    }

    public void setPower(double speed){
        motor.setPower(speed);
    }

    public void moveToPosition(int PositionThing,double speedThing) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(PositionThing);
        motor.setPower(speedThing);
        //Poggers!
        //.././././././...././././././.?>?>?*>?>*?>*??*?*?/8#*?#8//#*?*./8./#>*/.83?>*?8*./>.#?*>./#<>8./.?>*?#>?>#?>754../2.346321
    }
public void setMode(DcMotor.RunMode bees) {
        motor.setMode(bees);
}
    public MotorConfigurationType getMotorType(){
        MotorConfigurationType motorConfType = motor.getMotorType();
        return (motorConfType);
    }
}
