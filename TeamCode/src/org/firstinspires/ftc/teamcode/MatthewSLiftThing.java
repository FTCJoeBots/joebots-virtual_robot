package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MatthewSLiftThing {
    DcMotor MatthewMotor=null;

    public void init(HardwareMap hardwareMap,String Name){
        MatthewMotor = hardwareMap.get(DcMotor.class, Name);
        MatthewMotor.setPower(0);
    }

    public void setLiftPower(double speed){

        MatthewMotor.setPower(speed);
    }

    public void stopLiftMotor() {
        MatthewMotor.setPower(0);
    }

    public void startLiftMotor(double speed) {
        MatthewMotor.setPower(speed);
    }
    public void LiftUpManual(double LiftSpeed) {
        MatthewMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MatthewMotor.setTargetPosition(MatthewMotor.getCurrentPosition()+30);
        MatthewMotor.setPower(LiftSpeed);
    }
    public void LiftDownManual(double LiftSpeed) {
        MatthewMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MatthewMotor.setTargetPosition(MatthewMotor.getCurrentPosition()-30);
        MatthewMotor.setPower(LiftSpeed);
    }
    public int getLiftPosition() {
        return MatthewMotor.getCurrentPosition();
    }
    public void LiftToPosition(int PositionThing,double speedThing) {
        MatthewMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MatthewMotor.setTargetPosition(PositionThing);
        MatthewMotor.setPower(speedThing);
        //Poggers!
        //.././././././...././././././.?>?>?*>?>*?>*??*?*?/8#*?#8//#*?*./8./#>*/.83?>*?8*./>.#?*>./#<>8./.?>*?#>?>#?>754../2.346321
    }

    public void setLiftMode(DcMotor.RunMode bees) {
        MatthewMotor.setMode(bees);
    }
}
