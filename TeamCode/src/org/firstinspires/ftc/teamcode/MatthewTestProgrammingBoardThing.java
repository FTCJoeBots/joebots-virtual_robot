package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Matthew Lift Thing", group = "Test")
public class MatthewTestProgrammingBoardThing extends OpMode {
    MatthewSLiftThing MatthewLift1=null;
    MatthewSLiftThing MatthewLift2=null;
    MatthewSLiftThing MatthewLift3=null;
    MatthewSLiftThing MatthewLift4=null;
    Servo servo = null;
    BNO055IMU imu = null;
    ColorSensor colorSensor = null;
    AnalogInput analogInput = null;
    DigitalChannel digitalChannel = null;
    DistanceSensor distanceSensor = null;
    //Testing For Lift
    final int LIFTPOSITION1=1000;
    final int LIFTPOSITION2=2000;
    final int LIFTPOSITION3=3000;
    final int LIFTPOSITION4=4000;

    boolean CurrentStateDpadUp=false;
    boolean CurrentStateDpadDown=false;
    boolean PreviousStateDpadUp=false;
    boolean PreviousStateDpadDown=false;

    public void init(){
        MatthewLift1.init(hardwareMap,"motor");
        MatthewLift2.init(hardwareMap,"motor");
        MatthewLift3.init(hardwareMap,"motor");
        MatthewLift4.init(hardwareMap,"motor");
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0.5);

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        analogInput = hardwareMap.get(AnalogInput.class, "pot");
        digitalChannel = hardwareMap.get(DigitalChannel.class, "touch_sensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);


    }


    public void loop() {
        if(gamepad1.dpad_right) {
            MatthewLift1.LiftUpManual(0.5);
        }
        if(gamepad1.dpad_left) {
            MatthewLift1.LiftDownManual(0.5);
        }
        CurrentStateDpadUp= gamepad1.dpad_up;
        CurrentStateDpadDown=gamepad1.dpad_down;
    if(CurrentStateDpadUp&&CurrentStateDpadUp!=PreviousStateDpadUp) {

        if(MatthewLift1.getLiftPosition()<LIFTPOSITION1) {

            MatthewLift1.LiftToPosition(LIFTPOSITION1,0.5);

        }
        if(MatthewLift1.getLiftPosition()<LIFTPOSITION2) {

            MatthewLift1.LiftToPosition(LIFTPOSITION2,0.5);

        }
        if(MatthewLift1.getLiftPosition()<LIFTPOSITION3) {

            MatthewLift1.LiftToPosition(LIFTPOSITION3,0.5);

        }
        if(MatthewLift1.getLiftPosition()<LIFTPOSITION4) {

            MatthewLift1.LiftToPosition(LIFTPOSITION4,0.5);

        }
    }






    }
}
