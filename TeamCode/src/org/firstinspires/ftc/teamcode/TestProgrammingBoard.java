package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TestProgrammingBoard", group = "Test")
public class TestProgrammingBoard extends OpMode {
    liftmatthewangeski lm = new liftmatthewangeski();
    Servo servo = null;
    BNO055IMU imu = null;
    ColorSensor colorSensor = null;
    AnalogInput analogInput = null;
    DigitalChannel digitalChannel = null;
    DistanceSensor distanceSensor = null;
    final int LIFTPOSTION0 = 0;
    final int LIFTPOSITION1 = 500;
    final int LIFTPOSITION2 = 1000;
    final int LIFTPOSITION3 = 1500;
    final int LIFTPOSITION4 = 2000;
    boolean CurrentStateDpadup = false;
    boolean CurrentStateDpaddown = false;
    boolean PreviousStateDpadup = false;
    boolean PreviousStateDpaddown = false;

// 10/12/21 by Samaika still need to complete up and down lift


    public void init() {
        lm.init(hardwareMap);
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
        telemetry.addData("Lift position", lm.motor.getCurrentPosition());
        if (gamepad1.dpad_right) {
            lm.Lift_Up_Manual();
        }


        if (gamepad1.dpad_left) {
            lm.Lift_Down_Manual();
        }


        if (CurrentStateDpadup && CurrentStateDpadup != PreviousStateDpadup) {

            if (lm.motor.getCurrentPosition() < LIFTPOSITION1) {
                lm.Lift_To_Position(LIFTPOSITION1);
            }
            if (lm.motor.getCurrentPosition() < LIFTPOSITION2) {
                lm.Lift_To_Position(LIFTPOSITION2);
            }
            if (lm.motor.getCurrentPosition() < LIFTPOSITION3) {
                lm.Lift_To_Position(LIFTPOSITION3);
            }
            if (lm.motor.getCurrentPosition() < LIFTPOSITION4) {
                lm.Lift_To_Position(LIFTPOSITION4);
            }
        }

        servo.setPosition(0.5 * (gamepad1.right_stick_y + 1));
        Orientation orientation = imu.getAngularOrientation();
        telemetry.addData("Heading", "%.1f deg", orientation.firstAngle);
        telemetry.addData("Volts", analogInput.getVoltage());
        telemetry.addData("Touch", !digitalChannel.getState());
        telemetry.addData("Color", "R: %d  G: %d  B: %d", colorSensor.red(),
                colorSensor.green(), colorSensor.blue());
        telemetry.addData("Distance (CM)", distanceSensor.getDistance(DistanceUnit.CM));


        if (CurrentStateDpaddown && CurrentStateDpaddown != PreviousStateDpaddown) {

            if (lm.motor.getCurrentPosition() < LIFTPOSITION1) {
                lm.Lift_To_Position(LIFTPOSITION1);
            }
            if (lm.motor.getCurrentPosition() < LIFTPOSITION2) {
                lm.Lift_To_Position(LIFTPOSITION2);
            }
            if (lm.motor.getCurrentPosition() < LIFTPOSITION3) {
                lm.Lift_To_Position(LIFTPOSITION3);
            }
            if (lm.motor.getCurrentPosition() < LIFTPOSITION4) {
                lm.Lift_To_Position(LIFTPOSITION4);
            }
        }
    }
}
