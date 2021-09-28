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

@TeleOp(name = "Greg Programming Demo", group = "Test")
public class GregProgrammingBoardTest extends OpMode {

    DcMotor motor = null;
    Servo servo = null;
    BNO055IMU imu = null;
    ColorSensor colorSensor = null;
    AnalogInput analogInput = null;
    DigitalChannel digitalChannel = null;
    DistanceSensor distanceSensor = null;

    int state=0;
    boolean previousPressed=false;

    public void init(){
        motor = hardwareMap.get(DcMotor.class, "motor");

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

        MotorConfigurationType motorConfigType = motor.getMotorType();
        telemetry.addData("Motor", " MaxRPM = %.2f  MaxRPMFraction = %.2f", motorConfigType.getMaxRPM(),
                motorConfigType.getAchieveableMaxRPMFraction());
        telemetry.addData("Motor", " AchieveableMaxTicksPerSec = %.2f", motorConfigType.getAchieveableMaxTicksPerSecond());
        telemetry.addData("Motor ", " Ticks/Rot = %.2f  gearing = %.2f", motorConfigType.getTicksPerRev(),
                motorConfigType.getGearing());
        telemetry.addData("Motor Orientation", motorConfigType.getOrientation());

    }


    public void loop(){
        //motor.setPower(-gamepad1.left_stick_y);
        //servo.setPosition(0.5*(gamepad1.right_stick_y + 1));
        Orientation orientation = imu.getAngularOrientation();

        // Run the motor when the touch sensor is pressed
        if(!digitalChannel.getState() && !previousPressed) {
            //Makes the motor go forward if the state is one
            if(state==1) {
               motor.setPower(1);
            }
            //Makes the motor go backward if the state is two
            else if(state==2) {
                motor.setPower(-1);
            }
            //Stops the motor if the state is three
            else if(state==3) {
                motor.setPower(0.0);
            }
            state+=1;
            if(state>3) {
                state=1;
            }
            previousPressed=true;
        }
        if(digitalChannel.getState()) {
            previousPressed=false;
        }




        telemetry.addData("Heading", "%.1f deg", orientation.firstAngle);
        telemetry.addData("Ticks", motor.getCurrentPosition());
        telemetry.addData("Volts", analogInput.getVoltage());
        telemetry.addData("Touch", !digitalChannel.getState());
        telemetry.addData("Color", "R: %d  G: %d  B: %d", colorSensor.red(),
                colorSensor.green(), colorSensor.blue());
        telemetry.addData("Distance (CM)", distanceSensor.getDistance(DistanceUnit.CM));

    }

}
