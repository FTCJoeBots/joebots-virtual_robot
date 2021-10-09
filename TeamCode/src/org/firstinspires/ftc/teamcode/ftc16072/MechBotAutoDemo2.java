package org.firstinspires.ftc.teamcode.ftc16072;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import virtual_robot.util.AngleUtils;

/**
 * Example Autonomous Opmode
 *
 * Uses Line-following two drive around the tape at the perimeter of the lander.
 *
 * Requires mechanum bot configuration.
 *
 * Start with bot in center of lander, facing top of screen.
 *
 * Disabling for now; it was designed to work with Rover Ruckus field
 *
 */
//@Disabled
@Autonomous(name = "mechbot auto demo9", group = "Mechanum")
public class MechBotAutoDemo2 extends LinearOpMode {
    DcMotor m1, m2, m3, m4;
    //GyroSensor gyro;
    BNO055IMU imu;
    ColorSensor colorSensor;
    Servo backServo;

    public void runOpMode(){

        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





boolean SawWhite=false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        backServo = hardwareMap.servo.get("back_servo");

        Orientation orientation;
        int colors[]={colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha()};
        ElapsedTime waitTime = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Seconds since init", "%d. Press start when ready.", (int) waitTime.seconds());
            telemetry.update();
        }
        while (opModeIsActive() && !isStopRequested()) {
            colors[0]= colorSensor.red();
            colors[1]= colorSensor.green();
            colors[2]= colorSensor.blue();
            colors[3]= colorSensor.alpha();
            doThing(colors);
        }

    }
    public void doThing(int[] PassColors) {
        m1.setPower(1);
        m2.setPower(1);
        m3.setPower(1);
        m4.setPower(1);
        if(PassColors[3]>240) {
        }
    }
}
