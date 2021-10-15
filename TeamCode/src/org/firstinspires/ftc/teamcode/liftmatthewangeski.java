package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class liftmatthewangeski {
    DcMotor motor = null;
    final double LIFTPOSITIONSPEED=0.5;
    final double MANUALLIFTSPEED=0.5;
    public void init(HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class,"motor");
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void Stop_Lift_Motor(){
        motor.setPower(0);
    }
    public void Lift_Up_Manual(){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(motor.getCurrentPosition()+30);
        motor.setPower(MANUALLIFTSPEED);
    }
    public void Lift_Down_Manual(){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(motor.getCurrentPosition()-30);
        motor.setPower(MANUALLIFTSPEED);
    }
    public void Lift_To_Position(int LiftPosition) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(LiftPosition);
            motor.setPower(LIFTPOSITIONSPEED);
    }
    public void motor_stop() { motor.setPower(0); }

}
