package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MatthewSIntake {
    DcMotor MattIntake=null;
    public void init(HardwareMap hardwareMap){
        MattIntake = hardwareMap.get(DcMotor.class, "intakeMotor");
        MattIntake.setPower(0);
    }
    public void setIntakePower(double speed){

        MattIntake.setPower(speed);
    }

    public void stopIntakeMotor() {

        MattIntake.setPower(0);
    }
    public void startIntakeMotor(double speed) {

        MattIntake.setPower(speed);
    }
    public void setIntakeMode(DcMotor.RunMode bees) {

        MattIntake.setMode(bees);

    }
}
