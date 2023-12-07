package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Climb {

    DcMotorEx climb1, climb2;

    public Climb(DcMotorEx climb1, DcMotorEx climb2) {
        this.climb1 = climb1;
        this.climb2 = climb2;
    }

    public void climbcheck(Gamepad gamepad2) {
        if ((gamepad2.dpad_right || gamepad2.dpad_left) && gamepad2.dpad_down) {
            climb1.setPower(-0.2);
            climb2.setPower(0.2);
        } else if ((gamepad2.dpad_right || gamepad2.dpad_left) && gamepad2.dpad_up) {
            climb1.setPower(0.2);
            climb2.setPower(-0.2);
        } else if (gamepad2.dpad_down) {
            climb1.setPower(-1);
            climb2.setPower(1);
        } else if (gamepad2.dpad_up) {
            climb1.setPower(1);
            climb2.setPower(-1);
        } else {
            climb1.setPower(0);
            climb2.setPower(0);
        }
    }


}
