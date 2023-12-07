package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LiftAndDiff {

    DcMotorEx Lift1, Lift2;
    CRServo IntakeL, IntakeR;
    Servo Outtake, Plane, Diff1, Diff2;
    public double Diff1Rest;
    public double Diff2Rest;
    public double integralSum;
    public double lasterror;
    public double output;
    public double LastLift1Encoder;
    public double LastLift2Encoder;
    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;
    ElapsedTime timer = new ElapsedTime();
    public boolean IsRightTriggerPressed;
    public boolean IsLeftTriggerPressed;


    public LiftAndDiff(DcMotorEx Lift1, DcMotorEx Lift2, CRServo IntakeL, CRServo IntakeR, Servo Outtake, Servo Plane, Servo Diff1, Servo Diff2) {
        this.Lift1 = Lift1;
        this.Lift2 = Lift2;
        this.IntakeL = IntakeL;
        this.IntakeR = IntakeR;
        this.Outtake = Outtake;
        this.Plane = Plane;
        this.Diff1 = Diff1;
        this.Diff2 = Diff2;
        Diff2.setDirection(Servo.Direction.REVERSE);
        IntakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake.setPosition(1);
        Plane.setPosition(0.5);
        Diff1Rest = -0.12;
        Diff2Rest = 0.09;
        Diff2.setPosition(Diff2Rest);
        Diff1.setPosition(Diff1Rest);
        IntakeL.setDirection(DcMotor.Direction.REVERSE);
        Lift1.setDirection(DcMotorEx.Direction.REVERSE);
        Lift2.setDirection(DcMotorEx.Direction.REVERSE);
    }


    /**
     * Describe this function...
     */
    public void LiftAndDiffCheck(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.1) {
            IsRightTriggerPressed = true;
        } else {
            IsRightTriggerPressed = false;
        }
        if (gamepad2.left_trigger > 0.1) {
            IsLeftTriggerPressed = true;
        } else {
            IsLeftTriggerPressed = false;
        }
        if (Lift1.getCurrentPosition() > -400) {
            if (IsRightTriggerPressed == true) {
                Lift1.setPower(gamepad2.right_trigger * 0.2);
                Lift2.setPower(gamepad2.right_trigger * 0.2);
                LastLift1Encoder = Lift1.getCurrentPosition();
                LastLift2Encoder = Lift2.getCurrentPosition();
            } else {
                Lift1.setPower(-1 * gamepad2.left_trigger * 0.4);
                Lift2.setPower(-1 * gamepad2.left_trigger * 0.4);
                LastLift1Encoder = Lift1.getCurrentPosition();
                LastLift2Encoder = Lift2.getCurrentPosition();
            }
        } else {
            if (IsRightTriggerPressed == true) {
                Lift1.setPower(gamepad2.right_trigger);
                Lift2.setPower(gamepad2.right_trigger);
                LastLift1Encoder = Lift1.getCurrentPosition();
                LastLift2Encoder = Lift2.getCurrentPosition();
            } else {
                Lift1.setPower(-1 * gamepad2.left_trigger);
                Lift2.setPower(-1 * gamepad2.left_trigger);
                LastLift1Encoder = Lift1.getCurrentPosition();
                LastLift2Encoder = Lift2.getCurrentPosition();
            }
        }
        if (!IsLeftTriggerPressed && !IsRightTriggerPressed) {
            Lift1.setPower(PIDLift(LastLift1Encoder, Lift1.getCurrentPosition()));
            Lift2.setPower(PIDLift(LastLift2Encoder, Lift2.getCurrentPosition()));
        }
        if (Lift1.getCurrentPosition() > -70) {
            Diff2.setPosition(Diff2Rest + 0.01);
            Diff1.setPosition(Diff1Rest + 0.01);
        } else if (Lift1.getCurrentPosition() < -70 && Lift1.getCurrentPosition() > -350 || IsRightTriggerPressed == true) {
            Diff2.setPosition(Diff2Rest - 0.04);
            Diff1.setPosition(Diff1Rest - 0.04);
        } else {
            if (gamepad2.dpad_right) {
                Diff2.setPosition(Diff2Rest - 0.14);
                Diff1.setPosition(Diff1Rest - 0.43);
            } else {
                Diff2.setPosition(Diff2Rest + -0.08);
                Diff1.setPosition(Diff2Rest + 0.65);
            }
        }
        if (gamepad2.back) {
            Plane.setPosition(0.2);
        } else {
            Plane.setPosition(0.5);
        }
        if (gamepad2.a) {
            Outtake.setPosition(0.25);
        }
        if (gamepad2.b) {
            Outtake.setPosition(0.6);
        }
        if (gamepad2.left_bumper) {
            IntakeL.setPower(1);
            IntakeR.setPower(1);
        } else if (gamepad2.right_bumper) {
            IntakeL.setPower(-1);
            IntakeR.setPower(-1);
        } else {
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        }
    }

    public double PIDLift(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lasterror) * timer.seconds();
        lasterror = error;
        timer.reset();
        output = (error * kP) + (integralSum * kI) + (derivative * kD);
        return output;
    }

}
