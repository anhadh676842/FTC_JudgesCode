package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Diff Swerve TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;
    Climb climber;
    LiftAndDiff liftanddiff;

    public static double rightm1P = 1;
    public static double rightm1I = 0;
    public static double rightm1D = 0;
    public static double rightm1f = 0;
    public static double rightm2P = 1;
    public static double rightm2I = 0;
    public static double rightm2f = 0;
    public static double rightm2D = 0;
    public static double leftm1P = 1;
    public static double leftm1I = 0;
    public static double leftm1D = 0;
    public static double leftm1f = 0;
    public static double leftm2P = 1;
    public static double leftm2I = 0;
    public static double leftm2D = 0;
    public static double leftm2f = 0;



    //deadband for joysticks
    public double DEADBAND_MAG = 0.1;
    public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);

    public boolean willResetIMU = true;

    public void init() {
        robot = new Robot(this, false);
        climber = new Climb(robot.driveController.moduleRight.Encoder, robot.driveController.moduleLeft.Encoder);
        liftanddiff = new LiftAndDiff(robot.Lift1, robot.Lift2, robot.IntakeL, robot.IntakeR, robot.Outtake, robot.Plane, robot.Diff1, robot.Diff2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    //allows driver to indicate that the IMU should not be reset
    //used when starting TeleOp after auto or if program crashes in the middle of match
    //relevant because of field-centric controls
    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
    }
    public void start () {
        if (willResetIMU) robot.initIMU();
    }


    public void loop() {
        robot.driveController.moduleLeft.motor1.setVelocityPIDFCoefficients(leftm1P, leftm1I, leftm1D, leftm1f);
        robot.driveController.moduleLeft.motor2.setVelocityPIDFCoefficients(leftm2P, leftm2I, leftm2D, leftm2f);
        robot.driveController.moduleRight.motor1.setVelocityPIDFCoefficients(rightm1P, rightm1I, rightm1D, rightm1f);
        robot.driveController.moduleRight.motor1.setVelocityPIDFCoefficients(rightm2P, rightm2I, rightm2D, rightm2f);
        Vector2d joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        Vector2d joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad2.right_stick_y); //RIGHT joystick
        robot.driveController.updateUsingJoysticks(checkDeadband(joystick1), checkDeadband(joystick2));
        climber.climbcheck(gamepad2);
        liftanddiff.LiftAndDiffCheck(gamepad2);


        //uncomment for live tuning of ROT_ADVANTAGE constant


        telemetry.addData("LeftM1Velocity", robot.driveController.moduleLeft.motor1.getVelocity());
        telemetry.addData("LeftM2Velocity", robot.driveController.moduleLeft.motor2.getVelocity());
        telemetry.addData("RightM1Velocity", robot.driveController.moduleRight.motor1.getVelocity());
        telemetry.addData("RightM2Velocity", robot.driveController.moduleRight.motor2.getVelocity());
        telemetry.addData("LeftM1Target", robot.driveController.moduleLeft.motor1.getPower()*2600);
        telemetry.addData("LeftM2Target", robot.driveController.moduleLeft.motor2.getPower()*2600);
        telemetry.addData("RighttM1Target", robot.driveController.moduleRight.motor1.getPower()*2600);
        telemetry.addData("RightM2Target", robot.driveController.moduleRight.motor2.getPower()*2600);
        telemetry.update();
    }

    public void stop() {
        robot.driveController.moduleLeft.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveController.moduleLeft.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveController.moduleRight.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveController.moduleRight.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //returns zero vector if joystick is within deadband
    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return Vector2d.ZERO;
    }
}