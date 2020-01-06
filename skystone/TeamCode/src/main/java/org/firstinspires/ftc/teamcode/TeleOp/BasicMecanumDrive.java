package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@TeleOp(group = "Basic Drivetrain")
public class BasicMecanumDrive extends OpMode {

    private SampleMecanumDriveREVOptimized drive;

    private double[] motorPowers = new double[]{0, 0, 0, 0};
    private double x, y, rotation, maxPower, theta, cos, sin, tempx, startingDirection = Math.toRadians(90);
    private Double absoluteRotation;

    private boolean aPressed = false, yPressed = false, y2Pressed = false, xPressed = false, down = false, up = false;

    private PIDFController absoluteRotationPIDController, liftController;

    public static double skystoneHeightChange = 4, maxSlideHeight = 38, firstSkystoneHeight = 1.5;

    public static double maxLiftPower = 1, maxIntakePower = 1, SloMoPower = 0.5;


    @Override
    public void init() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
    }

    @Override
    public void start() {

    }
    @Override
    public void loop() {

        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        x = x * Math.abs(x);
        y = y * Math.abs(y);

        rotation = gamepad1.right_stick_x;


        motorPowers = new double[]{y + x + rotation, y - x + rotation, y + x - rotation, y - x - rotation};

        if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {
            maxPower = GetMaxAbsMotorPower();
            drive.setMotorPowers(motorPowers[0] / maxPower, motorPowers[1] / maxPower, motorPowers[2] / maxPower, motorPowers[3] / maxPower);
        } else {
            drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
        }

        drive.setIntakePower(maxIntakePower * -Math.pow(gamepad2.left_stick_y, 3), maxIntakePower * -Math.pow(gamepad2.right_stick_y, 3));

        if (gamepad2.a) {
            aPressed = true;
        } else if (aPressed) {
            drive.toggleClaw();
            aPressed = false;
        }

        if (gamepad1.y) {
            yPressed = true;
        } else if (yPressed) {
//            liftController.setTargetPosition(0);
            drive.setClawGrabbing(false);
//            drive.resetArm();
            yPressed = false;
        }

        if (gamepad1.x) {
            xPressed = true;
        } else if (xPressed) {
            drive.toggleArm();
            xPressed = false;
        }

        if (gamepad2.y) {
            y2Pressed = true;
        } else if (y2Pressed) {
            drive.toggleFoundation();
            y2Pressed = false;
        }

        if (gamepad1.left_bumper) {
            drive.DecArm();
        }
        if (gamepad1.right_bumper) {
            drive.IncArm();
        }


        drive.setLiftPower(maxLiftPower * (gamepad2.right_trigger - gamepad2.left_trigger));
    }

    private double GetMaxAbsMotorPower() {
        return Math.max(Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1])), Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));
    }
}
