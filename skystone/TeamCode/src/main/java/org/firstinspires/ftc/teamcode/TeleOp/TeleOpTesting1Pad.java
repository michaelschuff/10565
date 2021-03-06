package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.HEADING_PID;

import java.io.File;
import java.util.Scanner;

@Disabled
@Config
@TeleOp(group = "Basic Drivetrain")
public class TeleOpTesting1Pad extends OpMode {
    private SampleMecanumDriveREVOptimized drive;

    private double[] motorPowers = new double[]{0, 0, 0, 0};
    private double x, y, rotation, maxPower, theta, cos, sin, tempx, startingDirection = Math.toRadians(90);

    private boolean aPressed = false, yPressed = false, y2Pressed = false, xPressed = false, down = false, up = false, isResetting = false, V4BarOut = false, fActivated = false, rightBumpPressed = false, leftBumpPressed = false, bPressed = false;

    public static double maxLiftPower = 1, maxIntakePower = 1, SloMoPower = 0.5, firstStoneVal = 0.8, secondStoneVal = 0.6;

    @Override
    public void init() {
        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");
            Scanner sc = new Scanner(file);
            startingDirection = Float.parseFloat(sc.nextLine());
            sc.close();
        } catch (Exception e){

        }

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setRunUsingEncoder(false);
        drive.setClawGrabbing(true);
        drive.setArmIn(true);
        drive.setClawGrabbing(false);
    }

    @Override
    public void loop() {
        tempx = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;

        double tempTheta = Math.atan2(y, tempx);

        double mag = Math.pow(Math.sqrt(tempx*tempx + y*y), 1);

        tempx = mag * Math.cos(tempTheta);
        y = mag * Math.sin(tempTheta);

        theta = -startingDirection - drive.getRawExternalHeading() + Math.toRadians(45);
        cos = Math.cos(theta);
        sin = Math.sin(theta);
        x = tempx * cos - y * sin;
        y = tempx * sin + y * cos;

        rotation = Math.pow(gamepad1.right_stick_x, 3);


        motorPowers = new double[]{x + rotation, y + rotation, x - rotation, y - rotation};

        if (V4BarOut || fActivated) {
            if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {
                maxPower = GetMaxAbsMotorPower();
                drive.setMotorPowers(SloMoPower * motorPowers[0] / maxPower, SloMoPower * motorPowers[1] / maxPower, SloMoPower * motorPowers[2] / maxPower, SloMoPower * motorPowers[3] / maxPower);
            } else {
                drive.setMotorPowers(SloMoPower * motorPowers[0], SloMoPower * motorPowers[1], SloMoPower * motorPowers[2], SloMoPower * motorPowers[3]);
            }
        } else {
            if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {
                maxPower = GetMaxAbsMotorPower();
                drive.setMotorPowers(motorPowers[0] / maxPower, motorPowers[1] / maxPower, motorPowers[2] / maxPower, motorPowers[3] / maxPower);
            } else {
                drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            }
        }

        if (gamepad1.right_bumper) {
            rightBumpPressed = true;
        } else  if (rightBumpPressed) {
            drive.setArmPos(secondStoneVal, 1 - secondStoneVal);
            rightBumpPressed = false;
        }

        if (gamepad1.left_bumper) {
            leftBumpPressed = true;
        } else  if (leftBumpPressed) {
            drive.setArmPos(firstStoneVal, 1 - firstStoneVal);
            leftBumpPressed = false;
        }

        if (gamepad1.a) {
            aPressed = true;
        } else if (aPressed) {
            drive.toggleClaw();
            aPressed = false;
        }

        if (gamepad1.b) {
            bPressed = true;
        } else if (bPressed) {
            drive.toggleIntake();
            bPressed = false;
        }

        if (gamepad1.y) {
            yPressed = true;
        } else if (yPressed) {
            if (V4BarOut) {
                drive.resetEveryThing();
                isResetting = true;
            } else {
                fActivated = !fActivated;
                drive.toggleFoundation();
            }

            yPressed = false;
        }

        if (gamepad1.x) {
            xPressed = true;
        } else if (xPressed) {
            drive.toggleArm();
            xPressed = false;
        }

//        if (gamepad1.b) {
//            y2Pressed = true;
//        } else if (y2Pressed) {
//            fActivated = !fActivated;
//            drive.toggleFoundation();
//            y2Pressed = false;
//        }

        if (!isResetting) {
            drive.setLiftPower(maxLiftPower * (gamepad1.right_trigger - gamepad1.left_trigger));
        } else {
            if (drive.CheckLiftPos()) {
                isResetting = false;
            }
        }


        drive.updateClawGrabbed();

        V4BarOut = !drive.getIsArmIn();
    }

    private Double getDPadAngle(int x, int y) {
        if (x != 0 || y != 0) {
            return Math.atan2(y, x);
        }
        return null;
    }

    private double GetMaxAbsMotorPower() {
        return Math.max(Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1])), Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));
    }
}
