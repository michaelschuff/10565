package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.HEADING_PID;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Scanner;

@Config
@TeleOp(group = "Basic Drivetrain")
public class FieldCentricMecanumDrive extends OpMode {
    private SampleMecanumDriveREVOptimized drive;

    private double[] motorPowers = new double[]{0, 0, 0, 0};
    private double x, y, rotation, maxPower, theta, cos, sin, tempx, startingDirection;
    private Double absoluteRotation;

    private boolean aPressed = false, yPressed = false, y2Pressed = false, xPressed = false, down = false, up = false;

    private PIDFController absoluteRotationPIDController;

    public static double skystoneHeightChange = 4, maxSlideHeight = 12, firstSkystoneHeight = 1;

    public static double maxLiftPower = 0.75, maxIntakePower = 1;

    @Override
    public void init() {
        try {
            File file = new File("../Data/StartingDirection.txt");
            Scanner sc = new Scanner(file);
            startingDirection = Math.toRadians(Integer.parseInt(sc.nextLine()));
            sc.close();
        } catch (Exception e){

        }

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);


        absoluteRotationPIDController = new PIDFController(HEADING_PID);
        absoluteRotationPIDController.setInputBounds(0.0, 2.0 * Math.PI);
        absoluteRotationPIDController.setOutputBounds(-1.0, 1.0);
        absoluteRotationPIDController.setTargetPosition(startingDirection);

//        liftController = new PIDFController(new PIDCoefficients(3, 0, 0));
//        liftController.setOutputBounds(-1, 1);
//        liftController.setInputBounds(0, 0);
//        liftController.setTargetPosition(0);
    }

    @Override
    public void loop() {
        tempx = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;

        double tempTheta = Math.atan2(y, tempx);

        double mag = tempx*tempx + y*y;

        tempx = mag * Math.cos(tempTheta);
        y = mag * Math.sin(tempTheta);

        theta = Math.toRadians(90) - startingDirection - drive.getRawExternalHeading();
        cos = Math.cos(theta);
        sin = Math.sin(theta);
        x = tempx * cos - y * sin;
        y = tempx * sin + y * cos;
        x = x * Math.abs(x);
        y = y * Math.abs(y);




        absoluteRotation = getDPadAngle((gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0), (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
        if (absoluteRotation != null) {
            absoluteRotation = absoluteRotationPIDController.update(absoluteRotation);
        } else {
            rotation = Math.pow(gamepad1.right_stick_x, 3);
        }
        x = x * Math.abs(x);
        y = y * Math.abs(y);

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
            xPressed = true;
        } else if (xPressed) {
//            liftController.setTargetPosition(0);
            drive.setClawGrabbing(false);
            drive.resetArm();
            xPressed = false;
        }

        if (gamepad1.x) {
            y2Pressed = true;
        } else if (y2Pressed) {
            drive.toggleArm();
            y2Pressed = false;
        }

        if (gamepad2.y) {
            yPressed = true;
        } else if (yPressed) {
            drive.toggleFoundation();
            yPressed = false;
        }

        if (gamepad1.dpad_down) {
            drive.DecArm();
        }
        if (gamepad1.dpad_up) {
            drive.IncArm();
        }

//        if (gamepad2.dpad_down) {
//            down = true;
//        } else if (down) {
//            liftController.setTargetPosition(liftController.getTargetPosition() - skystoneHeightChange);
//            down = false;
//        }

//        if (gamepad2.dpad_up) {
//            up = true;
//        } else if (up) {
//            if (liftController.getTargetPosition() < firstSkystoneHeight) {
//                liftController.setTargetPosition(firstSkystoneHeight);
//            } else {
//                liftController.setTargetPosition(liftController.getTargetPosition() + skystoneHeightChange);
//            }
//            up = false;
//        } else if (!gamepad2.dpad_down){
//            liftController.setTargetPosition(liftTicksToInches(drive.lift.getCurrentPosition()));
//        }

//        if (Math.abs(Math.pow(gamepad2.right_trigger, 3) - Math.pow(gamepad2.left_trigger, 3)) < .01) {
//            drive.setLiftPower(maxPower * liftController.update(liftTicksToInches(drive.lift.getCurrentPosition())));
//        } else {
//            drive.setLiftPower(maxPower * (Math.pow(gamepad2.right_trigger, 3) - Math.pow(gamepad2.left_trigger, 3)));
//        }

        drive.setLiftPower(maxLiftPower * (gamepad2.right_trigger - gamepad2.left_trigger));
        telemetry.addData("liftPower",maxPower * (gamepad2.right_trigger - gamepad2.left_trigger));
        telemetry.addData("leftServo", drive.lArm.getPosition());
        telemetry.addData("rightServo", drive.rArm.getPosition());
        telemetry.addData("upDpad", gamepad1.dpad_up);
        telemetry.addData("downDpad", gamepad1.dpad_down);
        telemetry.update();
    }

    private double liftTicksToInches(int ticks) {
        return ticks / 103.6;
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
