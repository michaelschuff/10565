package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.HEADING_PID;

import java.io.File;
import java.util.Scanner;

@Config
@TeleOp(group = "Basic Drivetrain")
public class FieldCentricMecanumDrive extends OpMode {
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
//        try {
//            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");
//            Scanner sc = new Scanner(file);
//            startingDirection = Float.parseFloat(sc.nextLine());
//            sc.close();
//        } catch (Exception e){
//
//        }

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);


//        absoluteRotationPIDController = new PIDFController(HEADING_PID);
//        absoluteRotationPIDController.setInputBounds(0.0, 2.0 * Math.PI);
//        absoluteRotationPIDController.setOutputBounds(-1.0, 1.0);

//        liftController = new PIDFController(new PIDCoefficients(3, 0, 0));
//        liftController.setOutputBounds(-1, 1);
//        liftController.setInputBounds(0, 0);
//        liftController.setTargetPosition(0);
    }

    @Override
    public void loop() {
        drive.setIntakePower(-1, -1);
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




//        absoluteRotation = getDPadAngle((gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0), (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
//        if (absoluteRotation != null) {
//            absoluteRotationPIDController.setTargetPosition(absoluteRotation);
//        }

        rotation = Math.pow(gamepad1.right_stick_x, 3);
//        if (rotation == 0) {
//            rotation = absoluteRotationPIDController.update(drive.getRawExternalHeading());
//        }


        motorPowers = new double[]{x + rotation, y + rotation, x - rotation, y - rotation};

        if (gamepad1.b) {
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
            drive.resetArm();
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

//        telemetry.addData("fl",motorPowers[0]);
//        telemetry.addData("bl",motorPowers[1]);
//        telemetry.addData("br",motorPowers[2]);
//        telemetry.addData("fr",motorPowers[3]);
//        telemetry.update();
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
