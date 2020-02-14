package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonReader;
import org.firstinspires.ftc.teamcode.util.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.util.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.util.gamepad.TriggerReader;

import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.HEADING_PID;

import java.io.File;
import java.util.Scanner;

@Config
@TeleOp(group = "Basic Drivetrain")
public class FieldCentricMecanumDrive extends OpMode {
    private SampleMecanumDriveREVOptimized drive;

    private GamepadEx driver1, driver2;

    private double[] motorPowers = new double[]{0, 0, 0, 0};
    private double x, y, rotation, maxPower, theta, startingDirection = Math.toRadians(-90);

    private boolean isResetting = false, V4BarOut = false, fActivated = false;

    private ButtonReader a2, y1, y2, x1, x2, rBump1, lBump1, dUp2, dDown2, dLeft2, dRight2;

    public static double maxLiftPower = 1, maxIntakePower = 1, SloMoPower = .5, firstStoneVal = 0.8, secondStoneVal = 0.7, DownLiftPow = 1;

    @Override
    public void init() {
        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");
            Scanner sc = new Scanner(file);
            startingDirection = Float.parseFloat(sc.nextLine());
            sc.close();
        } catch (Exception e){
            telemetry.addLine("Could not load start heading");
            telemetry.update();
        }

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setRunUsingEncoder(false);
        drive.setClawGrabbing(true);
        drive.setArmIn(true);
        drive.setClawGrabbing(false);
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);


        x1 = new ButtonReader(driver1, GamepadKeys.Button.X);
        y1 = new ButtonReader(driver1, GamepadKeys.Button.Y);
        rBump1 = new ButtonReader(driver1, GamepadKeys.Button.RIGHT_BUMPER);
        lBump1 = new ButtonReader(driver1, GamepadKeys.Button.LEFT_BUMPER);


        a2 = new ButtonReader(driver2, GamepadKeys.Button.A);
        y2 = new ButtonReader(driver2, GamepadKeys.Button.Y);
        x2 = new ButtonReader(driver2, GamepadKeys.Button.X);
        dUp2 = new ButtonReader(driver2, GamepadKeys.Button.DPAD_UP);
        dRight2 = new ButtonReader(driver2, GamepadKeys.Button.DPAD_RIGHT);
        dDown2 = new ButtonReader(driver2, GamepadKeys.Button.DPAD_DOWN);
        dLeft2 = new ButtonReader(driver2, GamepadKeys.Button.DPAD_LEFT);

    }

    @Override
    public void loop() {
        x1.readValue();
        y1.readValue();
        rBump1.readValue();
        lBump1.readValue();
        a2.readValue();
        x2.readValue();
        y2.readValue();
        dUp2.readValue();
        dRight2.readValue();
        dDown2.readValue();
        dLeft2.readValue();

        x = driver1.getLeftX();
        y = driver1.getLeftY();

        theta = (Math.atan2(y, x) - drive.getExternalHeading() + startingDirection - Math.toRadians(135)) % (2 * Math.PI);

        if (theta < 0) {
            theta = 2 * Math.PI + theta;
        }

        double mag = Math.pow(Math.sqrt(x*x + y*y), 2);

        if (3 * Math.PI / 4 >= theta && theta >= Math.PI / 4) {
            y = mag;
            x = y / Math.tan(theta);
        } else if ((Math.PI / 4 >= theta && theta >= 0) || (7 * Math.PI / 4 <= theta && theta <= 2 * Math.PI)) {
            x = mag;
            y = x * Math.tan(theta);
        } else if (5 * Math.PI / 4 <= theta && theta <= 7 * Math.PI / 4) {
            y = -mag;
            x = y / Math.tan(theta);
        } else if (3 * Math.PI / 4 <= theta && theta <= 5 * Math.PI / 4) {
            x = -mag;
            y = x * Math.tan(theta);
        }

        rotation = Math.pow(driver1.getRightX(), 1) * Math.abs(driver1.getRightX());


        motorPowers = new double[]{x + rotation, y + rotation, x - rotation, y - rotation};

        if (V4BarOut || fActivated) {
            if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {
                maxPower = GetMaxAbsMotorPower();
                drive.setMotorPowers(adjustPower(SloMoPower * motorPowers[0] / maxPower), adjustPower(SloMoPower * motorPowers[1] / maxPower), adjustPower(SloMoPower * motorPowers[2] / maxPower), adjustPower(SloMoPower * motorPowers[3] / maxPower));
            } else {
                drive.setMotorPowers(adjustPower(SloMoPower * motorPowers[0]), adjustPower(SloMoPower * motorPowers[1]), adjustPower(SloMoPower * motorPowers[2]), adjustPower(SloMoPower * motorPowers[3]));
            }
        } else {
            if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {
                maxPower = GetMaxAbsMotorPower();
                drive.setMotorPowers(adjustPower(motorPowers[0] / maxPower), adjustPower(motorPowers[1] / maxPower), adjustPower(motorPowers[2] / maxPower), adjustPower(motorPowers[3] / maxPower));
            } else {
                drive.setMotorPowers(adjustPower(motorPowers[0]), adjustPower(motorPowers[1]), adjustPower(motorPowers[2]), adjustPower(motorPowers[3]));
            }
        }

        drive.setIntakePower(maxIntakePower * Math.pow(driver2.getLeftY(), 1) * Math.abs(driver2.getLeftY()), maxIntakePower * Math.pow(driver2.getRightY(), 1) * Math.abs(driver2.getRightY()));


        if (rBump1.wasJustPressed()) {
            drive.setArmPos(secondStoneVal, 1 - secondStoneVal);
        }

        if (lBump1.wasJustPressed()) {
            drive.setArmPos(firstStoneVal, 1 - firstStoneVal);
        }

        if (a2.wasJustPressed()) {
            drive.toggleClaw();
        }

        if (y1.wasJustPressed()) {
            drive.resetEveryThing();
            isResetting = true;
        }

        if (x1.wasJustPressed()) {
            drive.toggleArm();
        }

        if (x2.wasJustPressed()) {
            drive.releaseCapStone();
        }

        if (y2.wasJustPressed()) {
            fActivated = !fActivated;
            drive.toggleFoundation();
        }

        if (dUp2.wasJustPressed()) {
            startingDirection = Math.toRadians(90) - drive.getRawExternalHeading();
        }

        if (dRight2.wasJustPressed()) {
            startingDirection = -drive.getRawExternalHeading();
        }

        if (dDown2.wasJustPressed()) {
            startingDirection = -drive.getRawExternalHeading() - Math.toRadians(90);
        }

        if (dLeft2.wasJustPressed()) {
            startingDirection = Math.toRadians(180) - drive.getRawExternalHeading();
        }

        if (drive.getLiftPos() < 750) {
            DownLiftPow = .05 + Math.abs(drive.getLiftPos() / 750.0);
        } else {
            DownLiftPow = 1;
        }

        if (!isResetting) {
            drive.setLiftPower(maxLiftPower * (driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - DownLiftPow * driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        } else {
            if (drive.CheckLiftPos()) {
                isResetting = false;
            }
        }

        V4BarOut = !drive.getIsArmIn();

        drive.updateClawGrabbed();


        telemetry.addData("startingDirection", startingDirection);
        telemetry.addData("liftpos", drive.getLiftPos());
        telemetry.update();
    }

    private double adjustPower(double power) {
        double minPower = 0.15;
        if (power > 0) {
            return power * (1 - minPower) + minPower;
        } else if (power < 0) {
            return power * (1 - minPower) - minPower;
        }
        return 0;
    }

    private double GetMaxAbsMotorPower() {
        return Math.max(Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1])), Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));
    }
}
