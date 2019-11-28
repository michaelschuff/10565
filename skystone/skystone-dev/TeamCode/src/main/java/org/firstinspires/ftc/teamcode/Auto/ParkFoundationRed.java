package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

@Autonomous(group = "drive")
public class ParkFoundationRed extends LinearOpMode {
    public DcMotorEx fl, bl, br, fr;

    public int lastLeftEncoder, lastRightEncoder, lastMiddleEncoder;

    double[] samplingVals = {.19, .55};
    double[] intakeVals = {.8, 1};
    double[] outtakeVals = {.9, .5};
    double[] foundationVals = {.35, .7};

    Servo samplingServo = null;
    Servo intakeServo = null;
    Servo outtakeServo = null;
    Servo[] foundationServos = new Servo[2];

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");

        samplingServo = hardwareMap.servo.get("samplingServo");
        intakeServo = hardwareMap.servo.get("intakeServo");
        outtakeServo = hardwareMap.servo.get("outtakeServo");
        foundationServos[0] = hardwareMap.servo.get("backFoundationServo");
        foundationServos[1] = hardwareMap.servo.get("frontFoundationServo");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        lastLeftEncoder = bl.getCurrentPosition();
        lastRightEncoder = br.getCurrentPosition();
        lastMiddleEncoder = fl.getCurrentPosition();

        waitForStart();

        fl.setPower(-1);//left
        bl.setPower(1);
        fr.setPower(1);
        br.setPower(-1);

        sleep(1500);

//        double[] frontfoundationVals = {.65, 1};
//        double[] backfoundationVals = {.35, 0};
        foundationServos[0].setPosition(0);
        foundationServos[1].setPosition(1);

        sleep(500);

        fl.setPower(1);//right
        bl.setPower(-1);
        fr.setPower(-1);
        br.setPower(1);

        sleep(2500);

        foundationServos[0].setPosition(.35);
        foundationServos[1].setPosition(.65);

        sleep(500);


        fl.setPower(-1);//backward
        bl.setPower(-1);
        fr.setPower(-1);
        br.setPower(-1);

        sleep(250);

//        samplingServo.setPosition(1);

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        //start at -63 **-24

//        right(38);

        //check block
//        if (/*above is*/true) {
        //get into position to grab
//        } else {
//            backward(4);//and repeat
//        }

        //aftergrabbed skystone
//        left(6);

//        forward(0);//until across bridge

        //drop blo ck

        /*
            start at -63, -28 facing 0

            drive backwards until notice skystone
            calculate when to drop flopper
            drive and drop flopper
            get which pos final skystone is
            go to current yos + 8
            go to 9, 36
            open flopper
            go to 18 36
            go to 18 15
            go to 24 15
            close wafflemaker grabber
            go to 24 63
            open wafflemaker grabber
            go to 0 63
         */


    }

//    public void forward(double distance) {
//        while (avg(encoderTicksToInches(br.getCurrentPosition() - lastRightEncoder, 1050), encoderTicksToInches(bl.getCurrentPosition() - lastLeftEncoder, 1330)) < distance) {
//            setMotorPower((distance - avg(encoderTicksToInches(br.getCurrentPosition(), 1050), encoderTicksToInches(bl.getCurrentPosition() - lastLeftEncoder, 1330))) / distance);
//        }
//        setMotorPower(0);
//
//        lastRightEncoder = br.getCurrentPosition();
//        lastLeftEncoder = bl.getCurrentPosition();
//    }
//
//    public void backward(double distance) {
//        while (avg(encoderTicksToInches(br.getCurrentPosition() - lastRightEncoder, 1050), encoderTicksToInches(bl.getCurrentPosition() - lastLeftEncoder, 1330)) < -distance) {
//            setMotorPower((distance - Math.abs(avg(encoderTicksToInches(br.getCurrentPosition() - lastRightEncoder, 1050), encoderTicksToInches(bl.getCurrentPosition() - lastLeftEncoder, 1330)))) / distance);
//        }
//        setMotorPower(0);
//
//        lastRightEncoder = br.getCurrentPosition();
//        lastLeftEncoder = bl.getCurrentPosition();
//    }
//
//    public void right(double distance) {
//        while (encoderTicksToInches(fl.getCurrentPosition() - lastMiddleEncoder, 1050) < distance) {
//            setMotorPowerSideways((distance - encoderTicksToInches(fl.getCurrentPosition() - lastMiddleEncoder, 1050)) / distance);
//        }
//        setMotorPower(0);
//
//        lastMiddleEncoder = fl.getCurrentPosition();
//    }
//
//    public void left(double distance) {
//        while (encoderTicksToInches(fl.getCurrentPosition() - lastMiddleEncoder, 1050) < distance) {
//            setMotorPowerSideways((distance - encoderTicksToInches(fl.getCurrentPosition() - lastMiddleEncoder, 1050)) / distance);
//        }
//        setMotorPower(0);
//
//        lastMiddleEncoder = fl.getCurrentPosition();
//    }
//
//    public void turnLeft(double radians) {
//        double odomRadius = 5.3125;
//
//        while (encoderTicksToInches(fl.getCurrentPosition() - lastMiddleEncoder, 1050) < radians * odomRadius) {
//            setMotorPowerRotate(((radians * odomRadius) - encoderTicksToInches(fl.getCurrentPosition() - lastMiddleEncoder, 1050)) / (radians * odomRadius));
//        }
//        setMotorPower(0);
//
//        lastMiddleEncoder = fl.getCurrentPosition();
//
//    }
//
//    public void turnRight(double radians) {
//        radians = -radians;
//        double odomRadius = 5.3125;
//
//        while (encoderTicksToInches(fl.getCurrentPosition() - lastMiddleEncoder, 1050) < radians * odomRadius) {
//            setMotorPowerRotate(((radians * odomRadius) - encoderTicksToInches(fl.getCurrentPosition() - lastMiddleEncoder, 1050)) / (radians * odomRadius));
//        }
//        setMotorPower(0);
//
//        lastMiddleEncoder = fl.getCurrentPosition();
//
//    }
//
//    public double encoderTicksToInches(int ticks, int whichMotor) {
//        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / whichMotor;
//    }
//
//    public double avg(double a, double b) {
//        return (a + b) / 2;
//    }
//
//    public void setMotorPower(double a) {
//        fl.setPower(a);
//        bl.setPower(a);
//        fr.setPower(a);
//        br.setPower(a);
//    }
//
//    public void setMotorPowerSideways(double a) {
//        fl.setPower(-a);
//        bl.setPower(a);
//        fr.setPower(a);
//        br.setPower(-a);
//    }
//
//    public void setMotorPowerRotate(double a) {
//        fl.setPower(a);
//        bl.setPower(a);
//        fr.setPower(-a);
//        br.setPower(-a);
//    }
}
