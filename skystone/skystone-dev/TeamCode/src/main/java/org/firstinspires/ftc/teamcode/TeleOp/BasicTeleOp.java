package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;

@Config
@TeleOp(name="Basic TeleOP", group = "TeleOp")
public class BasicTeleOp extends OpMode {


    DcMotor[] driveMotors = new DcMotor[4];
    DcMotor[] intakeMotors = new DcMotor[2];

    Servo samplingServo = null;
    Servo intakeServo = null;
    Servo outtakeServo = null;
    Servo[] foundationServos = new Servo[2];

    public int lastLeftEncoder, lastRightEncoder, lastMiddleEncoder;

    double xVelocity, yVelocity, angle, magnitude;
    double flPower, frPower, blPower, brPower, rotation, maxPower;

    double[] samplingVals = {.19, .55};
    double[] intakeVals = {.8, 1};
    double[] outtakeVals = {.9, .5};
    double[] foundationVals = {.35, .7};

    boolean isFoundationClampDown = false;

    boolean a = false, b = false, x = false, y = false;
    boolean isA = false, isB = false, isX = false, isY = false;

    boolean first = true;

    @Override
    public void init() {
        driveMotors[0] = hardwareMap.dcMotor.get("br");
        driveMotors[1] = hardwareMap.dcMotor.get("bl");
        driveMotors[2] = hardwareMap.dcMotor.get("fr");
        driveMotors[3] = hardwareMap.dcMotor.get("fl");

        intakeMotors[0] = hardwareMap.dcMotor.get("lintake");
        intakeMotors[1] = hardwareMap.dcMotor.get("rintake");

        samplingServo = hardwareMap.servo.get("samplingServo");
        intakeServo = hardwareMap.servo.get("intakeServo");
        outtakeServo = hardwareMap.servo.get("outtakeServo");
        foundationServos[0] = hardwareMap.servo.get("backFoundationServo");
        foundationServos[1] = hardwareMap.servo.get("frontFoundationServo");

        driveMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){

        if (gamepad2.right_trigger > 0) {
            outtakeServo.setPosition(outtakeVals[1]);
        } else {
            outtakeServo.setPosition(outtakeVals[0]);
        }

        if (gamepad2.x) {
            x = true;
        } else if (x == true) {
            isX = !isX;
            x = false;
            if (isX) {
                foundationServos[0].setPosition(0);
                foundationServos[1].setPosition(1);
            } else {
                foundationServos[0].setPosition(.35);
                foundationServos[1].setPosition(.65);
            }
        }
        if (gamepad2.a) {
            a = true;
        } else if (a == true) {
            isA = !isA;
            a = false;
            if (isA) {
                intakeServo.setPosition(intakeVals[0]);
            } else {
                intakeServo.setPosition(intakeVals[1]);
            }
        } else {
            first = false;
        }
//
//        if (gamepad1.b) {
//            b = true;
//        } else if (b == true) {
//            isB = !isB;
//            b = false;
//            if (isB) {
//                outtakeServo.setPosition(outtakeVals[0]);
//            } else {
//                outtakeServo.setPosition(outtakeVals[1]);
//            }
//        }
//
        if (gamepad2.y) {
            y = true;
        } else if (y == true) {
            isY = !isY;
            y = false;
            if (isY) {
                samplingServo.setPosition(samplingVals[0]);
            } else {
                samplingServo.setPosition(samplingVals[1]);
            }
        }
//
//        if (gamepad1.y) {
//            y = true;
//        } else if (y == true) {
//            isY = !isY;
//            y = false;
//            if (isY) {
//                foundationServos[0].setPosition(foundationVals[0]);
//                foundationServos[1].setPosition(-foundationVals[0]);
//            } else {
//                foundationServos[0].setPosition(foundationVals[1]);
//                foundationServos[1].setPosition(-foundationVals[1]);
//            }
//        }

//        if (gamepad1.a) {
//            backward(10);
//        }
//        if (gamepad1.b) {
//            left(10);
//        }
//        if (gamepad1.x) {
//            right(10);
//        }
//        if (gamepad1.y) {
//            forward(10);
//        }
//        if (gamepad1.right_bumper) {
//            turnRight(Math.PI / 2);
//        }
//        if (gamepad1.left_bumper) {
//            turnLeft(Math.PI / 2);
//        }

        maxPower = 0.7;

        xVelocity = Math.pow(gamepad1.left_stick_x, 3);
        yVelocity = Math.pow(gamepad1.left_stick_y, 3);

        xVelocity = Range.clip(xVelocity, -1, 1);
        yVelocity = Range.clip(yVelocity, -1, 1);


        rotation = -Math.pow(gamepad1.right_stick_x, 3);

        angle = Math.atan2(xVelocity, yVelocity) - Math.PI;
        magnitude = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));

        flPower = magnitude * Math.sin(-angle + Math.PI / 4) - rotation;
        frPower = magnitude * Math.cos(-angle + Math.PI / 4) + rotation;
        blPower = magnitude * Math.cos(-angle + Math.PI / 4) - rotation;
        brPower = magnitude * Math.sin(-angle + Math.PI / 4) + rotation;

        if(Math.abs(flPower) > maxPower)
            maxPower = Math.abs(flPower);
        if(Math.abs(frPower) > maxPower)
            maxPower = Math.abs(frPower);
        if(Math.abs(blPower) > maxPower)
            maxPower = Math.abs(blPower);
        if(Math.abs(brPower) > maxPower)
            maxPower = Math.abs(brPower);

        flPower = flPower / maxPower;
        frPower = frPower / maxPower;
        blPower = blPower / maxPower;
        brPower = brPower / maxPower;

        driveMotors[0].setPower(brPower);
        driveMotors[1].setPower(blPower);
        driveMotors[2].setPower(frPower);
        driveMotors[3].setPower(flPower);

        intakeMotors[0].setPower(-Math.pow(gamepad2.left_stick_y, 3));
        intakeMotors[1].setPower(-Math.pow(gamepad2.right_stick_y, 3));

//        telemetry.addData("Yeet my skeet jeet", a);
    }


//    public void forward(double distance) {
//        while (avg(encoderTicksToInches(driveMotors[0].getCurrentPosition() - lastRightEncoder, 1050), encoderTicksToInches(driveMotors[1].getCurrentPosition() - lastLeftEncoder, 1330)) < distance) {
//            setMotorPower((distance - avg(encoderTicksToInches(driveMotors[0].getCurrentPosition() - lastRightEncoder, 1050), encoderTicksToInches(driveMotors[1].getCurrentPosition() - lastLeftEncoder, 1330))) / distance);
//        }
//        setMotorPower(0);
//
//        lastRightEncoder = driveMotors[0].getCurrentPosition();
//        lastLeftEncoder = driveMotors[1].getCurrentPosition();
//    }
//
//    public void backward(double distance) {
//        while (avg(encoderTicksToInches(driveMotors[0].getCurrentPosition() - lastRightEncoder, 1050), encoderTicksToInches(driveMotors[1].getCurrentPosition() - lastLeftEncoder, 1330)) < -distance) {
//            setMotorPower((distance - Math.abs(avg(encoderTicksToInches(driveMotors[0].getCurrentPosition() - lastRightEncoder, 1050), encoderTicksToInches(driveMotors[1].getCurrentPosition() - lastLeftEncoder, 1330)))) / distance);
//        }
//        setMotorPower(0);
//
//        lastRightEncoder = driveMotors[0].getCurrentPosition();
//        lastLeftEncoder = driveMotors[1].getCurrentPosition();
//    }
//
//    public void right(double distance) {
//        while (encoderTicksToInches(driveMotors[2].getCurrentPosition() - lastMiddleEncoder, 1050) < distance) {
//            setMotorPowerSideways((distance - encoderTicksToInches(driveMotors[2].getCurrentPosition() - lastMiddleEncoder, 1050)) / distance);
//        }
//        setMotorPower(0);
//
//        lastMiddleEncoder = driveMotors[2].getCurrentPosition();
//    }
//
//    public void left(double distance) {
//        while (encoderTicksToInches(driveMotors[2].getCurrentPosition() - lastMiddleEncoder, 1050) < distance) {
//            setMotorPowerSideways((distance - encoderTicksToInches(driveMotors[2].getCurrentPosition() - lastMiddleEncoder, 1050)) / distance);
//        }
//        setMotorPower(0);
//
//        lastMiddleEncoder = driveMotors[2].getCurrentPosition();
//    }
//
//    public void turnLeft(double radians) {
//        double odomRadius = 5.3125;
//
//        while (encoderTicksToInches(driveMotors[2].getCurrentPosition() - lastMiddleEncoder, 1050) < radians * odomRadius) {
//            setMotorPowerRotate(((radians * odomRadius) - encoderTicksToInches(driveMotors[2].getCurrentPosition() - lastMiddleEncoder, 1050)) / (radians * odomRadius));
//        }
//        setMotorPower(0);
//
//        lastMiddleEncoder = driveMotors[2].getCurrentPosition();
//
//    }
//
//    public void turnRight(double radians) {
//        radians = -radians;
//        double odomRadius = 5.3125;
//
//        while (encoderTicksToInches(driveMotors[2].getCurrentPosition() - lastMiddleEncoder, 1050) < radians * odomRadius) {
//            setMotorPowerRotate(((radians * odomRadius) - encoderTicksToInches(driveMotors[2].getCurrentPosition() - lastMiddleEncoder, 1050)) / (radians * odomRadius));
//        }
//        setMotorPower(0);
//
//        lastMiddleEncoder = driveMotors[2].getCurrentPosition();
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
//        driveMotors[0].setPower(a);
//        driveMotors[0].setPower(a);
//        driveMotors[0].setPower(a);
//        driveMotors[0].setPower(a);
//    }
//
//    public void setMotorPowerSideways(double a) {
//        driveMotors[0].setPower(-a);
//        driveMotors[0].setPower(a);
//        driveMotors[0].setPower(a);
//        driveMotors[0].setPower(-a);
//    }
//
//    public void setMotorPowerRotate(double a) {
//        driveMotors[0].setPower(a);
//        driveMotors[0].setPower(a);
//        driveMotors[0].setPower(-a);
//        driveMotors[0].setPower(-a);
//    }
}
