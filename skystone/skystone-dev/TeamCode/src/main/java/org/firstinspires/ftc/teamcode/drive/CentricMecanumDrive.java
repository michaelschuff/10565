package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

@TeleOp(name = "Basic Mecanum TeleOp", group = "TeleOp")
public class CentricMecanumDrive extends OpMode {

//    SampleMecanumDriveREVOptimized robot = new SampleMecanumDriveREVOptimized(hardwareMap);

    double xVelocity, yVelocity, angle, magnitude;
    double flPower, frPower, blPower, brPower, rotation, maxPower;
//
//    double armPower, intakePower, liftPower;
//
//    boolean aButtonDown = false, isClawGrabbing = false, yPressed = false;

    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor br = null;
    public DcMotor bl = null;

    HardwareMap hwMap = null;

    @Override
    public void init() {
//        robot.init(hardwareMap);
        hwMap = hardwareMap;

        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        maxPower = 0.7;

        xVelocity = Math.pow(gamepad1.left_stick_x, 3);
        yVelocity = Math.pow(gamepad1.left_stick_y, 3);

        if (xVelocity > 1) {
            xVelocity = 1;
        } else if (xVelocity < -1) {
            xVelocity = -1;
        }
        if (yVelocity > 1) {
            yVelocity = 1;
        } else if (yVelocity < -1) {
            yVelocity = -1;
        }
//        telemetry.addData("servo: ", robot.claw.getPosition());
//        if (gamepad1.y && yPressed == false) {
//            yPressed = true;
//            robot.ReleaseIntake.setPosition(.4);
//            sleep(4800);
//            robot.ReleaseIntake.setPosition(.5);
//        }
//
//        if (gamepad2.a) {
//            if (!aButtonDown) {
//                if (isClawGrabbing) {
//                    robot.claw.setPosition(.6);
//                } else {
//                    robot.claw.setPosition(.7);
//                }
//                isClawGrabbing = !isClawGrabbing;
//            }
//            aButtonDown = true;
//        } else {
//            aButtonDown = false;
//        }
//
//        liftPower = -Math.pow(gamepad2.right_stick_y, 3);
//
//        intakePower = -Math.pow(gamepad2.left_stick_y, 3);


        rotation = -Math.pow(gamepad1.right_stick_x, 3);


//        armPower = Math.pow(gamepad2.left_trigger, 3) - Math.pow(gamepad2.right_trigger, 3);




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

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

//        robot.arm.setPower(armPower);
//        robot.lift.setPower(liftPower);
//
//        robot.lIntake.setPower(intakePower);
//        robot.rIntake.setPower(intakePower);

//        telemetry.addData("Claw activated, isClawReleased = ", isClawGrabbing);
        telemetry.addData("front left/right power:", "%.2f %.2f", flPower, frPower);
        telemetry.addData("back left/right power:", "%.2f %.2f", blPower, brPower);
//        telemetry.addData("heading:", "%.2f", angle * 180 / Math.PI);
//        telemetry.addData("max power", "%.2f", maxPower);
//        telemetry.addData("arm power","%.2f", armPower);
//        telemetry.addData("arm power","%.2f", intakePower);
        telemetry.update();
    }
}
