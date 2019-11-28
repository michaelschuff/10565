package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


//TODO: This doesnt work
@TeleOp(group = "Basic Drivetrain")
public class DriverCentricAbsoluteRotation extends OpMode {
    SampleMecanumDriveREVOptimized drive;
    BNO055IMU imu;
    double[] motorPowers = new double[]{0, 0, 0, 0};
    double x, y, rotation, maxPower, theta, magnitude, targetAngle, previousTargetAngle, startAngle;

    @Override
    public void init() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        targetAngle = imu.getAngularOrientation().firstAngle;
        previousTargetAngle = targetAngle;
        startAngle = targetAngle;
    }


    @Override
    public void loop() {
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;

        theta = Math.atan2(y, x);
        magnitude = Math.sqrt(y * y + x * x);

        theta -= imu.getAngularOrientation().firstAngle;

        y = magnitude * Math.sin(theta);
        x = magnitude * Math.cos(theta);

        if (!(gamepad1.right_stick_y == 0 && gamepad1.right_stick_x == 0)) {
            targetAngle = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_y);
            if (Math.abs(previousTargetAngle - targetAngle) > .1) {
                previousTargetAngle = targetAngle;
                startAngle = imu.getAngularOrientation().firstAngle;
            }

        }


        x = x * Math.abs(x);
        y = y * Math.abs(y);



//            if (Math.abs(imu.getAngularOrientation().firstAngle - targetAngle) < 2 * Math.PI) {
//                rotation = 1;
//            } else {
//                rotation = -1;
//            }

        if (Math.abs(imu.getAngularOrientation().firstAngle - targetAngle) < .1) {
            rotation = 0;
            targetAngle = imu.getAngularOrientation().firstAngle;
        } else {
            rotation = (imu.getAngularOrientation().firstAngle - targetAngle) / startAngle;
        }

        motorPowers = new double[]{y + x + rotation, y - x + rotation, y + x - rotation, y - x - rotation};

        if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {
            maxPower = GetMaxAbsMotorPower();
            drive.setMotorPowers(motorPowers[0] / maxPower, motorPowers[1] / maxPower, motorPowers[2] / maxPower, motorPowers[3] / maxPower);
        } else {
            drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
        }
    }

    private double GetMaxAbsMotorPower() {
        return Math.max(Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1])), Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));
    }
}
