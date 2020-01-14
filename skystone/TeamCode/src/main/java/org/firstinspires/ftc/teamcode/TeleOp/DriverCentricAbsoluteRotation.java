package org.firstinspires.ftc.teamcode.TeleOp;

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
@TeleOp(group = "Basic Drivetrain")
public class DriverCentricAbsoluteRotation extends OpMode {
    private SampleMecanumDriveREVOptimized drive;

    private double[] motorPowers = new double[]{0, 0, 0, 0};
    private double x, y, rotation, maxPower, theta, cos, sin, tempx, startingDirection;
    private Double absoluteRotation;
    private PIDFController absoluteRotationPIDController;

    private boolean reachedTargetAngle = true;

    @Override
    public void init() {
        try {
            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");
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
    }


    @Override
    public void loop() {

        tempx = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        theta = Math.toRadians(90) - startingDirection - drive.getRawExternalHeading();
        cos = Math.cos(theta);
        sin = Math.sin(theta);

        absoluteRotation = getDPadAngle((gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0), (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
        if (absoluteRotation != null) {
            absoluteRotationPIDController.setTargetPosition(absoluteRotation);
        }

        rotation = Math.pow(gamepad1.right_stick_x, 3);
        if (rotation == 0) {
            rotation = absoluteRotationPIDController.update(drive.getRawExternalHeading());
        }
        x = tempx * cos - y * sin;
        y = tempx * sin + y * cos;
        x = x * Math.abs(x);
        y = y * Math.abs(y);


        motorPowers = new double[]{y + x + rotation, y - x + rotation, y + x - rotation, y - x - rotation};

        if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {
            maxPower = GetMaxAbsMotorPower();
            drive.setMotorPowers(motorPowers[0] / maxPower, motorPowers[1] / maxPower, motorPowers[2] / maxPower, motorPowers[3] / maxPower);
        } else {
            drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
        }
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
