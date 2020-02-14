package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

//@Disabled
@Config
@TeleOp(group = "MovementTests")
public class IndividualDrivetrainMotors extends OpMode {
    SampleMecanumDriveREVOptimized drive;

    private FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive.setRunUsingEncoder(false);
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {
        double pow = gamepad1.right_trigger - gamepad1.left_trigger;
        drive.setMotorPowers(-Math.pow(gamepad1.left_stick_y, 3) + pow, -Math.pow(gamepad2.left_stick_y, 3) + pow, -Math.pow(gamepad2.right_stick_y, 3) + pow, -Math.pow(gamepad1.right_stick_y, 3) + pow);

        telemetry.addData("power", pow);
        telemetry.addData("front left", 2 * Math.PI * 2 * drive.fl.getVelocity() / drive.fl.getMotorType().getTicksPerRev());
        telemetry.addData("front right", 2 * Math.PI * 2 * drive.fr.getVelocity() / drive.fr.getMotorType().getTicksPerRev());
        telemetry.addData("back left", 2 * Math.PI * 2 * drive.bl.getVelocity() / drive.bl.getMotorType().getTicksPerRev());
        telemetry.addData("back right", 2 * Math.PI * 2 * drive.br.getVelocity() / drive.br.getMotorType().getTicksPerRev());
        telemetry.update();
    }
}