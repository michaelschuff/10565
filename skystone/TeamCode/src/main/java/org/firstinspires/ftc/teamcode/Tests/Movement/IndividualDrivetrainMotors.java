package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@TeleOp(group = "MovementTests")
public class IndividualDrivetrainMotors extends OpMode {
    SampleMecanumDriveREVOptimized drive;

    private FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {
        drive.setMotorPowers(-Math.pow(gamepad1.left_stick_y, 3), -Math.pow(gamepad2.left_stick_y, 3), -Math.pow(gamepad2.right_stick_y, 3), -Math.pow(gamepad1.right_stick_y, 3));

//        telemetry.addData("front left", drive.fl.getVelocity());
//        telemetry.addData("front right", drive.fr.getVelocity());
//        telemetry.addData("back left", drive.bl.getVelocity());
//        telemetry.addData("back right", drive.br.getVelocity());
//        telemetry.update();
    }
}