package org.firstinspires.ftc.teamcode.Tests.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;

@Config
@TeleOp(group = "HardwareTests")
public class MotorTickTest extends OpMode {

    private ExpansionHubMotor motor;
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    public static String motorName = "testMotor";

    @Override
    public void init() {
        motor = hardwareMap.get(ExpansionHubMotor.class, motorName);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
        telemetry.addData("Ticks per rev", motor.getMotorType().getTicksPerRev());
        telemetry.addData("Tested ticks", motor.getCurrentPosition());
    }
}
