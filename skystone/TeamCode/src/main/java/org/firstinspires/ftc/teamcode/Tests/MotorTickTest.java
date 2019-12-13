package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.revextensions2.ExpansionHubMotor;

@Config
@Autonomous(group = "Tests")
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
        telemetry.addData("ticks", motor.getCurrentPosition());
    }
}
