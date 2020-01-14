package org.firstinspires.ftc.teamcode.Tests.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous(group="HardwareTests")
public class MecanumWheelAngleTest extends LinearOpMode {

    public static double m1=1, m2=0, m3=1, m4=0;
    public static int time=1500;
    private SampleMecanumDriveREVOptimized drive;

    DcMotor leftEncoder, frontEncoder;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        leftEncoder = hardwareMap.dcMotor.get("lIntake");
        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");

        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setMotorPowers(m1, m2, m3, m4);

        sleep(time);

        drive.setMotorPowers(0, 0, 0, 0);

        while(!isStopRequested()) {
            telemetry.addData("x", leftEncoder.getCurrentPosition());
            telemetry.addData("y", frontEncoder.getCurrentPosition());
            telemetry.addData("h", -drive.getRawExternalHeading());
            telemetry.update();
        }



    }
}
