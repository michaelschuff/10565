package org.firstinspires.ftc.teamcode.Tests.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous(group="PIDTuningTests")
public class MotorPIDTuner extends LinearOpMode {
    public static PIDCoefficients motorPID = new PIDCoefficients(3, 0, 0);
    public static double targetPosition = 0, inputLowerBound = 0, inputUpperBound = 10, outputLowerBound = -1, outputUpperBound = 1, ticksConversion = 1;
    public static String MotorName = "testMotor";
    private double prevMotorPosition = 0, currentMotorPosition = 0;

    private DcMotor motor;

    private PIDFController pidController;

    @Override
    public void runOpMode() {
        pidController = new PIDFController(motorPID);
        pidController.setInputBounds(inputLowerBound, inputUpperBound);
        pidController.setOutputBounds(outputLowerBound, outputUpperBound);

        motor = hardwareMap.get(DcMotor.class, MotorName);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (isStopRequested()) { return; }

        waitForStart();

        while(!isStopRequested()) {
            if (pidController.getTargetPosition() != targetPosition) {
                pidController.setTargetPosition(targetPosition);
            }

            currentMotorPosition = motor.getCurrentPosition() * ticksConversion;
            pidController.update(currentMotorPosition, (currentMotorPosition - prevMotorPosition) / System.nanoTime());
            prevMotorPosition = currentMotorPosition;

            telemetry.addData("currentMotorPosition", currentMotorPosition);
            telemetry.update();
        }
    }
}
