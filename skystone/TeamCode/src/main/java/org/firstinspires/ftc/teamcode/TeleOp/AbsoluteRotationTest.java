package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.io.File;
import java.util.Scanner;

import static org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase.HEADING_PID;

@Config
@TeleOp(group="Tests")
public class AbsoluteRotationTest extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private PIDFController HeadingControl = new PIDFController(HEADING_PID); //Our PID controller variable. It takes PID Coefficients as parameters in its construction, which need to be tuned

    private double theta = 0, x, y, rotation, startingDirection = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        //Get starting Direction
//        try {
//            File file = new File(AppUtil.ROOT_FOLDER + "/StartingDirection.txt");
//            Scanner sc = new Scanner(file);
//            startingDirection = Float.parseFloat(sc.nextLine());
//            sc.close();
//        } catch (Exception e){
//
//        }

        //Set come constants for our PID Controller
        HeadingControl.setInputBounds(0.0, 2.0 * Math.PI); //This tells the Controller that 0 to 2pi loops, so if we give it 4pi, it will interpret it as 0
        HeadingControl.setOutputBounds(-1, 1); //Controller outputs power of rotation

        if (isStopRequested()) return;

        while(!isStopRequested()) {
            x = gamepad1.right_stick_x;
            y = -gamepad1.right_stick_y;

            if (x != 0 || y != 0) {
                theta = Math.atan2(y, x); //get angle of joystick
                HeadingControl.setTargetPosition(theta);
            }

            rotation = HeadingControl.update(drive.getRawExternalHeading()); //get the output from our PID Controller
            drive.setMotorPowers(-rotation, -rotation, rotation, rotation);
            drive.updatePoseEstimate();
            telemetry.addData("heading", Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.addData("target", Math.toDegrees(HeadingControl.getTargetPosition()));
            telemetry.addData("error", Math.toDegrees(drive.getRawExternalHeading()) - Math.toDegrees(HeadingControl.getTargetPosition()));
            telemetry.update();
        }

    }
}