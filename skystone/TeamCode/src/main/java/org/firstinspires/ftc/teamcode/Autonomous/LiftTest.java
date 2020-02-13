package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

@Config
@Autonomous
public class LiftTest extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private PIDFController liftController;
    public static PIDCoefficients liftCoeffs = new PIDCoefficients(0, 0, 0);
    public static double kV = 0;


    @Override
    public void runOpMode() {
        if (isStopRequested()) return;
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setClawGrabbing(false);
        liftController = new PIDFController(liftCoeffs, kV);
        liftController.setInputBounds(0, 4000);
        liftController.setOutputBounds(-1, 1);
        if (isStopRequested()) return;

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(drive.trajectoryBuilder(new DriveConstraints(10, 10, 10, Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)))
                .forward(10)
                .addMarker(() -> {
                    liftController.setTargetPosition(500);
                    while (true) {
                        double pow = liftController.update(drive.getLiftPos(), drive.getLiftVel());
                        if (pow < 0.05 && pow > 0.05) {
                            break;
                        }
                    }
                    return Unit.INSTANCE;
                })
                .back(10)
                .build()
        );
    }
}
