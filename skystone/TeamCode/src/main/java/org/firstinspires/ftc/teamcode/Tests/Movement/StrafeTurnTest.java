package org.firstinspires.ftc.teamcode.Tests.Movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous(group="MovementTests")
public class StrafeTurnTest extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

    @Override
    public void runOpMode() {
        if (isStopRequested()) return;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        LineSegment line = new LineSegment(
                new Vector2d(0, 0),
                new Vector2d(72, 72)
        );
        LinearInterpolator interp = new LinearInterpolator(
                Math.toRadians(0), Math.toRadians(180)
        );
        PathSegment segment = new PathSegment(line, interp);
        Path path = new Path(segment);

        DriveConstraints constraints = new DriveConstraints(20, 40, 80, 1, 2, 4);
        //Trajectory traj = TrajectoryGenerator::generateSimpleTrajectory(path, constraints);

        waitForStart();

        //drive.followTrajectorySync(traj);
    }
}
