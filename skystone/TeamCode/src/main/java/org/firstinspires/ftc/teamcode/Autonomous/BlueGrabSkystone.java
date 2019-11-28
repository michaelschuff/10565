package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

public class BlueGrabSkystone extends LinearOpMode {
    SampleMecanumDriveREVOptimized drive;
    VuforiaLib_Skystone camera;
    float skystonePosition = 0;
    double tileSize = 23.25;
    double meshSize = .75;

    @Override
    public void runOpMode() throws InterruptedException{
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-tileSize - 9,3 * tileSize - 9 - meshSize / 2,0));

        camera = new VuforiaLib_Skystone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();

        camera.loop(true);
        try {
            skystonePosition = camera.getFieldPosition().get(1);
        } catch (Exception e) {

        }
        Trajectory trajectory = drive.trajectoryBuilder().build();
        if (Math.abs(skystonePosition) < 4) {//Middle
            trajectory = drive.trajectoryBuilder()
                .strafeTo(new Vector2d( -17, -tileSize - 3 - 9))
                .build();
        } else if (skystonePosition > 4) {//Left
            //TODO: find a creative way to grab this
            trajectory = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d( -11, -tileSize - 3 - 9))
                    .build();
        } else if (skystonePosition < -4) {//Right
            trajectory = drive.trajectoryBuilder()
                    .strafeTo(new Vector2d( -25, -tileSize - 3 - 9))
                    .build();
        } else {

        }

        waitForStart();


        drive.followTrajectorySync(trajectory);

        trajectory = drive.trajectoryBuilder()
                .strafeLeft(14)
                .build();

        drive.followTrajectorySync(trajectory);

        drive.ToggleIntake();

        trajectory = drive.trajectoryBuilder()
                .forward(4)
                .build();

        //TODO: Check if block was intaken

        //enough for now






    }
}
