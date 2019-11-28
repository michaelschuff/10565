package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_SkyStone;

@Autonomous(group = "Tests")
public class SkystoneIDSplineTest extends LinearOpMode {
    SampleMecanumDriveREVOptimized drive;
    VuforiaLib_SkyStone camera;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, 63, 0));

        camera = new VuforiaLib_SkyStone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();



        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(-36, 39))
                .build();

        if (!isStarted()) {
            waitForStart();
        }


        drive.followTrajectorySync(trajectory);



        float skystonePosition = 0;

//        camera.loop(true);
//        try {
//            skystonePosition = camera.getFieldPosition().get(1));
//        } catch (Exception e) {}

        if (Math.abs(skystonePosition) < 2) {//if center stone
            trajectory = drive.trajectoryBuilder()
                    .splineTo(new Vector2d(-36, 39))
                    .build();
        } else if (skystonePosition > 2) {//if left stone

        } else if (skystonePosition < -2) {//if right stone

        }


    }

    void GetSkystonePosition() {


    }

    float mmToInches(float mmLength) {
        return mmLength / 25.4f;
    }
}
