package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.TaskThread;
import org.firstinspires.ftc.teamcode.util.ThreadLinearOpMode;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

import java.util.ArrayList;

@Config
@Autonomous(group = "Tests")
public class AutoDevelopment extends ThreadLinearOpMode {
    private SampleMecanumDriveREVOptimized drive;
    private VuforiaLib_Skystone camera;
    private VectorF VuMarkPosition = null;//in mm
    private ArrayList<Trajectory> commands;
    private ArrayList<Integer> waitIndicies;
    private ArrayList<Integer> turnIndicies;
    private ArrayList<Integer> waitTimes;
    private ArrayList<Double> turnValues;
    private String[] stringCommands;

    public static String trajectories = "forward 10";
    public static Pose2d startingPosition = new Pose2d(0, 0, 0);


    @Override
    public void runMainOpMode() {
        commands = new ArrayList<Trajectory>();
        waitIndicies = new ArrayList<Integer>();
        turnIndicies = new ArrayList<Integer>();
        waitTimes = new ArrayList<Integer>();
        turnValues = new ArrayList<Double>();
//        camera = new VuforiaLib_Skystone();
//        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
//        camera.start();
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
//                camera.loop(true);
//                try {
//                    VuMarkPosition = camera.getFieldPosition();
//                } catch (Exception e) {
//
//                }
            }
        }));

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
//        drive.setPoseEstimate(startingPosition);
        /*  splineTo x y h
            strafeTo x y
            strafeRight y
            forward x
            strafeLeft y
            backward x
            reverse
            setreverse a
            wait n
        */
        stringCommands = trajectories.split(" ");
        int i = 0, trajectoryCount = 0;
        while (i < stringCommands.length) {
            switch(stringCommands[i]){
                case "splineTo":
                    commands.add(drive.trajectoryBuilder()
                            .splineTo(new Pose2d(Double.parseDouble(stringCommands[i + 1]), Double.parseDouble(stringCommands[i + 2]), Math.toRadians(Double.parseDouble(stringCommands[i + 3]))))
                            .build()
                    );
                    trajectoryCount++;
                    break;
                case "strafeTo":
                    commands.add(drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(Double.parseDouble(stringCommands[i + 1]), Double.parseDouble(stringCommands[i + 2])))
                            .build()
                    );
                    trajectoryCount++;
                    break;
                case "strafeRight":
                    commands.add(drive.trajectoryBuilder()
                            .strafeRight(Double.parseDouble(stringCommands[i + 1]))
                            .build()
                    );
                    trajectoryCount++;
                    break;
                case "forward":
                    commands.add(drive.trajectoryBuilder()
                            .forward(Double.parseDouble(stringCommands[i + 1]))
                            .build()
                    );
                    trajectoryCount++;
                    break;
                case "strafeLeft":
                    commands.add(drive.trajectoryBuilder()
                            .strafeLeft(Double.parseDouble(stringCommands[i + 1]))
                            .build()
                    );
                    trajectoryCount++;
                    break;
                case "backward":
                    commands.add(drive.trajectoryBuilder()
                            .back(Double.parseDouble(stringCommands[i + 1]))
                            .build()
                    );
                    trajectoryCount++;
                    break;
                case "turn":
                    turnValues.add(Double.parseDouble(stringCommands[i + 1]));
                    turnIndicies.add(trajectoryCount);
                    break;
                case "wait":
                    waitTimes.add(Integer.parseInt(stringCommands[i + 1]));
                    waitIndicies.add(trajectoryCount);
                    break;
                case "reverse":
                    i++;
                    trajectoryCount++;
                    switch(stringCommands[i]) {
                        case "splineTo":
                            commands.add(drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(Double.parseDouble(stringCommands[i]), Double.parseDouble(stringCommands[i + 1]), Double.parseDouble(stringCommands[i + 2])))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "strafeTo":
                            commands.add(drive.trajectoryBuilder()
                                    .reverse()
                                    .strafeTo(new Vector2d(Double.parseDouble(stringCommands[i]), Double.parseDouble(stringCommands[i + 1])))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "strafeRight":
                            commands.add(drive.trajectoryBuilder()
                                    .reverse()
                                    .strafeRight(Double.parseDouble(stringCommands[i]))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "forward":
                            commands.add(drive.trajectoryBuilder()
                                    .reverse()
                                    .forward(Double.parseDouble(stringCommands[i]))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "strafeLeft":
                            commands.add(drive.trajectoryBuilder()
                                    .reverse()
                                    .strafeLeft(Double.parseDouble(stringCommands[i]))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "backward":
                            commands.add(drive.trajectoryBuilder()
                                    .reverse()
                                    .back(Double.parseDouble(stringCommands[i]))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "turn":
                            turnValues.add(Double.parseDouble(stringCommands[i + 1]));
                            turnIndicies.add(trajectoryCount);
                            break;
                        case "wait":
                            waitTimes.add(Integer.parseInt(stringCommands[i + 1]));
                            waitIndicies.add(trajectoryCount);
                            break;
                        default:
                            break;
                    }
                    break;
                case "setReversed":
                    i++;
                    trajectoryCount++;
                    switch(stringCommands[i + 2]) {
                        case "splineTo":
                            commands.add(drive.trajectoryBuilder()
                                    .setReversed(Boolean.parseBoolean(stringCommands[i + 1]))
                                    .splineTo(new Pose2d(Double.parseDouble(stringCommands[i + 1]), Double.parseDouble(stringCommands[i + 2]), Double.parseDouble(stringCommands[i + 3])))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "strafeTo":
                            commands.add(drive.trajectoryBuilder()
                                    .setReversed(Boolean.parseBoolean(stringCommands[i + 1]))
                                    .strafeTo(new Vector2d(Double.parseDouble(stringCommands[i + 1]), Double.parseDouble(stringCommands[i + 2])))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "strafeRight":
                            commands.add(drive.trajectoryBuilder()
                                    .setReversed(Boolean.parseBoolean(stringCommands[i + 1]))
                                    .strafeRight(Double.parseDouble(stringCommands[i + 1]))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "forward":
                            commands.add(drive.trajectoryBuilder()
                                    .setReversed(Boolean.parseBoolean(stringCommands[i + 1]))
                                    .forward(Double.parseDouble(stringCommands[i + 1]))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "strafeLeft":
                            commands.add(drive.trajectoryBuilder()
                                    .setReversed(Boolean.parseBoolean(stringCommands[i + 1]))
                                    .strafeLeft(Double.parseDouble(stringCommands[i + 1]))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "backward":
                            commands.add(drive.trajectoryBuilder()
                                    .setReversed(Boolean.parseBoolean(stringCommands[i + 1]))
                                    .back(Double.parseDouble(stringCommands[i + 1]))
                                    .build()
                            );
                            trajectoryCount++;
                            break;
                        case "turn":
                            turnValues.add(Double.parseDouble(stringCommands[i + 1]));
                            turnIndicies.add(trajectoryCount);
                            break;
                        case "wait":
                            waitTimes.add(Integer.parseInt(stringCommands[i + 1]));
                            waitIndicies.add(trajectoryCount);
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            i++;
        }

        if (isStopRequested()) return;

        waitForStart();
        int t = 0, w = 0;
        for (int j = 0; j < commands.size(); j++) {
            if (t < turnIndicies.size()) {
                if (j == turnIndicies.get(t) - 1) {
                    drive.turnSync(turnValues.get(t));
                    t++;
                }
            }
            if (w < waitIndicies.size()) {
                if (j == waitIndicies.get(w) - 1) {
                    sleep(waitTimes.get(w));
                    w++;
                }
            }
            drive.followTrajectorySync(commands.get(j));
            if (isStopRequested()) return;
        }
    }
}
