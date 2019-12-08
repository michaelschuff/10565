package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

public abstract class ThreadLinearOpMode extends LinearOpMode {
    private List<TaskThread> threads = new ArrayList<>();

    public final void registerThread(TaskThread taskThread) {
        threads.add(taskThread);
    }

    public abstract void runMainOpMode();

    @Override
    public final void runOpMode() {
        runMainOpMode();
        waitForStart();
        for(TaskThread taskThread : threads) {
            taskThread.start();
        }
    }
}
