package org.firstinspires.ftc.teamcode.Tests.Gimmick;


import android.content.Context;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Config
@TeleOp(group="Gimmick")
public class SoundTest extends LinearOpMode {


    private boolean dPadUp = false, dPadRight = false, dPadDown = false, dPadLeft = false, rBumper = false, lBumper = false, xButton = false, yButton = false, bButton = false, aButton = false, soundPlaying = false;

    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };

    @Override
    public void runOpMode() {
        Context myApp = hardwareMap.appContext;
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        int soundID = -1, soundIndex = 0;
        waitForStart();

        while (!isStopRequested()) {
//            if (gamepad1.right_bumper) {
//                rBumper = true;
//            } else  if (rBumper) {
//                rBumper = false;
//            }
//
//            if (gamepad1.left_bumper) {
//                lBumper = true;
//            } else  if (lBumper) {
//                lBumper = false;
//            }
//
//            if (gamepad1.dpad_up) {
//                dPadUp = true;
//            } else  if (dPadUp) {
//                dPadUp = false;
//            }
//
//            if (gamepad1.dpad_down) {
//                dPadDown = true;
//            } else  if (dPadDown) {
//                dPadDown = false;
//            }
//
//            if (gamepad1.dpad_left) {
//                dPadLeft = true;
//            } else  if (dPadLeft) {
//                dPadLeft = false;
//            }
//
//            if (gamepad1.dpad_right) {
//                dPadRight = true;
//            } else  if (dPadRight) {
//                dPadRight = false;
//            }
//
//            if (gamepad1.x) {
//                xButton = true;
//            } else  if (xButton) {
//                xButton = false;
//            }
//
//            if (gamepad1.a) {
//                aButton = true;
//            } else  if (aButton) {
//                aButton = false;
//            }
//
//            if (gamepad1.y) {
//                yButton = true;
//            } else  if (yButton) {
//                yButton = false;
//            }
//
//            if (gamepad1.b) {
//                bButton = true;
//            } else  if (bButton) {
//                bButton = false;
//            }
            soundIndex = -1;
            if (gamepad1.dpad_up) {
                soundIndex = 0;
            }
            if (gamepad1.dpad_right) {
                soundIndex = 1;
            }
            if (gamepad1.dpad_down) {
                soundIndex = 2;
            }
            if (gamepad1.dpad_left) {
                soundIndex = 3;
            }
            if (gamepad1.right_bumper) {
                soundIndex = 4;
            }
            if (gamepad1.left_bumper) {
                soundIndex = 5;
            }
            if (gamepad1.x) {
                soundIndex = 6;
            }
            if (gamepad1.y) {
                soundIndex = 7;
            }
            if (gamepad1.b) {
                soundIndex = 8;
            }
            if (gamepad1.a) {
                soundIndex = 9;
            }

            if (!soundPlaying) {

                // Determine Resource IDs for the sounds you want to play, and make sure it's valid.
                if ((soundID = myApp.getResources().getIdentifier(sounds[soundIndex], "raw", myApp.getPackageName())) != 0){

                    // Signal that the sound is now playing.
                    soundPlaying = true;

                    // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
                    SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                            new Runnable() {
                                public void run() {
                                    soundPlaying = false;
                                }} );
                }
            }
        }
    }
}
