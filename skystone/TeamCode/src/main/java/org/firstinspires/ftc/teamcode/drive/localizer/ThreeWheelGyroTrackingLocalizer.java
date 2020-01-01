package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

@Config
public class ThreeWheelGyroTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1350;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 38.0 / 30.0; // output (wheel) speed / input (encoder) speed

    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    public BNO055IMU imu;

    public ThreeWheelGyroTrackingLocalizer(HardwareMap hardwareMap, BNO055IMU imu) {
        super(Arrays.asList(
                new Pose2d(0, 7.90625, 0), // left and right average
                new Pose2d(-7.28125, 0.0625, Math.toRadians(90)) // front
        ));

        this.imu = imu;

        leftEncoder = hardwareMap.dcMotor.get("lIntake");
        rightEncoder = hardwareMap.dcMotor.get("rIntake");
        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches((leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition() / 2)),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
