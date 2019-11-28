package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1350;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 38.0 / 30.0; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4; // in; offset of the lateral wheel

    private DcMotor leftEncoder, frontEncoder;
    private BNO055IMU imu;

    public TwoWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(FORWARD_OFFSET, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        leftEncoder = hardwareMap.dcMotor.get("leftIntake");
        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(-leftEncoder.getCurrentPosition()),
                encoderTicksToInches(-frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
