package org.firstinspires.ftc.teamcode.drive.mecanum;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Config
public class justLocalizer extends SampleMecanumDriveBase {
    private ExpansionHubEx hub;
    private BNO055IMU imu;

    public justLocalizer(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub = hardwareMap.get(ExpansionHubEx.class, "hub 1");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);


        setLocalizer(new TwoWheelLocalizer(hardwareMap, imu));
//        setLocalizer(new ThreeWheelGyroTrackingLocalizer(hardwareMap, imu));
//        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }
    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        return new PIDCoefficients(0, 0, 0);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {

    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return new ArrayList<>();
    }

    @Override
    public List<Double> getWheelVelocities() {
        return new ArrayList<>();
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {

    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
