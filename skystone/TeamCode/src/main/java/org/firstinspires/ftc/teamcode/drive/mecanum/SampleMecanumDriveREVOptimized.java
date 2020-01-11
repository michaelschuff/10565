package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.ThreeWheelGyroTrackingLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
@Disabled
@Config
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx hub;
    private ExpansionHubMotor fl, bl, br, fr, lIntake, rIntake;
    public ExpansionHubMotor lift;
    private Servo rFoundation, lFoundation, lArm, rArm, claw;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    //idle servo positions
    private static final double rFoundation1 = 0.5, lFoundation1 = 0.5, lArm1 = 0.35, rArm1 = 0.65, claw1 = 0;

    //activated servo positions
    private static final double rFoundation2 = 0.325, lFoundation2 = 0.675, lArm2 = 0.77, rArm2 = 0.23, claw2 = 0.325;

    //inactive servo positions
    private static final double rFoundation0 = 1, lFoundation0 = 0;

    private boolean isFoundationGrabbed = false, isArmUp = false, isClawGrabbed = false;

    public int InchesToLiftTicks = 540;
    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub = hardwareMap.get(ExpansionHubEx.class, "hub 1");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        fl = hardwareMap.get(ExpansionHubMotor.class, "fl");
        bl = hardwareMap.get(ExpansionHubMotor.class, "bl");
        br = hardwareMap.get(ExpansionHubMotor.class, "br");
        fr = hardwareMap.get(ExpansionHubMotor.class, "fr");
        lIntake = hardwareMap.get(ExpansionHubMotor.class, "lIntake");
        rIntake = hardwareMap.get(ExpansionHubMotor.class, "rIntake");
        lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        motors = Arrays.asList(fl, bl, br, fr);

        rFoundation = hardwareMap.get(Servo.class, "rFoundation");
        lFoundation = hardwareMap.get(Servo.class, "lFoundation");
        lArm = hardwareMap.get(Servo.class, "lArm");
        rArm = hardwareMap.get(Servo.class, "rArm");
        claw = hardwareMap.get(Servo.class, "claw");

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        rIntake.setDirection(DcMotorSimple.Direction.REVERSE);
//        lift.setDirection(DcMotorSimple.Direction.REVERSE);


        setLocalizer(new TwoWheelLocalizer(hardwareMap, imu));
//        setLocalizer(new ThreeWheelGyroTrackingLocalizer(hardwareMap, imu));
//        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    public void setIntakePower(double leftPower, double rightPower) {
        lIntake.setPower(leftPower);
        rIntake.setPower(rightPower);
    }

    public void setLiftPos(int liftPos) {
        lift.setTargetPosition(liftPos);
    }

    public void setLiftPower(double liftPower) {
        lift.setPower(liftPower);
    }

    public void setFoundation(short n) {
        switch (n) {
            case 0:
                rFoundation.setPosition(rFoundation0);
                lFoundation.setPosition(lFoundation0);
                break;
            case 1:
                rFoundation.setPosition(rFoundation1);
                lFoundation.setPosition(lFoundation1);
                break;
            case 2:
                rFoundation.setPosition(rFoundation2);
                lFoundation.setPosition(lFoundation2);
                break;

        }
    }

    public void toggleFoundation() {
        isFoundationGrabbed = !isFoundationGrabbed;
        rFoundation.setPosition(isFoundationGrabbed ? rFoundation2 : rFoundation0);
        lFoundation.setPosition(isFoundationGrabbed ? lFoundation2 : lFoundation0);
    }

    public void toggleArm() {
        isArmUp = !isArmUp;
        lArm.setPosition(isArmUp ? lArm1 : lArm2);
        rArm.setPosition(isArmUp ? rArm1 : rArm2);
    }

    public void IncArm() {
        lArm.setPosition(lArm.getPosition() + .01);
        rArm.setPosition(rArm.getPosition() - .01);
    }

    public void DecArm() {
        lArm.setPosition(lArm.getPosition() - .01);
        rArm.setPosition(rArm.getPosition() + .01);
    }

    public void setArmPos(double val1, double val2) {
        lArm.setPosition(val1);
        rArm.setPosition(val2);
    }

    public void toggleClaw() {
        isClawGrabbed = !isClawGrabbed;
        claw.setPosition(isClawGrabbed ? claw1 : claw2);
    }

    public void setClawGrabbing(boolean grabbed) {
        claw.setPosition(grabbed ? claw2 : claw1);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = fl.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(motor)));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        fl.setPower(v);
        bl.setPower(v1);
        br.setPower(v2);
        fr.setPower(v3);
    }

    public TrajectoryBuilder trajectoryBuilder(DriveConstraints driveConstraints) {
        return new TrajectoryBuilder(getPoseEstimate(), driveConstraints);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

}
