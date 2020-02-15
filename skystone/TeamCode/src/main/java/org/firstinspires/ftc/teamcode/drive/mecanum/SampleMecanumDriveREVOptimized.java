package org.firstinspires.ftc.teamcode.drive.mecanum;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
@Config
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx hub1;
    public ExpansionHubMotor fl, bl, br, fr, lIntake, rIntake;
    public ExpansionHubMotor fLift, bLift;
    private Servo rFoundation, lFoundation, rArm, lArm, claw;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    //idle servo positions
    public static double rFoundation1 = 0.5, lFoundation1 = 0.5, lArm1 = 0.37 , rArm1 = 0.63, claw1 = 0.75;

    //activated servo positions
    public static double rFoundation2 = 0.3, lFoundation2 = 0.7, lArm2 = 0.67, rArm2 = 0.33, claw2 = 0.98;

    //inactive servo positions
    public static double rFoundation0 = 1, lFoundation0 = 0;

    public boolean isFoundationGrabbed = false, isArmIn = true, isClawGrabbed = false, intaking = false;

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub1 = hardwareMap.get(ExpansionHubEx.class, "hub 1");

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
        fLift = hardwareMap.get(ExpansionHubMotor.class, "fLift");
        bLift = hardwareMap.get(ExpansionHubMotor.class, "bLift");
        motors = Arrays.asList(fl, bl, br, fr);

        rFoundation = hardwareMap.get(Servo.class, "rFoundation");
        lFoundation = hardwareMap.get(Servo.class, "lFoundation");
        lArm = hardwareMap.get(Servo.class, "lArm");
        rArm = hardwareMap.get(Servo.class, "rArm");
        claw = hardwareMap.get(Servo.class, "claw");

        bLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        bLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        rIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        bLift.setDirection(DcMotorSimple.Direction.REVERSE);


        setLocalizer(new TwoWheelLocalizer(hardwareMap, imu));
//        setLocalizer(new MecanumLocalizer(this, true));
//        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    public void setIntakePower(double leftPower, double rightPower) {
        lIntake.setPower(leftPower);
        rIntake.setPower(rightPower);
    }

    public void setLiftPos(int liftPos) {
        fLift.setTargetPosition(liftPos);
        bLift.setTargetPosition(liftPos);
    }

    public void setLiftPower(double liftPower) {
        bLift.setPower(liftPower);
        fLift.setPower(liftPower);
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

    public void releaseCapStone() {
        claw.setPosition(0);
        sleep(1000);
        setArmPos(0.45, 0.55);
        sleep(250);
        setArmPos(0.37, 0.63);
    }

    public void toggleFoundation() {
        isFoundationGrabbed = !isFoundationGrabbed;
        rFoundation.setPosition(isFoundationGrabbed ? rFoundation2 : rFoundation0);
        lFoundation.setPosition(isFoundationGrabbed ? lFoundation2 : lFoundation0);
    }

    public void toggleArm() {
        isArmIn = !isArmIn;
        lArm.setPosition(isArmIn ? lArm1 : lArm2);
        rArm.setPosition(isArmIn ? rArm1 : rArm2);
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

    public void setArmIn(boolean isIn) {
        isArmIn = isIn;
        lArm.setPosition(isIn ? lArm1: lArm2);
        rArm.setPosition(isIn ? rArm1: rArm2);
    }

    public void toggleClaw() {
        claw.setPosition(isClawGrabbed ? claw1 : claw2);
    }

    public boolean getIsArmIn() {
        updateV4BOut();
        return isArmIn;
    }

    public void resetLiftTicks() {
        bLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setClawGrabbing(boolean grabbed) {
        claw.setPosition(grabbed ? claw2 : claw1);
    }

    public void resetEveryThing() {
        setClawGrabbing(false);
        bLift.setPower(0.75);
        fLift.setPower(0.75);
        sleep(250);
        setArmPos(lArm1, rArm1);
        sleep(200);
        bLift.setPower(-0.75);
        fLift.setPower(-0.75);
    }

    public boolean CheckLiftPos() {
        int pos = bLift.getCurrentPosition();
        if (bLift.getVelocity() < 0.01 && pos < 100) {
            bLift.setPower(0);
            fLift.setPower(0);
            bLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        } else if (pos < 1000) {
            double pow = 0.075 + Math.sqrt(Math.abs(pos) / 1000.0);
            bLift.setPower(-pow);
            fLift.setPower(-pow);
        }
        return false;
    }

    public void toggleIntake() {
        intaking = !intaking;
        if (intaking) {
            lIntake.setPower(-1);
            rIntake.setPower(-1);
        } else {
            lIntake.setPower(0);
            rIntake.setPower(0);
        }
    }

    public void setRunUsingEncoder(boolean a) {
        if (a) {
            for (ExpansionHubMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else {
            for (ExpansionHubMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    private void updateV4BOut() {
        if (lArm.getPosition() < 0.5) {
            isArmIn = true;
        } else {
            isArmIn = false;
        }
    }

    public void updateClawGrabbed() {
        if (claw.getPosition() == claw1) {
            isClawGrabbed = false;
        } else {
            isClawGrabbed = true;
        }
        updateV4BOut();
    }

    public int getLiftPos() {
        return bLift.getCurrentPosition();
    }

    public double getLiftVel() {
        return bLift.getVelocity();
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
        RevBulkData bulkData = hub1.getBulkInputData();

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
        RevBulkData bulkData = hub1.getBulkInputData();

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
//        fl.setPower(adjustPower(v));
//        bl.setPower(adjustPower(v1));
//        br.setPower(adjustPower(v2));
//        fr.setPower(adjustPower(v3));
    }

    private double adjustPower(double power) {
        double minPower = 0.15;
        if (power > 0) {
            return power * (1 - minPower) + minPower;
        } else if (power < 0) {
            return power * (1 - minPower) - minPower;
        }
        return 0;
    }

    public TrajectoryBuilder trajectoryBuilder(DriveConstraints driveConstraints) {
        return new TrajectoryBuilder(getPoseEstimate(), driveConstraints);
    }

    @Override
    public double getRawExternalHeading() {
            return imu.getAngularOrientation().firstAngle;
    }

    public double getMaxMotorVelocity(){
        return fl.getVelocity() > fr.getVelocity() ? bl.getVelocity() > br.getVelocity() ? fl.getVelocity() > bl.getVelocity() ? fl.getVelocity() : bl.getVelocity() : fl.getVelocity() > br.getVelocity() ? fl.getVelocity() : bl.getVelocity() : bl.getVelocity() > br.getVelocity() ? fr.getVelocity() > bl.getVelocity() ? fr.getVelocity() : bl.getVelocity() : fr.getVelocity() > br.getVelocity() ? fr.getVelocity(): br.getVelocity();
    }

}
