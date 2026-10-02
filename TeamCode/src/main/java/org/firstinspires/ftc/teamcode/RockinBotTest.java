package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RockinBotTest {
    private LinearOpMode o;
    private DcMotorEx shooter = null;
    private double shooterPower = 0;
    double shooterSpeed = 1.0;
    private PIDFCoefficients pidf = null;

    public RockinBotTest(LinearOpMode opMode) {
        o = opMode;
        o.telemetry.addData("This code was last updated", "8/18/2025, 2:45 pm"); // Todo: Update this date when the code is updated
        o.telemetry.update();
        initializeVar();
    }

    public void initializeVar() {
        shooter = o.hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // The velocity PIDF only takes effect when the shooter runs with its encoder
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.vv("Rockin' Robots", "Hardware Initialized. Starting PIDF: " + pidf);
    }

    public void adjustpValue(double delta) {
        pidf.p = Math.max(0.0, pidf.p + delta);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        RobotLog.vv("Rockin' Robots", "PIDF changed. New p value: " + pidf.p);
    }

    public void shooterPower(double speed) {
        RobotLog.vv("Rockin' Robots", "shooterPower(%.2f)", speed);
        shooterSpeed = speed;
        shooter.setPower(speed);
    }

    public void printDataOnScreen() {
        shooterPower = shooter.getCurrent(CurrentUnit.MILLIAMPS);
        o.telemetry.addData("Shooter Speed and Power", "%.2f", shooterSpeed, shooterPower);
        o.telemetry.addData("Velocity (ticks/s)", "%.2f", shooter.getVelocity());
        o.telemetry.addData("P value", "%.2f", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
        o.telemetry.update();
    }
}