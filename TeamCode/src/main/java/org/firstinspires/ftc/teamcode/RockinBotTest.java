package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RockinBotTest {
    private LinearOpMode o;
    private DcMotorEx motor = null;
    private double motorPower = 0;
    double motorSpeed = 1.0;

    public RockinBotTest(LinearOpMode opMode) {
        o = opMode;
        o.telemetry.addData("This code was last updated", "8/18/2025, 2:45 pm"); // Todo: Update this date when the code is updated
        o.telemetry.update();
        initializeVar();
    }

    public void initializeVar() {
        motor = o.hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RobotLog.vv("Rockin' Robots", "Hardware Initialized");
    }

    public void motorPower(double speed) {
        RobotLog.vv("Rockin' Robots", "motorPower(%.2f)", speed);
        motorSpeed = speed;
        motor.setPower(speed);
    }

    public void printDataOnScreen() {
        motorPower = motor.getCurrent(CurrentUnit.MILLIAMPS);
        o.telemetry.addData("motor Speed and Power", "%.2f", motorSpeed, motorPower);
        o.telemetry.update();
    }
}