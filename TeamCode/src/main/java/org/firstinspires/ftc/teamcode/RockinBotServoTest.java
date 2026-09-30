package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.RobotLog;

public class RockinBotServoTest {
    private final LinearOpMode o;
    private CRServo lumberjack = null;
    private double lumberjackPower = 0;

    public RockinBotServoTest(LinearOpMode opMode) {
        o = opMode;
        initializeVar();
    }

    public void initializeVar() {
        lumberjack = o.hardwareMap.get(CRServo.class, "lumberjack");
        RobotLog.vv("Rockin' Robots", "Lumberjack servo initialized");
    }

    public void start() {
        lumberjackPower = 0;
        lumberjack.setPower(lumberjackPower);
        RobotLog.vv("Rockin' Robots", "Lumberjack power: %.2f", lumberjackPower);
    }

    public void setLumberjackPower(double power) {
        if (power == lumberjackPower) {
            return;
        }

        lumberjackPower = power;
        lumberjack.setPower(lumberjackPower);
        RobotLog.vv("Rockin' Robots", "Lumberjack power: %.2f", lumberjackPower);
    }

    public void printDataOnScreen() {
        o.telemetry.addData("Lumberjack power", "%.2f", lumberjackPower);
        o.telemetry.update();
    }
}
