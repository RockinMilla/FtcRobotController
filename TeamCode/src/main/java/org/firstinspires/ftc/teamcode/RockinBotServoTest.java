package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

public class RockinBotServoTest {
    private final LinearOpMode o;
    private Servo lumberjack = null;
    private double lumberjackPosition;

    public RockinBotServoTest(LinearOpMode opMode) {
        o = opMode;
        initializeVar();
    }

    public void initializeVar() {
        lumberjack = o.hardwareMap.get(Servo.class, "lumberjack");
        lumberjackPosition = lumberjack.getPosition();
        RobotLog.vv("Rockin' Robots", "Lumberjack servo initialized at %.2f", lumberjackPosition);
    }

    public void start() {
        lumberjackPosition = lumberjack.getPosition();
        RobotLog.vv("Rockin' Robots", "Lumberjack position: %.2f", lumberjackPosition);
    }

    public void adjustLumberjackPosition(double adjustment) {
        double newPosition = Range.clip(lumberjackPosition + adjustment, 0.0, 1.0);
        if (newPosition == lumberjackPosition) {
            return;
        }

        lumberjackPosition = newPosition;
        lumberjack.setPosition(lumberjackPosition);
        RobotLog.vv("Rockin' Robots", "Lumberjack position: %.2f", lumberjackPosition);
    }

    public void holdLumberjackPosition() {
        lumberjack.setPosition(lumberjackPosition);
        RobotLog.vv("Rockin' Robots", "Lumberjack holding at %.2f", lumberjackPosition);
    }

    public void printDataOnScreen(boolean rightBumper, boolean leftBumper) {
        String state = rightBumper == leftBumper
                ? "Holding"
                : rightBumper ? "Moving up" : "Moving down";
        o.telemetry.addData("Servo type", "Standard positional");
        o.telemetry.addData("Lumberjack position", "%.2f", lumberjackPosition);
        o.telemetry.addData("State", state);
        o.telemetry.addData("Right bumper (up)", rightBumper);
        o.telemetry.addData("Left bumper (down)", leftBumper);
        o.telemetry.update();
    }
}
