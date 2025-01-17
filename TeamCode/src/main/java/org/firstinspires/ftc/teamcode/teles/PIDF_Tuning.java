package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;

@Config
@TeleOp(name = "PIDF_ARM_LOOP", group = "drive")
public class PIDF_Tuning extends OpMode {

    public PIDController controller, slidecontroller;
    public static double p=0.0075,i=0,d=0.00003;
    public static double sp = 0.02, si=0, sd=0.0003, sf = 0.00005;

    public static double ticksPerRotation = 29.444444;
    public static double angleOffset = 54.714;
    EeshMechanism mechanism;

    public static int target = 0;
    public static int starget = 0;
    TouchSensor dontBreakIntake, dontBreakMotor;

    public void init() {
        mechanism = new EeshMechanism(hardwareMap);


        dontBreakIntake = hardwareMap.touchSensor.get("dontBreakIntake");
        dontBreakMotor =  hardwareMap.touchSensor.get("dontBreakMotor");

        controller = new PIDController(p, i, d);
        slidecontroller = new PIDController(0,0,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }
    public void loop() {
        controller.setPID(p,i,d);
        mechanism.update();
        int pos = -mechanism.wormCurrent, copycatPos = -mechanism.copycatCurrent;
        double wormPow = controller.calculate(pos, target);
        boolean dontBreakIntakeLim = dontBreakIntake.isPressed() && wormPow < 0;
        boolean dontBreakMotorLim = dontBreakMotor.isPressed() && wormPow > 0;
        double copycatPow = controller.calculate(copycatPos, target);

        mechanism.worm.worm.setPower(wormPow);
        mechanism.worm.copycat.setPower(copycatPow);


        slidecontroller.setPID(sp, si, sd);
        int spos = mechanism.slideCurrent;
        double spow = slidecontroller.calculate(mechanism.slideCurrent, starget);
        double ff = (spos+1865) * sf * Math.sin(mechanism.worm.getAngle());
        mechanism.slide.setPower(spow+ff);
        telemetry.addData("power: ", wormPow);
        telemetry.addData("copycat power: ", copycatPow);
        //telemetry.addData("worm current: ", worm.getCurrent(CurrentUnit.MILLIAMPS));
        //telemetry.addData("copyact current: ", copycat.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("pos: ", pos);
        telemetry.addData("copycat pos: ", copycatPos);
        telemetry.addData("target: ", target);
        telemetry.addData("sff: ", ff);
        telemetry.addData("spow: ", spow+ff);
        telemetry.addData("spos: ", spos);
        telemetry.addData("starget: ", starget);
        telemetry.addData("angle: ", pos/ticksPerRotation + angleOffset);
        telemetry.addData("error: ", target-pos);
        telemetry.addData("copycat error: ", target- copycatPos);
        telemetry.update();
    }
}