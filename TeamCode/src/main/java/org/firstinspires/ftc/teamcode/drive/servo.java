package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@Config
@TeleOp
public class servo extends OpMode {

    public static double b = 0.01;
    public static double a = .5;
    public static double x = .99;
    public static double bo = 0.01;
    public static double ao = .5;
    public static double xo = .99;
    ServoImplEx servo;
    ServoImplEx servo2;

    // AnalogInput sEncoder;
    AnalogInput sEncoder;
    //AnalogInput sEncoder2;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = (ServoImplEx) hardwareMap.get(Servo.class, "rArm");
        servo2 = (ServoImplEx) hardwareMap.get(Servo.class, "lArm");
        servo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        servo2.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder = hardwareMap.get(AnalogInput.class, "laEncoder");
        //sEncoder2 = hardwareMap.get(AnalogInput.class, "sEnxcoder2");


    }

    @Override
    public void loop() {

        double pos = sEncoder.getVoltage() / 3.3 * 360;
        //double pos2 = sEncoder2.get   Voltage() / 3.3 * 360;


        if (gamepad1.a) {
            servo.setPosition(a);
            servo2.setPosition(ao);
        }
        if (gamepad1.b) {
            servo.setPosition(b);
            servo2.setPosition(bo);
        }
        if (gamepad1.x) {
            servo.setPosition(x);
            servo2.setPosition(xo);
        }
        telemetry.addData("Run time",getRuntime());
        telemetry.addData("pos1", pos);
        // telemetry.addData("pos2", pos2);
        telemetry.addData("rPos1", servo.getPosition());
        // telemetry.addData("rPos2", servo2.getPosition());
        telemetry.addData("test", 2);
        telemetry.update();
    }


}