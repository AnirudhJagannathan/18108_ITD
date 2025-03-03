package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;
import org.firstinspires.ftc.teamcode.teleopsubs.Slides;
import org.firstinspires.ftc.teamcode.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.util.wrappers.WServo;

import java.util.HashMap;
import java.util.List;

@Config
public class RobotHardware {
        public DcMotorEx slideLeft;
        public DcMotorEx slideRight;
        public DcMotorEx intake;
        public DcMotorEx hSlides;

        public WActuatorGroup slideLeftActuator;
        public WActuatorGroup slideRightActuator;

        public WActuatorGroup hSlideActuactor;
        public WActuatorGroup intakeActuactor;

        public WEncoder slideLeftEnc;
        public WEncoder slideRightEnc;
        public WEncoder hSlideEnc;


        public WServo arm1;
        public Servo arm2;
        public Servo intake1;
        public Servo intake2;

        public Servo hypLeft;
        public Servo hypRight;
        public Servo clawServo;
        public Servo swerve;

        public Arm arm;
        public Claw claw;
        public Intake spintake;
        public Slides slides;
        public SensorColor color;

        public NormalizedColorSensor colorSensor;

        public List<LynxModule> modules;
        public LynxModule CONTROL_HUB;


        public boolean encoderCheck = true;

        public MotorEx testMotor;


        private static RobotHardware instance = null;
        private boolean enabled;

        private int alliance;
        private HardwareMap hardwareMap;

        public HashMap<Sensors.SensorType, Object> values;

        public static RobotHardware getInstance() {
            if (instance == null) {
                instance = new RobotHardware();
            }
            instance.enabled = true;
            return instance;
        }

        public void init(final HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            this.values = new HashMap<>();

            values.put(Sensors.SensorType.SLIDE_LEFT_ENC, 0);
            values.put(Sensors.SensorType.SLIDE_RIGHT_ENC, 0);
            values.put(Sensors.SensorType.H_SLIDE_ENC, 0);

            slideLeft = hardwareMap.get(DcMotorEx.class, "VSlidesA");
            slideRight = hardwareMap.get(DcMotorEx.class, "VSlidesB");
            intake = hardwareMap.get(DcMotorEx.class, "spintake");
            hSlides = hardwareMap.get(DcMotorEx.class, "hSlides");

            slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

            arm1 = new WServo(hardwareMap.get(Servo.class, "arm"));
            arm2 = new WServo(hardwareMap.get(Servo.class, "arm2"));

            intake1 = new WServo(hardwareMap.get(Servo.class, "intake1"));
            intake2 = new WServo(hardwareMap.get(Servo.class, "intake2"));

            clawServo = new WServo(hardwareMap.get(Servo.class, "claw"));
            swerve = new WServo(hardwareMap.get(Servo.class, "swerve"));

            hypLeft = new WServo(hardwareMap.get(Servo.class, "hypLeft"));
            hypRight = new WServo(hardwareMap.get(Servo.class, "hypRight"));

            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

            testMotor = new MotorEx(hardwareMap, "VSlidesA");
            if (testMotor.encoder.getPosition() == 0)
                encoderCheck = false;

            slideLeftEnc = new WEncoder(new MotorEx(hardwareMap, "VSlidesA").encoder);
            slideRightEnc = new WEncoder(new MotorEx(hardwareMap, "VSlidesB").encoder);
            hSlideEnc = new WEncoder(new MotorEx(hardwareMap, "hSlides").encoder);

            this.slideLeftActuator = new WActuatorGroup(
                    () -> intSubscriber(Sensors.SensorType.SLIDE_LEFT_ENC), slideLeft)
                    .setPIDController(new PIDController(0.008, 0.0, 0.0004))
                    .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, 0.0)
//                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000))
                    .setErrorTolerance(50);

            this.slideRightActuator = new WActuatorGroup(
                    () -> intSubscriber(Sensors.SensorType.SLIDE_RIGHT_ENC), slideRight)
                    .setPIDController(new PIDController(0.008, 0.0, 0.0004))
                    .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, 0.0)
//                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000))
                    .setErrorTolerance(50);

            this.hSlideActuactor = new WActuatorGroup(
                    () -> intSubscriber(Sensors.SensorType.H_SLIDE_ENC), slideRight)
                    .setPIDController(new PIDController(0.008, 0.0, 0.0004))
                    .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, 0.0)
//                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000))
                    .setErrorTolerance(50);

            modules = hardwareMap.getAll(LynxModule.class);

            for (LynxModule m : modules) {
                m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber()))
                    CONTROL_HUB = m;
            }


            arm = new Arm(hardwareMap);
            claw = new Claw(hardwareMap);
            spintake = new Intake(hardwareMap);
            slides = new Slides(hardwareMap);
            color = new SensorColor(alliance);

            slides.reset();
        }

    public int intSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public void read() {
        values.put(Sensors.SensorType.SLIDE_LEFT_ENC, slideLeftEnc.getPosition());
        values.put(Sensors.SensorType.SLIDE_RIGHT_ENC, slideRightEnc.getPosition());
        values.put(Sensors.SensorType.H_SLIDE_ENC, hSlideEnc.getPosition());

        color.read();
    }

    public void write() {
        slides.write();
        arm.write();
        color.write();
    }

    public void periodic() {
         slides.periodic();
         arm.periodic();
         claw.periodic();
         color.periodic();
    }

    public void clearBulkCache() {
        CONTROL_HUB.clearBulkCache();
    }

    public void setAlliance(int alliance) {
         this.alliance = alliance;
    }

    public void setColorSensor(SensorColor color) {
         this.color = color;
    }
}
