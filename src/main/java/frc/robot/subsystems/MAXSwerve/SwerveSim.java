package frc.robot.subsystems.MAXSwerve;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N11;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.NumericalJacobian;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import frc.robot.Constants;
import frc.robot.util.Util;

import static java.lang.Math.pow;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.copySign;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.log;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

import java.util.Arrays;
import java.util.function.Function;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSim {
    private final double dt = 0.02;

    private final double wr = DriveConstants.ModuleConstants.kWheelDiameterMeters / 2; // Wheel radius (meters)
    private final double mr = 60, mw = 0.1; // Robot, wheel mass (kg)
    private final double It = 0.01, Ir = 0.01, IR = 5; // Wheel turning, driving inertia, robot inertia
    private final double a = DriveConstants.kWheelBase / 2, b = DriveConstants.kTrackWidth / 2; // Distance from center to front, side (m)
    private final double k = 50; // Smoothing constant
    private double[] stuck = {1, 1, 1, 1}; // Defines how much a wheel can spin (0=no spin)
    private double kVD, kAD; // Motor drive stats
    private double kVT, kAT; // Motor turning stats

    private DCMotor driveMotor, turnMotor;
    private double driveG, turnG;

    private double wireResistance = 0.005; // ohms for 1m of 12awg wire

    private Vector<N22> state = new Vector<N22>(N22.instance);
    private Vector<N8> input = new Vector<N8>(N8.instance);

    private int max_depth = 0;
    private int iters = 0;
    private int calcs = 0;
    private int bat_iters = 0;

    public SwerveSim(DCMotor drivingMotor, DCMotor turningMotor, double drivingG, double turningG) {
        this.driveMotor = drivingMotor;
        this.turnMotor = turningMotor;

        this.driveG = drivingG;
        this.turnG = turningG;

        this.kVD = -drivingG * drivingG * drivingMotor.KtNMPerAmp / (drivingMotor.KvRadPerSecPerVolt * drivingMotor.rOhms);
        this.kVT = -turningG * turningG * turningMotor.KtNMPerAmp / (turningMotor.KvRadPerSecPerVolt * turningMotor.rOhms);

        this.kAD = drivingG * drivingMotor.KtNMPerAmp / drivingMotor.rOhms;
        this.kAT = turningG * turningMotor.KtNMPerAmp / turningMotor.rOhms;



        // System.out.println(kVD);
        // System.out.println(kVT);
        // System.out.println(kAD);
        // System.out.println(kAT);
        // System.exit(0);
    }

    public static double currentLimit(DCMotor motor, double angVel, double volts, double limit) {
        double I = motor.getCurrent(angVel, volts);
        if (Math.abs(I) > limit) {
            return motor.getVoltage(motor.getTorque(Math.copySign(limit, I)), angVel);
        } else {
            return volts;
        }
    }

    private double currentLimitDrive(int n, double v) {
        return currentLimit(driveMotor, getDriveAngularVelocity(n) * driveG, v, Constants.CurrentLimits.drive);
    }

    private double currentLimitTurn(int n, double v) {
        return currentLimit(turnMotor, getTurnAngularVelocity(n) * turnG, v, Constants.CurrentLimits.turning);
    }

    public void resetPosition(Pose2d pose) {
        state.set(0, 0, pose.getX());
        state.set(1, 0, pose.getY());
        state.set(2, 0, pose.getRotation().getRadians());
    }

    public double getX() {
        return state.get(0);
    }

    public double getY() {
        return state.get(1);
    }

    public double getAngle() {
        return state.get(2);
    }

    public double getAngularVelocity() {
        return state.get(13);
    }

    @AutoLogOutput
    public Rotation2d getRotation() {
        return new Rotation2d(state.get(2));
    }

    public Pose2d getPose() {
        return new Pose2d(getX(), getY(), getRotation());
    }

    public double getDriveAngularPosition(int n) {
        return state.get(3 + n);
    }

    public double getDriveAngularVelocity(int n) {
        return state.get(14 + n);
    }

    public double getTurnAngularPosition(int n) {
        return state.get(7 + n);
    }

    public double getTurnAngularVelocity(int n) {
        return state.get(18 + n);
    }

    public double getXVel() {
        return state.get(11);
    }

    public double getYVel() {
        return state.get(12);
    }

    public ChassisSpeeds getChassisSpeeds() {
        double vx = getXVel() * cos(getAngle()) + getYVel() * sin(getAngle());
        double vy = getXVel() * cos(getAngle() + PI / 2) + getYVel() * sin(getAngle() + PI / 2);
        double vel = Math.hypot(getXVel(), getYVel());
        return new ChassisSpeeds(vx, vy, vel);
    }

    public void setDriveVoltage(int n, double voltage) {
        input.set(n, 0, voltage);
    }

    public void setTurnVoltage(int n, double voltage) {
        input.set(4 + n, 0, voltage);
    }

    private double fix(double x) {
        // if (Double.isNaN(x)) {
        //     return 1;
        // } else if (x == Double.NEGATIVE_INFINITY) {
        //     return -9999999.0;
        // } else if (x == Double.POSITIVE_INFINITY) {
        //     return 9999999.0;
        if (Double.isFinite(x)) {
            return MathUtil.clamp(x, -9999999.0, 9999999.0);
        } else {
            return 0;
        }
    }

    private double deriv(Function<Double, Double> f, double x) {
        return (f.apply(x + 1e-5) - f.apply(x)) / 1e-5;
        // return 0;
    }

    private double smooth(double x) {
        return Math.min(Math.abs(x * k), 1);
        // return 1;
        
        // double kx = Math.abs(x * k);
        // if (kx == 0.0) {
        //     return 0;
        // } else if (kx < 1) {
        //     return 3 * kx * kx - 2 * kx * kx * kx;
        // } else {
        //     return 1;
        // }
    }

    private double cof(double x) {
        // return 1.5 - 0.4 * 2 / PI * Math.atan(40 * x);
        return 1.5;
    }

    private double rollingCof(double x) {
        return 6*(0.25 - 0.08 * 2 / PI * Math.atan(20 * Math.abs(x)));
        // return 1.25;
    }

    // Jacobian of the system
    // Current implementation uses numerical jacobian approximation
    private Matrix<N22, N22> jacobian(Matrix<N22, N1> state) {
        double q0 = state.get(0, 0), dq0 = state.get(11, 0);
        double q1 = state.get(1, 0), dq1 = state.get(12, 0);
        double q2 = state.get(2, 0), dq2 = state.get(13, 0);
        double q3 = state.get(3, 0), dq3 = state.get(14, 0);
        double q4 = state.get(4, 0), dq4 = state.get(15, 0);
        double q5 = state.get(5, 0), dq5 = state.get(16, 0);
        double q6 = state.get(6, 0), dq6 = state.get(17, 0);
        double q7 = state.get(7, 0), dq7 = state.get(18, 0);
        double q8 = state.get(8, 0), dq8 = state.get(19, 0);
        double q9 = state.get(9, 0), dq9 = state.get(20, 0);
        double q10 = state.get(10, 0), dq10 = state.get(21, 0);

        Matrix<N22, N22> result = new Matrix<N22, N22>(N22.instance, N22.instance);

        double x0 = -dt;
        double x1 = a*dq2;
        double x2 = sin(q10);
        double x3 = dq6*wr;
        double x4 = x1 - x2*x3;
        double x5 = cos(q2);
        double x6 = b*dq2;
        double x7 = cos(q10);
        double x8 = -x3*x7 + x6;
        double x9 = sin(q2);
        double x10 = x4*x5 + x8*x9;
        double x11 = x5*x8;
        double x12 = x4*x9;
        double x13 = dq0 + x11 - x12;
        double x14 = pow(x13, 2);
        double x15 = dq1 + x10;
        double x16 = pow(x15, 2);
        double x17 = x14 + x16;
        double x18 = sqrt(x17);
        double x19 = fix(1/x18);
        double x20 = cof(x18);
        double x21 = smooth(x18);
        double x22 = x20*x21;
        double x23 = x19*x22;
        double x24 = x10*x23;
        double x25 = sin(q7);
        double x26 = dq3*wr;
        double x27 = x1 - x25*x26;
        double x28 = cos(q7);
        double x29 = x26*x28 + x6;
        double x30 = x27*x5 - x29*x9;
        double x31 = x27*x9 + x29*x5;
        double x32 = -dq0 + x31;
        double x33 = -x32;
        double x34 = pow(x33, 2);
        double x35 = dq1 + x30;
        double x36 = pow(x35, 2);
        double x37 = x34 + x36;
        double x38 = sqrt(x37);
        double x39 = fix(1/x38);
        double x40 = cof(x38);
        double x41 = smooth(x38);
        double x42 = x40*x41;
        double x43 = x39*x42;
        double x44 = x30*x43;
        double x45 = sin(q8);
        double x46 = dq4*wr;
        double x47 = x1 + x45*x46;
        double x48 = cos(q8);
        double x49 = x46*x48 + x6;
        double x50 = x47*x5 + x49*x9;
        double x51 = x47*x9 - x49*x5;
        double x52 = dq0 + x51;
        double x53 = pow(x52, 2);
        double x54 = -dq1 + x50;
        double x55 = -x54;
        double x56 = pow(x55, 2);
        double x57 = x53 + x56;
        double x58 = sqrt(x57);
        double x59 = fix(1/x58);
        double x60 = cof(x58);
        double x61 = smooth(x58);
        double x62 = x60*x61;
        double x63 = x59*x62;
        double x64 = x50*x63;
        double x65 = sin(q9);
        double x66 = dq5*wr;
        double x67 = x1 + x65*x66;
        double x68 = x5*x67;
        double x69 = cos(q9);
        double x70 = x6 - x66*x69;
        double x71 = x70*x9;
        double x72 = x68 - x71;
        double x73 = x5*x70 + x67*x9;
        double x74 = dq0 + x73;
        double x75 = pow(x74, 2);
        double x76 = dq1 - x68 + x71;
        double x77 = pow(x76, 2);
        double x78 = x75 + x77;
        double x79 = sqrt(x78);
        double x80 = fix(1/x79);
        double x81 = cof(x79);
        double x82 = smooth(x79);
        double x83 = x81*x82;
        double x84 = x80*x83;
        double x85 = x72*x84;
        double x86 = x60*deriv(this::smooth, x58);
        double x87 = -x51;
        double x88 = -x50*x52 + x55*x87;
        double x89 = fix(1/x57);
        double x90 = x52*x89;
        double x91 = x88*x90;
        double x92 = x61*deriv(this::cof, x58);
        double x93 = -x88;
        double x94 = fix(pow(x57, -3.0/2.0));
        double x95 = x62*x94;
        double x96 = x52*x95;
        double x97 = x81*deriv(this::smooth, x79);
        double x98 = -x72;
        double x99 = x73*x76 - x74*x98;
        double x100 = -x99;
        double x101 = fix(1/x78);
        double x102 = x101*x74;
        double x103 = x100*x102;
        double x104 = x82*deriv(this::cof, x79);
        double x105 = fix(pow(x78, -3.0/2.0));
        double x106 = x105*x83;
        double x107 = x106*x74;
        double x108 = -x11 + x12;
        double x109 = x108*x15;
        double x110 = x10*x13 + x109;
        double x111 = x110*x13;
        double x112 = fix(1/x17);
        double x113 = x20*deriv(this::smooth, x18);
        double x114 = x112*x113;
        double x115 = x21*deriv(this::cof, x18);
        double x116 = x112*x115;
        double x117 = x30*x33 + x31*x35;
        double x118 = x117*x33;
        double x119 = fix(1/x37);
        double x120 = x40*deriv(this::smooth, x38);
        double x121 = x119*x120;
        double x122 = x41*deriv(this::cof, x38);
        double x123 = x119*x122;
        double x124 = fix(pow(x17, -3.0/2.0));
        double x125 = x124*x22;
        double x126 = fix(pow(x37, -3.0/2.0));
        double x127 = x126*x42;
        double x128 = 4*mw;
        double x129 = 9.8066499999999994*mr + 39.226599999999998*mw;
        double x130 = (1.0/4.0)*dt;
        double x131 = x129*x130;
        double x132 = x131/(mr + x128);
        double x133 = x25*x5 + x28*x9;
        double x134 = x133*x43;
        double x135 = x133*x33;
        double x136 = x25*x9 - x28*x5;
        double x137 = x136*x35;
        double x138 = x135 + x137;
        double x139 = x138*x33;
        double x140 = x121*x139;
        double x141 = x123*x139;
        double x142 = -x136;
        double x143 = -x35;
        double x144 = x133*x32;
        double x145 = x142*x143 - x144;
        double x146 = x127*x33;
        double x147 = x132*x26;
        double x148 = x45*x5 + x48*x9;
        double x149 = x148*x63;
        double x150 = x148*x52;
        double x151 = x45*x9 - x48*x5;
        double x152 = x151*x55;
        double x153 = x150 + x152;
        double x154 = x153*x90;
        double x155 = x154*x86;
        double x156 = x154*x92;
        double x157 = -x151;
        double x158 = -x52;
        double x159 = -x148*x158 + x157*x54;
        double x160 = x132*x46;
        double x161 = x5*x65 + x69*x9;
        double x162 = x161*x84;
        double x163 = x161*x74;
        double x164 = -x5*x69 + x65*x9;
        double x165 = x164*x76;
        double x166 = x163 + x165;
        double x167 = x102*x166;
        double x168 = x167*x97;
        double x169 = x104*x167;
        double x170 = -x164;
        double x171 = -x76;
        double x172 = x163 + x170*x171;
        double x173 = x132*x66;
        double x174 = x2*x5 + x7*x9;
        double x175 = x174*x23;
        double x176 = x13*x174;
        double x177 = x2*x9 - x5*x7;
        double x178 = x15*x177;
        double x179 = x176 + x178;
        double x180 = x13*x179;
        double x181 = x114*x180;
        double x182 = x116*x180;
        double x183 = -x177;
        double x184 = -x13;
        double x185 = x15*x183 + x174*x184;
        double x186 = x125*x13;
        double x187 = x185*x186;
        double x188 = x132*x3;
        double x189 = x53*x89;
        double x190 = x189*x86;
        double x191 = x189*x92;
        double x192 = x101*x75;
        double x193 = x192*x97;
        double x194 = x104*x192;
        double x195 = x112*x14;
        double x196 = x113*x195;
        double x197 = x115*x195;
        double x198 = x119*x34;
        double x199 = x120*x198;
        double x200 = x122*x198;
        double x201 = x23 + x43 + x63 + x84;
        double x202 = x55*x90;
        double x203 = x202*x86;
        double x204 = x202*x92;
        double x205 = x102*x76;
        double x206 = x205*x97;
        double x207 = x104*x205;
        double x208 = x13*x15;
        double x209 = x114*x208;
        double x210 = x116*x208;
        double x211 = x33*x35;
        double x212 = x121*x211;
        double x213 = x123*x211;
        double x214 = x55*x96;
        double x215 = x107*x76;
        double x216 = x15*x186;
        double x217 = x146*x35;
        double x218 = x132*(x203 + x204 + x206 + x207 + x209 + x210 + x212 + x213 - x214 - x215 - x216 - x217);
        double x219 = a*x9;
        double x220 = b*x5;
        double x221 = x219 - x220;
        double x222 = x221*x63;
        double x223 = x219 + x220;
        double x224 = x223*x84;
        double x225 = x221*x23;
        double x226 = x223*x43;
        double x227 = a*x5;
        double x228 = b*x9;
        double x229 = x227 + x228;
        double x230 = x221*x52 - x229*x55;
        double x231 = x230*x90;
        double x232 = x223*x74;
        double x233 = x227 - x228;
        double x234 = x232 - x233*x76;
        double x235 = x102*x234;
        double x236 = -x13*x221 + x15*x229;
        double x237 = x13*x236;
        double x238 = -x223*x33 + x233*x35;
        double x239 = x238*x33;
        double x240 = -x221;
        double x241 = x158*x240 + x229*x54;
        double x242 = x171*x233 + x232;
        double x243 = x15*x229 + x184*x221;
        double x244 = -x233;
        double x245 = x143*x244 + x223*x32;
        double x246 = x136*x43;
        double x247 = x136*x33;
        double x248 = x133*x35;
        double x249 = x247 - x248;
        double x250 = x249*x33;
        double x251 = x121*x250;
        double x252 = x123*x250;
        double x253 = x133*x143 + x142*x32;
        double x254 = wr*x132;
        double x255 = x151*x63;
        double x256 = x151*x52;
        double x257 = x148*x55;
        double x258 = x256 - x257;
        double x259 = x258*x90;
        double x260 = x259*x86;
        double x261 = x259*x92;
        double x262 = x148*x54 + x157*x158;
        double x263 = x164*x84;
        double x264 = x164*x74;
        double x265 = x161*x76;
        double x266 = x264 - x265;
        double x267 = x102*x266;
        double x268 = x267*x97;
        double x269 = x104*x267;
        double x270 = -x161*x171 + x170*x74;
        double x271 = -x270;
        double x272 = x177*x23;
        double x273 = x13*x177;
        double x274 = x15*x174;
        double x275 = -x274;
        double x276 = x273 + x275;
        double x277 = x13*x276;
        double x278 = x114*x277;
        double x279 = x116*x277;
        double x280 = x183*x184 + x275;
        double x281 = x55*x89;
        double x282 = x281*x88;
        double x283 = x55*x95;
        double x284 = x101*x76;
        double x285 = x100*x284;
        double x286 = x106*x76;
        double x287 = x110*x15;
        double x288 = x117*x35;
        double x289 = x138*x35;
        double x290 = x121*x289;
        double x291 = x123*x289;
        double x292 = x127*x35;
        double x293 = x153*x281;
        double x294 = x293*x86;
        double x295 = x293*x92;
        double x296 = x166*x284;
        double x297 = x296*x97;
        double x298 = x104*x296;
        double x299 = x15*x179;
        double x300 = x114*x299;
        double x301 = x116*x299;
        double x302 = x125*x15;
        double x303 = x185*x302;
        double x304 = x56*x89;
        double x305 = x304*x86;
        double x306 = x304*x92;
        double x307 = x101*x77;
        double x308 = x307*x97;
        double x309 = x104*x307;
        double x310 = x112*x16;
        double x311 = x113*x310;
        double x312 = x115*x310;
        double x313 = x119*x36;
        double x314 = x120*x313;
        double x315 = x122*x313;
        double x316 = x230*x281;
        double x317 = x234*x284;
        double x318 = x15*x236;
        double x319 = x238*x35;
        double x320 = x249*x35;
        double x321 = x121*x320;
        double x322 = x123*x320;
        double x323 = x258*x281;
        double x324 = x323*x86;
        double x325 = x323*x92;
        double x326 = x266*x284;
        double x327 = x326*x97;
        double x328 = x104*x326;
        double x329 = x15*x276;
        double x330 = x114*x329;
        double x331 = x116*x329;
        double x332 = x16 + pow(x184, 2);
        double x333 = sqrt(x332);
        double x334 = fix(1/x333);
        double x335 = smooth(x333);
        double x336 = x334*x335;
        double x337 = cof(x333);
        double x338 = x337*x9;
        double x339 = x336*x338;
        double x340 = x337*x5;
        double x341 = x336*x340;
        double x342 = deriv(this::smooth, x333);
        double x343 = x340*x342;
        double x344 = fix(1/x332);
        double x345 = -x10*x184 + x109;
        double x346 = -x344*x345;
        double x347 = x15*x346;
        double x348 = x335*deriv(this::cof, x333);
        double x349 = x348*x5;
        double x350 = x184*x346;
        double x351 = x338*x342;
        double x352 = x348*x9;
        double x353 = fix(pow(x332, -3.0/2.0));
        double x354 = x335*x353;
        double x355 = x340*x354;
        double x356 = x345*x355;
        double x357 = x338*x354;
        double x358 = x345*x357;
        double x359 = a*x129;
        double x360 = pow(x143, 2);
        double x361 = pow(x32, 2) + x360;
        double x362 = sqrt(x361);
        double x363 = fix(1/x362);
        double x364 = smooth(x362);
        double x365 = x363*x364;
        double x366 = cof(x362);
        double x367 = x366*x9;
        double x368 = x365*x367;
        double x369 = x366*x5;
        double x370 = x365*x369;
        double x371 = fix(1/x361);
        double x372 = x143*x31 + x30*x32;
        double x373 = x371*x372;
        double x374 = x32*x373;
        double x375 = deriv(this::smooth, x362);
        double x376 = x367*x375;
        double x377 = deriv(this::cof, x362);
        double x378 = x364*x377;
        double x379 = x378*x9;
        double x380 = fix(pow(x361, -3.0/2.0));
        double x381 = x364*x380;
        double x382 = x369*x381;
        double x383 = x143*x372;
        double x384 = pow(x171, 2);
        double x385 = x384 + x75;
        double x386 = sqrt(x385);
        double x387 = fix(1/x386);
        double x388 = cof(x386);
        double x389 = smooth(x386);
        double x390 = x388*x389;
        double x391 = x387*x390;
        double x392 = x391*x5;
        double x393 = x391*x9;
        double x394 = fix(1/x385);
        double x395 = x171*x73 + x74*x98;
        double x396 = x395*x74;
        double x397 = x394*x396;
        double x398 = deriv(this::smooth, x386);
        double x399 = x388*x398;
        double x400 = x399*x5;
        double x401 = deriv(this::cof, x386);
        double x402 = x389*x401;
        double x403 = x402*x5;
        double x404 = fix(pow(x385, -3.0/2.0));
        double x405 = x390*x404;
        double x406 = x405*x9;
        double x407 = x171*x395;
        double x408 = x394*x407;
        double x409 = x408*x9;
        double x410 = x405*x5;
        double x411 = b*x129;
        double x412 = x397*x9;
        double x413 = pow(x54, 2);
        double x414 = pow(x158, 2) + x413;
        double x415 = sqrt(x414);
        double x416 = fix(1/x415);
        double x417 = smooth(x415);
        double x418 = x416*x417;
        double x419 = cof(x415);
        double x420 = x419*x9;
        double x421 = x418*x420;
        double x422 = x419*x5;
        double x423 = x418*x422;
        double x424 = fix(1/x414);
        double x425 = -x158*x50 + x54*x87;
        double x426 = x424*x425;
        double x427 = x426*x54;
        double x428 = deriv(this::smooth, x415);
        double x429 = x422*x428;
        double x430 = deriv(this::cof, x415);
        double x431 = x417*x430;
        double x432 = x431*x5;
        double x433 = -x425;
        double x434 = fix(pow(x414, -3.0/2.0));
        double x435 = x417*x434;
        double x436 = x422*x435;
        double x437 = x433*x436;
        double x438 = x158*x426;
        double x439 = x420*x428;
        double x440 = x431*x9;
        double x441 = x420*x435;
        double x442 = x433*x441;
        double x443 = x369*x375;
        double x444 = x378*x5;
        double x445 = x143*x373;
        double x446 = x367*x381;
        double x447 = dt/(IR + x128*(pow(a, 2) + pow(b, 2)));
        double x448 = (1.0/4.0)*x447;
        double x449 = x143*x145*x371;
        double x450 = -x145;
        double x451 = x143*x382;
        double x452 = x129*x448;
        double x453 = x159*x424*x54;
        double x454 = -x159;
        double x455 = x436*x54;
        double x456 = x394*x9;
        double x457 = x172*x456*x74;
        double x458 = x171*x172*x394;
        double x459 = x15*x185*x344;
        double x460 = x184*x185*x344;
        double x461 = x23*x9;
        double x462 = x125*x14;
        double x463 = x209*x5;
        double x464 = x210*x5;
        double x465 = x43*x9;
        double x466 = x127*x34;
        double x467 = x212*x5;
        double x468 = x213*x5;
        double x469 = x63*x9;
        double x470 = x53*x95;
        double x471 = x203*x5;
        double x472 = x204*x5;
        double x473 = x84*x9;
        double x474 = x106*x75;
        double x475 = x206*x5;
        double x476 = x207*x5;
        double x477 = x16*x344;
        double x478 = x15*x184;
        double x479 = x344*x478;
        double x480 = x35*x371;
        double x481 = x32*x480;
        double x482 = x32*x446;
        double x483 = x143*x480;
        double x484 = x456*x74;
        double x485 = x484*x76;
        double x486 = x171*x394;
        double x487 = x486*x76;
        double x488 = x406*x74;
        double x489 = x424*x55;
        double x490 = x158*x489;
        double x491 = x158*x441;
        double x492 = x489*x54;
        double x493 = x242*x484;
        double x494 = x242*x486;
        double x495 = x171*x242;
        double x496 = x241*x424;
        double x497 = x158*x496;
        double x498 = x245*x371;
        double x499 = x32*x498;
        double x500 = x143*x498;
        double x501 = x496*x54;
        double x502 = x15*x243;
        double x503 = x344*x502;
        double x504 = x184*x243;
        double x505 = x344*x504;
        double x506 = x242*x394*x74;
        double x507 = x253*x371;
        double x508 = x32*x507;
        double x509 = x143*x507;
        double x510 = wr*x452;
        double x511 = x262*x424;
        double x512 = x158*x511;
        double x513 = x511*x54;
        double x514 = x271*x484;
        double x515 = x271*x486;
        double x516 = x280*x344;
        double x517 = x15*x516;
        double x518 = x184*x516;
        double x519 = -x280;
        double x520 = x117*x248;
        double x521 = x117*x247;
        double x522 = x135*x43 + x137*x43;
        double x523 = 1.0/Ir;
        double x524 = wr*x131*x523;
        double x525 = x524*stuck[0];
        double x526 = x138*x26;
        double x527 = x247*x526;
        double x528 = x248*x526;
        double x529 = x144*x35;
        double x530 = x247*x32;
        double x531 = x137*x33;
        double x532 = x238*x248;
        double x533 = x127*x245;
        double x534 = x238*x247;
        double x535 = -4*kVD;
        double x536 = rollingCof(dq3);
        double x537 = x247*x249;
        double x538 = x127*x253;
        double x539 = x248*x249;
        double x540 = pow(wr, 2)*x129;
        double x541 = deriv(this::rollingCof, dq3);
        double x542 = x130*x523;
        double x543 = x88*x89;
        double x544 = x257*x543;
        double x545 = x93*x95;
        double x546 = x256*x543;
        double x547 = x150*x63 + x152*x63;
        double x548 = x524*stuck[1];
        double x549 = x153*x46;
        double x550 = x549*x89;
        double x551 = x256*x550;
        double x552 = x549*x95;
        double x553 = x257*x550;
        double x554 = x150*x281;
        double x555 = x152*x90;
        double x556 = x230*x89;
        double x557 = x256*x556;
        double x558 = x230*x95;
        double x559 = x257*x556;
        double x560 = rollingCof(dq4);
        double x561 = x258*x89;
        double x562 = x256*x561;
        double x563 = x262*x95;
        double x564 = x257*x561;
        double x565 = deriv(this::rollingCof, dq4);
        double x566 = x100*x101;
        double x567 = x265*x566;
        double x568 = x106*x99;
        double x569 = x264*x566;
        double x570 = x163*x84 + x165*x84;
        double x571 = x524*stuck[2];
        double x572 = x166*x66;
        double x573 = x101*x572;
        double x574 = x264*x573;
        double x575 = x106*x572;
        double x576 = x265*x573;
        double x577 = x163*x284;
        double x578 = x102*x165;
        double x579 = x101*x234;
        double x580 = x264*x579;
        double x581 = x106*x234;
        double x582 = x265*x579;
        double x583 = rollingCof(dq5);
        double x584 = x101*x266;
        double x585 = x264*x584;
        double x586 = x106*x271;
        double x587 = x265*x584;
        double x588 = deriv(this::rollingCof, dq5);
        double x589 = x110*x274;
        double x590 = x110*x273;
        double x591 = x176*x23 + x178*x23;
        double x592 = x524*stuck[3];
        double x593 = x179*x3;
        double x594 = x273*x593;
        double x595 = x274*x593;
        double x596 = x15*x176;
        double x597 = x13*x178;
        double x598 = x236*x274;
        double x599 = x125*x243;
        double x600 = x236*x273;
        double x601 = rollingCof(dq6);
        double x602 = x273*x276;
        double x603 = x125*x280;
        double x604 = x274*x276;
        double x605 = deriv(this::rollingCof, dq6);
        double x606 = 1 - dt*kVT/It;
    
        result.set(0, 0, 1);
        result.set(0, 11, x0);
        result.set(1, 1, 1);
        result.set(1, 12, x0);
        result.set(2, 2, 1);
        result.set(2, 13, x0);
        result.set(3, 3, 1);
        result.set(3, 14, x0);
        result.set(4, 4, 1);
        result.set(4, 15, x0);
        result.set(5, 5, 1);
        result.set(5, 16, x0);
        result.set(6, 6, 1);
        result.set(6, 17, x0);
        result.set(7, 7, 1);
        result.set(7, 18, x0);
        result.set(8, 8, 1);
        result.set(8, 19, x0);
        result.set(9, 9, 1);
        result.set(9, 20, x0);
        result.set(10, 10, 1);
        result.set(10, 21, x0);
        result.set(11, 2, -x132*(x103*x104 + x103*x97 + x107*x99 + x111*x114 + x111*x116 - x111*x125 + x118*x121 + x118*x123 - x118*x127 + x24 + x44 - x64 - x85 + x86*x91 + x91*x92 + x93*x96));
        result.set(11, 7, x147*(x134 + x140 + x141 - x145*x146));
        result.set(11, 8, x160*(x149 + x155 + x156 - x159*x96));
        result.set(11, 9, x173*(-x107*x172 + x162 + x168 + x169));
        result.set(11, 10, x188*(x175 + x181 + x182 + x187));
        result.set(11, 11, -x132*(x105*x75*x81*x82 + x124*x14*x20*x21 + x126*x34*x40*x41 - x190 - x191 - x193 - x194 - x196 - x197 - x199 - x200 - x201 + x53*x60*x61*x94) + 1);
        result.set(11, 12, x218);
        result.set(11, 13, x132*(x104*x235 - x107*x242 + x114*x237 + x116*x237 + x121*x239 + x123*x239 - x146*x245 - x186*x243 + x222 + x224 - x225 - x226 + x231*x86 + x231*x92 + x235*x97 - x241*x96));
        result.set(11, 14, x254*(-x146*x253 + x246 + x251 + x252));
        result.set(11, 15, x254*(x255 + x260 + x261 - x262*x96));
        result.set(11, 16, x254*(-x107*x271 + x263 + x268 + x269));
        result.set(11, 17, x254*(-x186*x280 + x272 + x278 + x279));
        result.set(12, 2, -x132*(x104*x285 + x108*x23 + x114*x287 + x116*x287 + x121*x288 + x123*x288 - x125*x287 - x127*x288 + x282*x86 + x282*x92 + x283*x93 + x285*x97 + x286*x99 + x31*x43 - x51*x63 - x73*x84));
        result.set(12, 7, x147*(-x145*x292 + x246 + x290 + x291));
        result.set(12, 8, x160*(-x159*x283 + x255 + x294 + x295));
        result.set(12, 9, x173*(-x172*x286 + x263 + x297 + x298));
        result.set(12, 10, x188*(x272 + x300 + x301 + x303));
        result.set(12, 11, x218);
        result.set(12, 12, -x132*(x105*x77*x81*x82 + x124*x16*x20*x21 + x126*x36*x40*x41 - x201 - x305 - x306 - x308 - x309 - x311 - x312 - x314 - x315 + x56*x60*x61*x94) + 1);
        result.set(12, 13, x132*(x104*x317 + x114*x318 + x116*x318 + x121*x319 + x123*x319 + x229*x23 - x229*x63 + x233*x43 - x233*x84 - x241*x283 - x242*x286 - x243*x302 - x245*x292 + x316*x86 + x316*x92 + x317*x97));
        result.set(12, 14, -x254*(x126*x249*x35*x40*x41 + x133*x39*x40*x41 - x321 - x322));
        result.set(12, 15, -x254*(x148*x59*x60*x61 + x258*x55*x60*x61*x94 - x324 - x325));
        result.set(12, 16, -x254*(x105*x266*x76*x81*x82 + x161*x80*x81*x82 - x327 - x328));
        result.set(12, 17, -x254*(x124*x15*x20*x21*x276 + x174*x19*x20*x21 - x330 - x331));
        result.set(13, 2, x448*(a*x129*(-x158*x423 - x158*x442 + x421*x50 - x421*x54 + x423*x87 + x427*x429 + x427*x432 + x437*x54 - x438*x439 - x438*x440) + a*x129*(x171*x388*x389*x395*x404*x5 - x171*x393 + x387*x388*x389*x5*x74 + x388*x389*x395*x404*x74*x9 - x392*x73 - x393*x98 - x399*x412 - x400*x408 - x402*x412 - x403*x408) + b*x129*(x143*x370 + x30*x370 + x31*x368 - x32*x368 - x32*x372*x382 + x374*x443 + x374*x444 + x376*x445 + x379*x445 - x383*x446) + b*x129*(-x158*x421 + x158*x437 + x421*x87 - x423*x50 + x423*x54 + x427*x439 + x427*x440 + x429*x438 + x432*x438 + x442*x54) - x359*(-x10*x339 + x108*x334*x335*x337*x5 + x15*x334*x335*x337*x9 - x15*x356 - x184*x341 - x184*x358 - x343*x347 - x347*x349 - x350*x351 - x350*x352) - x359*(x143*x364*x371*x372*x377*x5 + x143*x366*x371*x372*x375*x5 - x143*x368 - x30*x368 + x31*x363*x364*x366*x5 + x32*x364*x366*x372*x380*x9 - x32*x370 - x374*x376 - x374*x379 - x382*x383) - x411*(x10*x341 + x108*x339 - x15*x341 - x15*x358 - x184*x339 + x184*x356 + x343*x350 - x347*x351 - x347*x352 + x349*x350) - x411*(x171*x392 + x392*x98 - x393*x73 + x393*x74 - x396*x410 + x397*x400 + x397*x403 - x399*x409 - x402*x409 + x406*x407)));
        result.set(13, 7, x26*x452*(a*(-x133*x368 - x142*x370 + x145*x32*x364*x371*x377*x9 + x145*x32*x366*x371*x375*x9 + x32*x364*x366*x380*x450*x9 - x443*x449 - x444*x449 - x450*x451) + b*(x126*x145*x33*x40*x41*x5 + x126*x145*x35*x40*x41*x9 - x134*x5 - x140*x5 - x141*x5 - x246*x9 - x290*x9 - x291*x9)));
        result.set(13, 8, x452*x46*(-a*(-x148*x421 - x157*x423 + x158*x159*x417*x424*x430*x9 + x158*x159*x419*x424*x428*x9 + x158*x417*x419*x434*x454*x9 - x429*x453 - x432*x453 - x454*x455) + b*(-x149*x5 - x155*x5 - x156*x5 + x159*x5*x52*x60*x61*x94 + x159*x55*x60*x61*x9*x94 - x255*x9 - x294*x9 - x295*x9)));
        result.set(13, 9, -x452*x66*(a*(-x161*x393 - x170*x392 + x171*x172*x388*x389*x404*x5 + x172*x388*x389*x404*x74*x9 - x399*x457 - x400*x458 - x402*x457 - x403*x458) + b*(x105*x172*x5*x74*x81*x82 + x105*x172*x76*x81*x82*x9 - x162*x5 - x168*x5 - x169*x5 - x263*x9 - x297*x9 - x298*x9)));
        result.set(13, 10, x3*x452*(a*(x15*x185*x335*x337*x353*x5 - x174*x339 - x183*x341 + x184*x185*x335*x337*x353*x9 - x343*x459 - x349*x459 - x351*x460 - x352*x460) + b*(x175*x5 + x181*x5 + x182*x5 + x187*x5 + x272*x9 + x300*x9 + x301*x9 + x303*x9)));
        result.set(13, 11, x452*(a*(x190*x9 + x191*x9 + x214*x5 + x469 - x470*x9 - x471 - x472) + a*(x193*x9 + x194*x9 + x215*x5 + x473 - x474*x9 - x475 - x476) - a*(x196*x9 + x197*x9 + x216*x5 + x461 - x462*x9 - x463 - x464) - a*(x199*x9 + x200*x9 + x217*x5 + x465 - x466*x9 - x467 - x468) + b*(-x190*x5 - x191*x5 - x203*x9 - x204*x9 + x5*x53*x60*x61*x94 - x5*x63 + x52*x55*x60*x61*x9*x94) - b*(x105*x5*x75*x81*x82 + x105*x74*x76*x81*x82*x9 - x193*x5 - x194*x5 - x206*x9 - x207*x9 - x5*x84) - b*(x124*x13*x15*x20*x21*x9 + x124*x14*x20*x21*x5 - x196*x5 - x197*x5 - x209*x9 - x210*x9 - x23*x5) + b*(x126*x33*x35*x40*x41*x9 + x126*x34*x40*x41*x5 - x199*x5 - x200*x5 - x212*x9 - x213*x9 - x43*x5)));
        result.set(13, 12, x452*(a*(x143*x482 - x360*x382 + x370 + x376*x481 + x379*x481 - x443*x483 - x444*x483) + a*(-x16*x355 + x341 + x343*x477 + x349*x477 + x351*x479 + x352*x479 - x357*x478) - a*(-x171*x488 - x384*x410 + x387*x388*x389*x5 - x399*x485 - x400*x487 - x402*x485 - x403*x487) - a*(-x413*x436 + x423 - x429*x492 - x432*x492 + x439*x490 + x440*x490 + x491*x54) + b*(-x305*x9 - x306*x9 - x469 - x471 - x472 + x5*x52*x55*x60*x61*x94 + x56*x60*x61*x9*x94) - b*(x105*x5*x74*x76*x81*x82 + x105*x77*x81*x82*x9 - x308*x9 - x309*x9 - x473 - x475 - x476) - b*(x124*x13*x15*x20*x21*x5 + x124*x16*x20*x21*x9 - x311*x9 - x312*x9 - x461 - x463 - x464) + b*(x126*x33*x35*x40*x41*x5 + x126*x36*x40*x41*x9 - x314*x9 - x315*x9 - x465 - x467 - x468)));
        result.set(13, 13, x447*(2.4516624999999999*mr + 9.8066499999999994*mw)*(a*(x223*x393 + x233*x392 - x242*x488 + x399*x493 + x400*x494 + x402*x493 + x403*x494 - x410*x495) - a*(x143*x245*x364*x371*x377*x5 + x143*x245*x366*x371*x375*x5 - x223*x368 + x244*x363*x364*x366*x5 + x245*x32*x364*x366*x380*x9 - x245*x451 - x376*x499 - x379*x499) - a*(x15*x243*x335*x337*x353*x5 + x184*x243*x335*x337*x353*x9 - x221*x339 - x229*x341 - x343*x503 - x349*x503 - x351*x505 - x352*x505) + a*(x158*x241*x417*x419*x434*x9 + x229*x416*x417*x419*x5 - x240*x421 + x241*x417*x424*x430*x5*x54 + x241*x419*x424*x428*x5*x54 - x241*x455 - x439*x497 - x440*x497) - b*(x221*x341 - x229*x339 + x343*x505 + x349*x505 - x351*x503 - x352*x503 - x355*x504 + x357*x502) + b*(-x143*x245*x446 + x223*x370 + x244*x368 - x245*x32*x382 + x376*x500 + x379*x500 + x443*x499 + x444*x499) + b*(-x158*x241*x436 + x229*x421 + x240*x423 - x241*x441*x54 + x429*x497 + x432*x497 + x439*x501 + x440*x501) - b*(x171*x242*x388*x394*x398*x9 + x171*x242*x389*x394*x401*x9 - x223*x392 + x233*x387*x388*x389*x9 + x242*x388*x389*x404*x5*x74 - x400*x506 - x403*x506 - x406*x495)) + 1);
        result.set(13, 14, x510*(a*(-x133*x370 + x142*x368 + x253*x451 - x253*x482 + x376*x508 + x379*x508 - x443*x509 - x444*x509) + b*(x126*x253*x33*x40*x41*x5 + x126*x253*x35*x40*x41*x9 + x133*x39*x40*x41*x9 - x246*x5 - x251*x5 - x252*x5 - x321*x9 - x322*x9)));
        result.set(13, 15, x510*(-a*(-x148*x423 + x157*x421 + x262*x455 - x262*x491 - x429*x513 - x432*x513 + x439*x512 + x440*x512) + b*(x148*x59*x60*x61*x9 - x255*x5 - x260*x5 - x261*x5 + x262*x5*x52*x60*x61*x94 + x262*x55*x60*x61*x9*x94 - x324*x9 - x325*x9)));
        result.set(13, 16, -x510*(a*(-x161*x392 + x170*x387*x388*x389*x9 - x171*x270*x410 - x270*x488 - x399*x514 - x400*x515 - x402*x514 - x403*x515) + b*(x105*x271*x5*x74*x81*x82 + x105*x271*x76*x81*x82*x9 + x161*x80*x81*x82*x9 - x263*x5 - x268*x5 - x269*x5 - x327*x9 - x328*x9)));
        result.set(13, 17, x510*(a*(x15*x355*x519 - x174*x341 + x183*x339 + x184*x357*x519 + x343*x517 + x349*x517 + x351*x518 + x352*x518) - b*(x124*x13*x20*x21*x280*x5 + x124*x15*x20*x21*x280*x9 + x174*x19*x20*x21*x9 - x272*x5 - x278*x5 - x279*x5 - x330*x9 - x331*x9)));
        result.set(14, 2, x525*(x121*x520 - x121*x521 + x123*x520 - x123*x521 - x127*x520 + x127*x521 + x134*x31 - x136*x44 + x522));
        result.set(14, 7, x525*(x121*x527 - x121*x528 + x123*x527 - x123*x528 - x127*x527 + x127*x528 + x522));
        result.set(14, 11, x525*(x121*x529 - x121*x530 + x123*x529 - x123*x530 + x135*x292 - x136*x466 + x246));
        result.set(14, 12, -x525*(-x121*x531 - x123*x531 - x127*x133*x36 + x133*x314 + x133*x315 + x134 + x137*x146));
        result.set(14, 13, -x525*(x121*x532 - x121*x534 + x123*x532 - x123*x534 + x134*x233 + x136*x226 + x247*x533 - x248*x533));
        result.set(14, 14, x542*(x535 + x540*(x121*x537 - x121*x539 + x123*x537 - x123*x539 + pow(x133, 2)*x43 + pow(x136, 2)*x43 - x247*x538 + x248*x538) + 4*x541*(x541*signum(dq3))*smooth(dq3) + 4*copySign(x536, dq3)*deriv(this::smooth, dq3))*stuck[0] + 1);
        result.set(15, 2, x548*(-x149*x51 + x151*x64 - x256*x545 + x257*x545 + x544*x86 + x544*x92 - x546*x86 - x546*x92 + x547));
        result.set(15, 8, x548*(-x256*x552 + x257*x552 + x547 + x551*x86 + x551*x92 - x553*x86 - x553*x92));
        result.set(15, 11, x548*(x150*x283 + x151*x190 + x151*x191 - x151*x470 + x255 - x554*x86 - x554*x92));
        result.set(15, 12, -x548*(x148*x305 + x148*x306 - x148*x56*x95 + x149 + x152*x96 - x555*x86 - x555*x92));
        result.set(15, 13, x548*(x149*x229 + x151*x222 - x256*x558 + x257*x558 + x557*x86 + x557*x92 - x559*x86 - x559*x92));
        result.set(15, 15, x542*(x535 + x540*(pow(x148, 2)*x63 + pow(x151, 2)*x63 - x256*x563 + x257*x563 + x562*x86 + x562*x92 - x564*x86 - x564*x92) + 4*x565*(x565*signum(dq4))*smooth(dq4) + 4*copySign(x560, dq4)*deriv(this::smooth, dq4))*stuck[1] + 1);
        result.set(16, 2, x571*(x104*x567 - x104*x569 - x162*x73 + x164*x85 - x264*x568 + x265*x568 + x567*x97 - x569*x97 + x570));
        result.set(16, 9, x571*(x104*x574 - x104*x576 - x264*x575 + x265*x575 + x570 + x574*x97 - x576*x97));
        result.set(16, 11, x571*(-x104*x577 + x163*x286 + x164*x193 + x164*x194 - x164*x474 + x263 - x577*x97));
        result.set(16, 12, -x571*(-x104*x578 - x106*x161*x77 + x107*x165 + x161*x308 + x161*x309 + x162 - x578*x97));
        result.set(16, 13, x571*(x104*x580 - x104*x582 + x162*x233 + x164*x224 - x264*x581 + x265*x581 + x580*x97 - x582*x97));
        result.set(16, 16, x542*(x535 + x540*(x104*x585 - x104*x587 + pow(x161, 2)*x84 + pow(x164, 2)*x84 - x264*x586 + x265*x586 + x585*x97 - x587*x97) + 4*x588*(x588*signum(dq5))*smooth(dq5) + 4*copySign(x583, dq5)*deriv(this::smooth, dq5))*stuck[2] + 1);
        result.set(17, 2, x592*(x108*x175 + x114*x589 - x114*x590 + x116*x589 - x116*x590 - x125*x589 + x125*x590 - x177*x24 + x591));
        result.set(17, 10, x592*(x114*x594 - x114*x595 + x116*x594 - x116*x595 - x125*x594 + x125*x595 + x591));
        result.set(17, 11, x592*(-x114*x596 - x116*x596 + x176*x302 + x177*x196 + x177*x197 - x177*x462 + x272));
        result.set(17, 12, -x592*(-x114*x597 - x116*x597 - x125*x16*x174 + x174*x311 + x174*x312 + x175 + x178*x186));
        result.set(17, 13, -x592*(x114*x598 - x114*x600 + x116*x598 - x116*x600 + x175*x229 + x177*x225 + x273*x599 - x274*x599));
        result.set(17, 17, x542*(x535 + x540*(x114*x602 - x114*x604 + x116*x602 - x116*x604 + pow(x174, 2)*x23 + pow(x177, 2)*x23 - x273*x603 + x274*x603) + 4*x605*(x605*signum(dq6))*smooth(dq6) + 4*copySign(x601, dq6)*deriv(this::smooth, dq6))*stuck[3] + 1);
        result.set(18, 18, x606);
        result.set(19, 19, x606);
        result.set(20, 20, x606);
        result.set(21, 21, x606);
    
        return result;
    }

    // Change in state with respect to time
    private Matrix<N22, N1> dqdt(Matrix<N22, N1> state, Matrix<N8, N1> input) {
        double q0 = state.get(0, 0), dq0 = state.get(11, 0);
        double q1 = state.get(1, 0), dq1 = state.get(12, 0);
        double q2 = state.get(2, 0), dq2 = state.get(13, 0);
        double q3 = state.get(3, 0), dq3 = state.get(14, 0);
        double q4 = state.get(4, 0), dq4 = state.get(15, 0);
        double q5 = state.get(5, 0), dq5 = state.get(16, 0);
        double q6 = state.get(6, 0), dq6 = state.get(17, 0);
        double q7 = state.get(7, 0), dq7 = state.get(18, 0);
        double q8 = state.get(8, 0), dq8 = state.get(19, 0);
        double q9 = state.get(9, 0), dq9 = state.get(20, 0);
        double q10 = state.get(10, 0), dq10 = state.get(21, 0);

        double x0 = sin(q2);
        double x1 = a*dq2;
        double x2 = sin(q8);
        double x3 = dq4*wr;
        double x4 = x1 + x2*x3;
        double x5 = cos(q2);
        double x6 = b*dq2;
        double x7 = cos(q8);
        double x8 = x3*x7 + x6;
        double x9 = dq0 + x0*x4 - x5*x8;
        double x10 = -dq1 + x0*x8 + x4*x5;
        double x11 = -x10;
        double x12 = sqrt(pow(x11, 2) + pow(x9, 2));
        double x13 = cof(x12)*fix(smooth(x12)/x12);
        double x14 = sin(q9);
        double x15 = dq5*wr;
        double x16 = x1 + x14*x15;
        double x17 = cos(q9);
        double x18 = -x15*x17 + x6;
        double x19 = dq0 + x0*x16 + x18*x5;
        double x20 = pow(x19, 2);
        double x21 = dq1 + x0*x18 - x16*x5;
        double x22 = sqrt(x20 + pow(x21, 2));
        double x23 = cof(x22)*fix(smooth(x22)/x22);
        double x24 = cos(q7);
        double x25 = dq3*wr;
        double x26 = x24*x25 + x6;
        double x27 = sin(q7);
        double x28 = x1 - x25*x27;
        double x29 = -dq0 + x0*x28 + x26*x5;
        double x30 = -x29;
        double x31 = dq1 - x0*x26 + x28*x5;
        double x32 = sqrt(pow(x30, 2) + pow(x31, 2));
        double x33 = cof(x32)*fix(smooth(x32)/x32);
        double x34 = cos(q10);
        double x35 = dq6*wr;
        double x36 = -x34*x35 + x6;
        double x37 = sin(q10);
        double x38 = x1 - x35*x37;
        double x39 = dq0 - x0*x38 + x36*x5;
        double x40 = dq1 + x0*x36 + x38*x5;
        double x41 = pow(x40, 2);
        double x42 = sqrt(pow(x39, 2) + x41);
        double x43 = cof(x42)*fix(smooth(x42)/x42);
        double x44 = 4*mw;
        double x45 = 9.8066499999999994*mr + 39.226599999999998*mw;
        double x46 = (1.0/4.0)*x45;
        double x47 = x46/(mr + x44);
        double x48 = b*x45;
        double x49 = -x21;
        double x50 = sqrt(x20 + pow(x49, 2));
        double x51 = a*x45;
        double x52 = -x9;
        double x53 = sqrt(pow(x10, 2) + pow(x52, 2));
        double x54 = -x39;
        double x55 = sqrt(x41 + pow(x54, 2));
        double x56 = -x31;
        double x57 = sqrt(pow(x29, 2) + pow(x56, 2));
        double x58 = 1.0/Ir;
        double x59 = wr*x46;
        double x60 = 1.0/It;

        return MatBuilder.fill(
            N22.instance,
            N1.instance,
            dq0,
            dq1,
            dq2,
            dq3,
            dq4,
            dq5,
            dq6,
            dq7,
            dq8,
            dq9,
            dq10,
            -x47*(x13*x9 + x19*x23 + x30*x33 + x39*x43),
            -x47*(x11*x13 + x21*x23 + x31*x33 + x40*x43),
            -1.0/4.0*(-x13*x48*(x0*x11 + x5*x9) + x23*x48*(x0*x21 + x19*x5) - x33*x48*(x0*x31 + x30*x5) + x43*x48*(x0*x40 + x39*x5) + x51*(x0*x29 - x5*x56)*cof(x57)*fix(smooth(x57)/x57) + x51*(x0*x54 + x40*x5)*cof(x55)*fix(smooth(x55)/x55) - x51*(x0*x52 - x10*x5)*cof(x53)*fix(smooth(x53)/x53) + x51*(x0*x19 + x49*x5)*cof(x50)*fix(smooth(x50)/x50))/(IR + x44*(pow(a, 2) + pow(b, 2))),
            stuck[0]*x58*(input.get(0, 0)*kAD + dq3*kVD - x33*x59*(x30*(x0*x27 - x24*x5) - x31*(x0*x24 + x27*x5)) - smooth(Math.abs(dq3))*copySign(1, dq3)*rollingCof(Math.abs(dq3))),
            stuck[1]*x58*(input.get(1, 0)*kAD + dq4*kVD - x13*x59*(-x11*(x0*x7 + x2*x5) + x9*(x0*x2 - x5*x7)) - smooth(Math.abs(dq4))*copySign(1, dq4)*rollingCof(Math.abs(dq4))),
            stuck[2]*x58*(input.get(2, 0)*kAD + dq5*kVD - x23*x59*(x19*(x0*x14 - x17*x5) - x21*(x0*x17 + x14*x5)) - smooth(Math.abs(dq5))*copySign(1, dq5)*rollingCof(Math.abs(dq5))),
            stuck[3]*x58*(input.get(3, 0)*kAD + dq6*kVD - x43*x59*(x39*(x0*x37 - x34*x5) - x40*(x0*x34 + x37*x5)) - smooth(Math.abs(dq6))*copySign(1, dq6)*rollingCof(Math.abs(dq6))),
            x60*(input.get(4, 0)*kAT + dq7*kVT),
            x60*(input.get(5, 0)*kAT + dq8*kVT),
            x60*(input.get(6, 0)*kAT + dq9*kVT),
            x60*(input.get(7, 0)*kAT + dq10*kVT)
        );
    }

    Matrix<N22, N1> newton(Function<Matrix<N22, N1>, Matrix<N22, N1>> f, Matrix<N22, N1> x, int i) {
        // var jacobian1 = numericalJacobian(N22.instance, N22.instance, f, x);
        var jacobian = jacobian(x);

        // System.out.println(jacobian1);
        // System.out.println(jacobian);
        // System.exit(0);

        Matrix<N22, N1> diff;

        // double speed = 1.4 / pow(2, i / 100);
        double speed = 0.1;

        try {
            diff = jacobian.solve(f.apply(x).times(-speed));
            // if (i == 299) {
            //     System.out.println(diff);
            //     System.out.println(f.apply(x));
            //     System.out.println(jacobian.minus(jacobian1));
            // };
        } catch (Exception e) {
            System.err.println(e);
            System.out.println(x);
            System.out.println(jacobian); 
            System.out.println(input);
            System.out.println(f.apply(x));
            System.exit(0);
            diff = jacobian.solve(f.apply(x).times(-speed));
        }

        return x.plus(diff);
    }

    Matrix<N22, N1> r2(Function<Matrix<N22, N1>, Matrix<N22, N1>> f, Matrix<N22, N1> x, double dt) {
        // var jacobian = jacobian(x);
        var jacobian = numericalJacobian(N22.instance, N22.instance, f, state);
        double d = 1 - sqrt(2) / 2;
        double a21 = (sqrt(2) - 1) / 2;

        calcs += 1;

        var A = Matrix.eye(N22.instance).minus(jacobian.times(d * dt)).inv();
        var k1 = A.times(f.apply(x));
        var k2 = A.times(f.apply(x.plus(k1.times(a21 * dt))));
        return x.plus(k2.times(dt));
    }

    Matrix<N22, N1> r4(Function<Matrix<N22, N1>, Matrix<N22, N1>> f, Matrix<N22, N1> x, double dt) {
        // var jacobian = jacobian(x);
        var jacobian = numericalJacobian(N22.instance, N22.instance, f, state);
        double w[] = { 0.9451564786, 0.341323172, 0.5655139575, -0.8519936081 };
        double a21 = -0.5;
        double a31 = -0.1012236115;
        double a41 = -0.3922096763;
        double a32 = 0.9762236115;
        double a42 = 0.7151140251;
        double a43 = 0.1430371625;

        double d = 0.5728160625;

        calcs += 1;

        try {
            var A = Matrix.eye(N22.instance).minus(jacobian.times(d * dt)).inv();
            var k1 = A.times(f.apply(x));
            var k2 = A.times(f.apply(x.plus(k1.times(a21 * dt))));
            var k3 = A.times(f.apply(x.plus(k1.times(a31).plus(k2.times(a32)).times(dt))));
            var k4 = A.times(f.apply(x.plus(k1.times(a41).plus(k2.times(a42)).plus(k3.times(a43)).times(dt))));
            return x.plus(k1.times(w[0]).plus(k2.times(w[1])).plus(k3.times(w[2])).plus(k4.times(w[3])).times(dt));
        } catch (Exception e) {
            System.out.println(jacobian.times(d * dt));
            System.exit(1);
            return new Matrix<N22, N1>(N22.instance, N1.instance);
        }
    }

    Matrix<N22, N1> r_iter(Function<Matrix<N22, N1>, Matrix<N22, N1>> f, Matrix<N22, N1> ym, Matrix<N22, N1> x, double dt, int depth) {
        var ym1 = r2(f, x, dt/2);
        var ym2 = r2(f, ym1, dt/2);

        max_depth = max(max_depth, depth);
        iters += 1;

        double e = new Vector<N22>(ym2.minus(ym)).norm();

        if (e > 0.1 && depth < 8) {
            var ym3 = r_iter(f, ym1, x, dt/2, depth + 1);
            var ym4 = r2(f, ym3, dt/2);

            return r_iter(f, ym4, ym3, dt/2, depth + 1);
        } else {
            return ym2;
        }
    }

    // private double getDriveCurrent(int n) {
    //     return driveMotor.getCurrent(getDriveAngularVelocity(n) * driveG, input.get(n));
    // }

    // private double getTurnCurrent(int n) {
    //     return turnMotor.getCurrent(getTurnAngularVelocity(n) * turnG, input.get(n + 4));
    // }

    // private double getDriveTorque(int n) {
    //     return driveMotor.getTorque(getDriveCurrent(n));
    // }

    // private double getTurnTorque(int n) {
    //     return turnMotor.getTorque(getTurnCurrent(n));
    // }

    private boolean isSlipping(int n) {
        double[] xi = { a, -a, -a, a };
        double[] yi = { b, b, -b, -b };

        double dtheta = getAngularVelocity();
        double stheta = getRotation().getSin();
        double ctheta = getRotation().getCos();

        double xs =  xi[n] * stheta * dtheta + yi[n] * ctheta * dtheta + wr * cos(getAngle() + getTurnAngularPosition(n)) * getDriveAngularVelocity(n) - getXVel();
        double ys = -xi[n] * ctheta * dtheta + yi[n] * stheta * dtheta + wr * sin(getAngle() + getTurnAngularPosition(n)) * getDriveAngularVelocity(n) - getYVel();

        double slip = hypot(xs, ys);

        return (slip > 0.02);
    }

    // Iterative calculation of voltage (doesnt work)
    // private double balanceVoltage(Function<Double, Double> totalCurrent) {
    //     double v = 12;
    //     for (int i = 0; i < 200; i++) {
    //         double newv = 12 - 0.02 * totalCurrent.apply(v);
    //         if (Math.abs(newv - v) < 0.01) {
    //             bat_iters = i + 1;
    //             return newv;
    //         }

    //         v = newv;
    //     }

    //     bat_iters = 200;
    //     return v;
    // }

    private double solve_voltage(int[] e, Matrix<N8, N1> input) {
        double rb = 0.02;
        double r1 = driveMotor.rOhms + wireResistance;
        double r2 = turnMotor.rOhms + wireResistance;

        double top = -12 * r1 * r2;

        for (int i = 0; i < 4; i++) {
            top += -rb * Math.abs(getDriveAngularVelocity(i)) * driveG / driveMotor.KvRadPerSecPerVolt * r2;
            top += -rb * Math.abs(getTurnAngularVelocity(i)) * turnG / turnMotor.KvRadPerSecPerVolt * r1;
            top += rb * e[i] * Math.abs(input.get(i, 0)) * r2;
            top += rb * e[i + 4] * Math.abs(input.get(i + 4, 0)) * r1;
        }

        double bottom = -r1 * r2 - 4 * rb * (r1 + r2);

        for (int i = 0; i < 4; i++) {
            bottom += rb * e[i] * r2;
            bottom += rb * e[i + 4] * r1;
        }

        return top / bottom;
    }
    
    private double battery_voltage(Matrix<N8, N1> input) {
        final Integer[] idx = { 0, 1, 2, 3, 4, 5, 6, 7 };
        final double[] data = input.getData();

        // Sort idx by voltage data
        java.util.Arrays.sort(idx, new java.util.Comparator<Integer>() {
            @Override public int compare(final Integer o1, final Integer o2) {
                return Double.compare(Math.abs(data[o1]), Math.abs(data[o2]));
            }
        });

        // System.out.println(Arrays.toString(idx));
        // System.out.println(Arrays.toString(data));

        int[] e = { 0, 0, 0, 0, 0, 0, 0, 0 };
        for (int i : idx) {
            double v = solve_voltage(e, input);
            // System.out.println(String.format("%f %f", v, 12 - 0.02 * currents(input, v)));
            if (Util.epsilonEquals(12 - 0.02 * currents(input, v), v)) {
                return v;
            }
            e[i] = 1;
        }

        return solve_voltage(e, input);
    }

    // Total current
    private double currents(Matrix<N8, N1> input, double v) {
        double c = 0;

        for (int n = 0; n < 4; n++) {
            double dv = input.get(n, 0);
            double tv = input.get(n + 4, 0);

            c += getDriveSupplyCurrent(n, Math.copySign(Math.min(Math.abs(dv), v), dv));
            c += getTurnSupplyCurrent(n, Math.copySign(Math.min(Math.abs(tv), v), tv));
        }

        return c;
    }

    public static <Rows extends Num, Cols extends Num, States extends Num>
        Matrix<Rows, Cols> numericalJacobian(
            Nat<Rows> rows,
            Nat<Cols> cols,
            Function<Matrix<Cols, N1>, Matrix<States, N1>> f,
            Matrix<Cols, N1> x)
    {
        var result = new Matrix<>(rows, cols);

        for (int i = 0; i < cols.getNum(); i++) {
            var dxPlus = x.copy();
            var dxMinus = x.copy();
            dxPlus.set(i, 0, dxPlus.get(i, 0) + 1e-9);
            dxMinus.set(i, 0, dxMinus.get(i, 0) - 1e-9);
            var dF = f.apply(dxPlus).minus(f.apply(dxMinus)).div(2 * 1e-9);

            result.setColumn(i, Matrix.changeBoundsUnchecked(dF));
        }

        return result;
    }

    private double getDriveSupplyCurrent(int n, double volts) {
        return (Math.abs(volts) - Math.abs(getDriveAngularVelocity(n)) * driveG / driveMotor.KvRadPerSecPerVolt) / (driveMotor.rOhms + wireResistance);
    }

    private double getTurnSupplyCurrent(int n, double volts) {
        return (Math.abs(volts) - Math.abs(getTurnAngularVelocity(n)) * turnG / turnMotor.KvRadPerSecPerVolt) / (turnMotor.rOhms + wireResistance);
    }

    public void update() {
        var limitedInput = input.copy();
                        
        for (int n = 0; n < 4; n++) {
            limitedInput.set(n, 0, currentLimitDrive(n, input.get(n, 0)));
            limitedInput.set(n + 4, 0, currentLimitTurn(n, input.get(n + 4, 0)));

            // totalCurrent += Math.abs(driveMotor.getCurrent(getDriveAngularVelocity(n) * driveG, limitedInput.get(n, 0)));
            // totalCurrent += Math.abs(turnMotor.getCurrent(getTurnAngularVelocity(n) * turnG, limitedInput.get(n + 4, 0)));
        }

        // double batteryVoltage = balanceVoltage((v) -> currents(limitedInput, v));
        double batteryVoltage = battery_voltage(limitedInput);

        for (int n = 0; n < 4; n++) {
            limitedInput.set(n, 0, MathUtil.clamp(limitedInput.get(n, 0), -batteryVoltage, batteryVoltage));
            limitedInput.set(n + 4, 0, MathUtil.clamp(limitedInput.get(n + 4, 0), -batteryVoltage, batteryVoltage));

            // totalCurrent += Math.abs(driveMotor.getCurrent(getDriveAngularVelocity(n) * driveG, limitedInput.get(n, 0)));
            // totalCurrent += Math.abs(turnMotor.getCurrent(getTurnAngularVelocity(n) * turnG, limitedInput.get(n + 4, 0)));
        }

        // Logger.recordOutput("Model/Total Current", totalCurrent);
        Logger.recordOutput("Model/Battery Voltage", batteryVoltage);
        // Logger.recordOutput("Model/Battery Voltage 2", v2);

        // for (int n = 0; n < 8; n++) {
        //     // double v = limitedInput.get(n, 0);
        //     limitedInput.set(n, 0, MathUtil.clamp(limitedInput.get(n, 0), -cappedVoltage, cappedVoltage));
        // }

        Logger.recordOutput("Model/Input", limitedInput.getData());

        Function<Matrix<N22, N1>, Matrix<N22, N1>> fF = (q) -> q.minus(dqdt(q, limitedInput).times(dt)).minus((Matrix<N22, N1>)state);
        Function<Matrix<N22, N1>, Matrix<N22, N1>> f = (q) -> dqdt(q, limitedInput);

        double error = 999.0;
        Matrix<N22, N1> next = state.copy();

        // int i = 0;

        // for (int i = 0; i < 32; i++) {
        //     next = r4(f, next, dt / 32);
        // }


        next = r_iter(f, r2(f, next, dt), next, dt, 0);
        error = new Vector<N22>(fF.apply(next)).norm();
        Logger.recordOutput("Model/Max depth", max_depth);
        Logger.recordOutput("Model/Iters", iters);
        Logger.recordOutput("Model/Calcs", calcs);
        Logger.recordOutput("Model/Battery Iters", bat_iters);

        max_depth = 0;
        iters = 0;
        calcs = 0;

        // while (Math.abs(error) > 1e-4 && i < 1) {
        //     var update = r4(f, next, i);
        //     // double nextErr = f.apply(next).mean();
        //     // System.out.println(i);
        //     // System.out.println(f.apply(update));
        //     // System.out.println(update.minus(next));
        //     next = update;
        //     error = new Vector<N22>(fF.apply(next)).norm();
        //     i++;
        // }

        Logger.recordOutput("Model/Error", error);
        // Logger.recordOutput("Model/Iterations", i);
        Logger.recordOutput("Model/State", state.getData());

        for (int n = 0; n < 4; n++) {
            String prefix = "Model/Module " + Integer.toString(n);
            double cr = driveMotor.getCurrent(getDriveAngularVelocity(n) * driveG, limitedInput.get(n, 0));
            Logger.recordOutput(prefix + "/Drive Stator Current", cr);
            Logger.recordOutput(prefix + "/Drive Supply Current", getDriveSupplyCurrent(n, limitedInput.get(n, 0)));
            Logger.recordOutput(prefix + "/Turn Current", turnMotor.getCurrent(getTurnAngularVelocity(n) * turnG, limitedInput.get(n + 4, 0)));
            Logger.recordOutput(prefix + "/Drive Voltage", input.get(n, 0));
            Logger.recordOutput(prefix + "/Drive Limited Voltage", limitedInput.get(n, 0));
            Logger.recordOutput(prefix + "/Turn Voltage", limitedInput.get(n + 4, 0));
            // Logger.recordOutput(prefix + "/Drive Torque", getDriveTorque(n));
            // Logger.recordOutput(prefix + "/Turn Torque", getTurnTorque(n));
            Logger.recordOutput(prefix + "/Is Slipping", isSlipping(n));
        }

        state = new Vector<N22>(next);
    }
}

final class N22 extends Num implements Nat<N22> {
    private N22() {}

    @Override
    public int getNum() {
        return 22;
    }

    public static final Nat<N22> instance = new N22();
}   