package frc.robot.subsystems.MAXSwerve;

import edu.wpi.first.math.MatBuilder;
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
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sqrt;
import java.util.function.Function;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSim {
    private final double dt = 0.02;

    private final double wr = DriveConstants.ModuleConstants.kWheelDiameterMeters / 2; // Wheel radius (meters)
    private final double mr = 60, mw = 0.1; // Robot, wheel mass (kg)
    private final double It = 0.01, Ir = 0.01, IR = 5; // Wheel turning, driving inertia, robot inertia
    private final double a = DriveConstants.kWheelBase / 2, b = DriveConstants.kTrackWidth / 2; // Distance from center to front, side (m)
    private final double k = 200; // Smoothing constant
    private double[] stuck = {1, 1, 1, 1}; // Defines how much a wheel can spin (0=no spin)
    private final double mu_k = 1; // Coefficient of kinetic friction
    private double kVD, kAD; // Motor drive stats
    private double kVT, kAT; // Motor turning stats

    private Vector<N22> state = new Vector<N22>(N22.instance);
    private Vector<N8> input = new Vector<N8>(N8.instance);

    public SwerveSim(DCMotor drivingMotor, DCMotor turningMotor, double drivingG, double turningG) {
        this.kVD = -pow(drivingG, 2) * drivingMotor.KtNMPerAmp / (drivingMotor.KvRadPerSecPerVolt * drivingMotor.rOhms);
        this.kVT = -pow(turningG, 2) * turningMotor.KtNMPerAmp / (turningMotor.KvRadPerSecPerVolt * turningMotor.rOhms);

        this.kAD = drivingG * drivingMotor.KtNMPerAmp / drivingMotor.rOhms;
        this.kAT = turningG * turningMotor.KtNMPerAmp / turningMotor.rOhms;
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
        if (Double.isNaN(x)) {
            return 1;
        } else if (x == Double.NEGATIVE_INFINITY) {
            return -9999999.0;
        } else if (x == Double.POSITIVE_INFINITY) {
            return 9999999.0;
        } else {
            return x;
        }
    }

    private double smooth(double x) {
        return Math.min(Math.abs(x * k), 1);
    }

    private double cof(double x) {
        return -0.8 * 2 / PI * Math.atan(3 * x) + 1.5;
    }


    // private double cof(double v)

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
        double x60 = pow(It, -2);

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
            -stuck[0]*x58*(x33*x59*(x30*(x0*x27 - x24*x5) - x31*(x0*x24 + x27*x5)) - x58*(input.get(0, 0)*kAD + dq3*kVD)),
            -stuck[1]*x58*(x13*x59*(-x11*(x0*x7 + x2*x5) + x9*(x0*x2 - x5*x7)) - x58*(input.get(1, 0)*kAD + dq4*kVD)),
            -stuck[2]*x58*(x23*x59*(x19*(x0*x14 - x17*x5) - x21*(x0*x17 + x14*x5)) - x58*(input.get(2, 0)*kAD + dq5*kVD)),
            -stuck[3]*x58*(x43*x59*(x39*(x0*x37 - x34*x5) - x40*(x0*x34 + x37*x5)) - x58*(input.get(3, 0)*kAD + dq6*kVD)),
            x60*(input.get(4, 0)*kAT + dq7*kVT),
            x60*(input.get(5, 0)*kAT + dq8*kVT),
            x60*(input.get(6, 0)*kAT + dq9*kVT),
            x60*(input.get(7, 0)*kAT + dq10*kVT)
        );
    }

    Matrix<N22, N1> newton(Function<Matrix<N22, N1>, Matrix<N22, N1>> f, Matrix<N22, N1> x) {
        var jacobian = NumericalJacobian.numericalJacobian(N22.instance, N22.instance, f, x);

        var diff = jacobian.solve(f.apply(x).times(-0.1));

        return x.plus(diff);
    }

    void update() {
        Function<Matrix<N22, N1>, Matrix<N22, N1>> f = (q) -> q.minus(dqdt(q, this.input).times(dt)).minus((Matrix<N22, N1>)this.state);

        double error = 999.0;
        Vector<N22> next = state;
        int i = 0;

        while (Math.abs(error) > 1e-10 && i < 1000) {
            var update = new Vector<N22>(newton(f, next));
            // double nextErr = f.apply(next).mean();
            // System.out.println(i);
            // System.out.println(f.apply(update));
            // System.out.println(update.minus(next));
            next = update;
            error = new Vector<N22>(f.apply(next)).norm();
            i++;
        }

        Logger.recordOutput("Model/Error", error);
        Logger.recordOutput("Model/Iterations", i);
        Logger.recordOutput("Inputs", input.getData());
        Logger.recordOutput("State", state.getData());

        state = next;
    }
}

final class N22 extends Num implements Nat<N22> {
    private N22() {}

    @Override
    public int getNum() {
        return 22;
    }

    public static final N22 instance = new N22();
}   