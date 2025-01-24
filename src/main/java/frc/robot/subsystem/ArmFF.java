package frc.robot.subsystem;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;

//https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
public class ArmFF {
    //gravityy !!!! 
    static final double g = 9.80;

    //gearbox of da 2 motors
    DCMotor arm = DCMotor.getKrakenX60(1);
    DCMotor wrist = DCMotor.getNeo550(1);

    //FF model that convert betw voltage and joing pos, vel, and accel
    
    //inertia matrix * acceleration 
    // + centrifugal and Coiolis terms (dont ask me wtf is that) * velocities 
    // + torque applied due to gravity
    // = net torque at each joint

    //dcmotor can calculate voltage based on torque and target velocity
    public double getArmVoltFF(Vector<N2> pos){
        return calculate(pos).get(0,0);
    }

    public double getWristVoltFF(Vector<N2> pos){
        return calculate(pos).get(1,0);
    }

    public double getArmVoltFF(Vector<N2> pos, Vector<N2> vel, Vector<N2> accel){
        return calculate(pos, vel, accel).get(0,0);
    }

    public double getWristVoltFF(Vector<N2> pos, Vector<N2> vel, Vector<N2> accel){
        return calculate(pos, vel, accel).get(1,0);
    }

    public Vector<N2> calculate(Vector<N2> pos){
        return calculate(pos, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0,0.0));
    }
    
    public Vector<N2> calculate(Vector<N2> pos, Vector<N2> vel, Vector<N2> accel){
        var t = M(pos)
            .times(accel)
            .plus(C(pos, vel).times(vel))
            .plus(T(pos));

    return VecBuilder.fill(
        arm.getVoltage(t.get(0, 0), vel.get(0, 0)),
        wrist.getVoltage(t.get(1, 0), vel.get(1, 0)));
    }

    public record Joint(
        double mass,
        double length,
        double inertiaAbtCoM,
        double disFromPivot2CoG
    ){}

    //state of system is a matrix, with top row is arm, bottom is wrist

    

    Joint armConfig;
    Joint wristConfig;

    //1 = angle from horizontal to arm, 2 = angle from arm to wrist


    //inertia matrix calculation
    private Matrix<N2, N2> M(Vector<N2> pos){
        var m = new Matrix<>(N2.instance, N2.instance);

        //set value at 0,0 of matrix
        m.set(
            0,
            0,
            armConfig.mass() * Math.pow(armConfig.disFromPivot2CoG(), 2.0)
                + wristConfig.mass() * (Math.pow(armConfig.length(), 2.0) + Math.pow(wristConfig.disFromPivot2CoG(), 2.0))
                + armConfig.inertiaAbtCoM()
                + wristConfig.inertiaAbtCoM()
                + 2
                    * wristConfig.mass()
                    * armConfig.length()
                    * wristConfig.disFromPivot2CoG()
                    * Math.cos(pos.get(1,0)));

        //set value at 1,0 of matrix 
        m.set(
            0,
            1,
            wristConfig.mass() * Math.pow(wristConfig.disFromPivot2CoG, 2)
                + wristConfig.inertiaAbtCoM
                + wristConfig.mass * armConfig.length * wristConfig.disFromPivot2CoG * Math.cos(pos.get(1,0)));

        m.set(
            1,
            0,
            wristConfig.mass() * Math.pow(wristConfig.disFromPivot2CoG, 2)
            + wristConfig.inertiaAbtCoM
            + wristConfig.mass * armConfig.length * wristConfig.disFromPivot2CoG * Math.cos(pos.get(1,0)));

        m.set(
            1,
            1,
            wristConfig.mass * Math.pow(wristConfig.disFromPivot2CoG, 2)
            + wristConfig.inertiaAbtCoM);

        return m;
    }

    private Matrix<N2, N2> C(Vector<N2> velocity, Vector<N2> pos){
        Matrix<N2, N2> c = new Matrix<>(N2.instance, N2.instance);

        c.set(
            0, 
            0, 
            - wristConfig.mass
             * armConfig.length 
             * armConfig.disFromPivot2CoG 
             * Math.sin(pos.get(1,0))
             * velocity.get(1,0));
        
        c.set(
            0,
            1,
            - wristConfig.mass
            * armConfig.length
            * armConfig.disFromPivot2CoG
            * Math.sin(pos.get(1,0))
            * (velocity.get(1, 0) + velocity.get(0,0))
        );

        c.set(
            1,
            0,
            wristConfig.mass
            * armConfig.length
            * wristConfig.disFromPivot2CoG
            * Math.sin(pos.get(1,0))
            * velocity.get(0, 0));
        
        c.set(1, 1, 0);

        return c;
    }

    private Matrix<N2, N1> T(Vector<N2> pos){
        Matrix<N2, N1> t = new Matrix<>(N2.instance, N1.instance);

        t.set(
            0,
            0,
            (armConfig.mass * armConfig.disFromPivot2CoG 
                + wristConfig.mass * armConfig.length)
            * g * Math.cos(pos.get(0,0))
            + wristConfig.mass
            * wristConfig.disFromPivot2CoG
            * g
            * Math.cos(pos.get(1,0) + pos.get(0,0))
        );

        t.set(
            1,
            0,
            wristConfig.mass
            * wristConfig.disFromPivot2CoG
            * g
            * Math.cos(pos.get(1,0) + pos.get(0,0)));
        
        return t;
    }

    public static Matrix<N2, N2> plus(Matrix<N2, N2> e, Matrix<N2, N2> a){
        Matrix<N2, N2> result = new Matrix<>(N2.instance, N2.instance);

        result.set(
            0,
            0,
            e.get(0, 0) + e.get(0, 0));
        
        result.set(
            0,
            1,
            e.get(0, 1) + e.get(0, 1));

        result.set(
            1,
            0, 
            e.get(1, 0) + e.get(1, 0));
        
        result.set(
            1,
            1, 
            e.get(1, 1) + e.get(1, 1));
        return result;
    }
    
}
