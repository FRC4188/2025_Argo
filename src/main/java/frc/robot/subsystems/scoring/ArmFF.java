package frc.robot.subsystems.scoring;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.scoring.superstructure.SuperstructureConfig;
import frc.robot.subsystems.scoring.superstructure.SuperstructureConfig.Joint;

//https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
public class ArmFF {
    //gravityy !!!! 
    static final double g = 9.80;

    //gearbox of da 2 motors
    DCMotor arm = DCMotor.getKrakenX60(1);
    DCMotor wrist = DCMotor.getNeo550(1);
    
    Joint armConfig = SuperstructureConfig.arm;
    Joint wristConfig = SuperstructureConfig.wrist;

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



    /**get updated sim state of superstructure based on current states + applied volts on joints
     * @param state in order of (pos1, pos2, velocity1, velocity2)
     * @param volt in order of (applied volts 1, applied volts 2)
     * @param dt
     * @return the antiderivative of vector of (vel1, vel2, accel1, accel2) aka new (pos1, pos2, vel1, vel2)
     */
    public Vector<N4> simState(Vector<N4> state, Vector<N2> volt, double dt){
        return new Vector<>(
            NumericalIntegration.rkdp(
                (Matrix<N4, N1> x, Matrix<N2, N1> u) -> {
                    var pos = VecBuilder.fill(state.get(0, 0), state.get(1,0));
                    var vel = VecBuilder.fill(state.get(2, 0), state.get(3,0));

                    var torque = VecBuilder.fill(
                        arm.getTorque(arm.getCurrent(state.get(2), volt.get(0))),
                        wrist.getTorque(wrist.getCurrent(state.get(3), volt.get(1)))
                    );
                    var accel = M(pos).inv()
                        .times(
                            torque.minus(C(vel, pos).times(vel).plus(T(pos)))
                        );
                
                    return MatBuilder.fill(Nat.N4(), Nat.N1(), 
                        vel.get(0, 0),
                        vel.get(1,0),
                        accel.get(0,0),
                        accel.get(1,0)
                        );
                },
                state, 
                volt,
                dt
            )
        );
    }

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
            wristConfig.mass() * Math.pow(wristConfig.disFromPivot2CoG(), 2)
                + wristConfig.inertiaAbtCoM()
                + wristConfig.mass() * armConfig.length() * wristConfig.disFromPivot2CoG() * Math.cos(pos.get(1,0)));

        m.set(
            1,
            0,
            wristConfig.mass() * Math.pow(wristConfig.disFromPivot2CoG(), 2)
            + wristConfig.inertiaAbtCoM()
            + wristConfig.mass() * armConfig.length() * wristConfig.disFromPivot2CoG() * Math.cos(pos.get(1,0)));

        m.set(
            1,
            1,
            wristConfig.mass() * Math.pow(wristConfig.disFromPivot2CoG(), 2)
            + wristConfig.inertiaAbtCoM());

        return m;
    }

    private Matrix<N2, N2> C(Vector<N2> velocity, Vector<N2> pos){
        Matrix<N2, N2> c = new Matrix<>(N2.instance, N2.instance);
        
        c.set(
            0, 
            0, 
            - wristConfig.mass()
             * armConfig.length() 
             * armConfig.disFromPivot2CoG()
             * Math.sin(pos.get(1,0))
             * velocity.get(1,0));
        
        c.set(
            0,
            1,
            - wristConfig.mass()
            * armConfig.length()
            * armConfig.disFromPivot2CoG()
            * Math.sin(pos.get(1,0))
            * (velocity.get(1, 0) + velocity.get(0,0))
        );

        c.set(
            1,
            0,
            wristConfig.mass()
            * armConfig.length()
            * wristConfig.disFromPivot2CoG()
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
            (armConfig.mass() * armConfig.disFromPivot2CoG()
                + wristConfig.mass() * armConfig.length())
            * g * Math.cos(pos.get(0,0))
            + wristConfig.mass()
            * wristConfig.disFromPivot2CoG()
            * g
            * Math.cos(pos.get(1,0) + pos.get(0,0))
        );

        t.set(
            1,
            0,
            wristConfig.mass()
            * wristConfig.disFromPivot2CoG()
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
