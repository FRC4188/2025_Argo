package frc.robot.subsystems.scoring.intake;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayDeque;
import java.util.Queue;

import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class IntakeIOSim implements IntakeIO {
    // private final Distance width;
    // private final Distance extendyLength;
    
    // change coral to algae and vice versa whenever :D
    private final String targetedGamePieceType = "Coral";
    private final int capacity;
    private final int gamePiecesInsideIntake;
    private final AbstractDriveTrainSimulation driveTrainSim;
    
    // make a file for "GamePieceOnFieldSim" if needed
    private final Queue<GamePieceOnFieldSim> gamePiecesToRemove;
    private boolean intakeRunning;

    public enum IntakeSide {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }

// TODO: Find actual width of intake and extension of intake over bumper    
    // private final IntakeSimulation IntakeSimulation;
    // public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    //     this.IntakeSimulation = IntakeSimulation.OverTheBumperIntake(
    //         targetedGamePieceType, 
    //         driveTrain,
    //         width,
    //         extendyLength,
    //         IntakeSimulation.IntakeSide.BACK,
    //         1
    //     );  
        
    public static IntakeSimulation OverTheBumperIntake (
        String targetedGamePieceType,
        AbstractDriveTrainSimulation driveTrainSim,
        Distance width,
        Distance extendyLength,
        IntakeSide intakeside,
        int capacity
    ) {
        return new IntakeSimulation (
            targetedGamePieceType,
            driveTrainSim,
            getIntakeRectangle(driveTrainSim, width.in(Meters), extendyLength.in(Meters), intakeside),
            capacity);
    }

    private static Rectangle getIntakeRectangle(AbstractDriveTrainSimulation driveTrainSim, double width, double extendyLength, IntakeSide intakeside) {
        final Rectangle rectangleyIntake = new Rectangle(width, extendyLength);
        rectangleyIntake.rotate (
            switch (intakeside) {
                case LEFT, RIGHT -> 0;
                case FRONT, BACK -> Math.toRadians(90);
            });
    
            //TODO: fix/tune transformed distance
        final double transformedDistance = extendyLength / 2 - 0.01;

        rectangleyIntake.translate (
            switch (intakeside) {
                case LEFT -> new Vector2(0, driveTrainSim.config.bumperWidthY.in(Meters) / 2 + transformedDistance);
                case RIGHT -> new Vector2(0, -driveTrainSim.config.bumperWidthY.in(Meters) / 2 - transformedDistance);
                case FRONT -> new Vector2(driveTrainSim.config.bumperLengthX.in(Meters) / 2 + transformedDistance, 0);
                case BACK -> new Vector2(-driveTrainSim.config.bumperLengthX.in(Meters) / 2 - transformedDistance / 2 , 0);
            });

        return rectangleyIntake;
        }

        // remove void if necessary
        public IntakeSimulation(String targetedGamePieceType, AbstractDriveTrainSimulation driveTrainSim, Convex shape, int capacity) {
            super(shape);
            super.setDensity(0);

            this.targetedGamePieceType = targetedGamePieceType;
            this.gamePiecesInsideIntake = 0;

            if (capacity > 1) throw new IllegalArgumentException("max capacity is 1 per game piece");
            this.capacity = capacity;

            this.gamePiecesToRemove = new ArrayDeque<>(capacity);

            this.intakeRunning = false;
            this.driveTrainSim = driveTrainSim;

            // register();
        }

        public void runIntake() {
            if (intakeRunning) return;

            driveTrainSim.addFixture(this);
            this.intakeRunning = true;
        }

        public void stopIntake() {
            if (!intakeRunning) return;
            
            driveTrainSim.removeFixture(this);
            this.intakeRunning = false;
        }

        public int getHowManyGamePieces() {
            return gamePiecesInsideIntake;
        }

        public boolean obtainGamePieceFromIntake() {
            if (gamePiecesInsideIntake < 1) return false;
            gamePiecesInsideIntake--;
            return true;
        }



        // TODO: need to add intake sim to field if not already done
        // public void register() {
        //     register(SimulatedArena.getInstance()):
        // }

        // public void register(SimulatedArena arena) {
        //     arena.addIntakeSimulation(this);
        // }
}

