package frc.robot.subsystems.scoring.intake;

    import static edu.wpi.first.units.Units.Meters;
    
    import java.util.ArrayDeque;
    import java.util.Objects;
    import java.util.Queue;
    
    import org.dyn4j.collision.CollisionBody;
    import org.dyn4j.collision.Fixture;
    import org.dyn4j.dynamics.Body;
    import org.dyn4j.dynamics.BodyFixture;
    import org.dyn4j.dynamics.contact.Contact;
    import org.dyn4j.dynamics.contact.SolvedContact;
    import org.dyn4j.geometry.Convex;
    import org.dyn4j.geometry.Rectangle;
    import org.dyn4j.geometry.Vector2;
    import org.dyn4j.world.ContactCollisionData;
    import org.dyn4j.world.listener.ContactListener;
    import org.ironmaple.simulation.SimulatedArena;
    import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
    import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
    import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
    
    import edu.wpi.first.math.MathUtil;
    import edu.wpi.first.math.system.plant.DCMotor;
    import edu.wpi.first.math.system.plant.LinearSystemId;
    import edu.wpi.first.units.measure.Distance;
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.simulation.DCMotorSim;
    import frc.robot.Constants;
    
    // should be the working file for intake simulation
    public class IntakeSimulation extends BodyFixture implements IntakeIO {
        // private final Distance width;
        // private final Distance extendyLength;
        
        // change coral to algae and vice versa whenever :D
        private final String targetedGamePieceType;
        private final int capacity;
        private int gamePiecesInsideIntake;
        private final AbstractDriveTrainSimulation driveTrainSim;
        
        // make a file for "GamePieceOnFieldSim" if needed
        private final Queue<GamePieceOnFieldSimulation> gamePiecesToRemove;
        private boolean intakeRunning;
    
        private double appliedVolts = 0.0;

        private final DCMotorSim sim = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getFalcon500(1), 
                    18.0 / 12.0,
                    0.001), 
                DCMotor.getFalcon500(1));
    
        public enum IntakeSide {
            FRONT,
            // LEFT,
            // RIGHT,
            BACK
        }
       
    // TODO: Find actual width of intake and extension of intake over bumper    
            
        public static IntakeSimulation OverTheBumperIntake (
            String targetedGamePieceType,
            AbstractDriveTrainSimulation driveTrainSim,
            Distance width,
            Distance extendyLength,
            IntakeSide intakeside,
            int capacity
        ) {
            return new IntakeSimulation(
                targetedGamePieceType,
                driveTrainSim,
                getIntakeRectangle(driveTrainSim, width.in(Meters), extendyLength.in(Meters), intakeside),
                capacity);
        }
    
        private static Rectangle getIntakeRectangle(AbstractDriveTrainSimulation driveTrainSim, double width, double extendyLength, IntakeSide intakeside) {
            final Rectangle rectangleyIntake = new Rectangle(width, extendyLength);
            rectangleyIntake.rotate (
                switch (intakeside) {
                    // case LEFT, RIGHT -> 0;
                    case FRONT, BACK -> Math.toRadians(90);
                });
        
                //TODO: fix/tune transformed distance
            final double transformedDistance = extendyLength / 2 - 0.01;
    
            rectangleyIntake.translate (
                switch (intakeside) {
                    // case LEFT -> new Vector2(0, driveTrainSim.config.bumperWidthY.in(Meters) / 2 + transformedDistance);
                    // case RIGHT -> new Vector2(0, -driveTrainSim.config.bumperWidthY.in(Meters) / 2 - transformedDistance);
                    case FRONT -> new Vector2(driveTrainSim.config.bumperLengthX.in(Meters) / 2 + transformedDistance, 0);
                    case BACK -> new Vector2(-driveTrainSim.config.bumperLengthX.in(Meters) / 2 - transformedDistance / 2 , 0);
                });
    
            return rectangleyIntake;
            }
    
            // add/remove return type if necessary
            public IntakeSimulation (
                String targetedGamePieceType, 
                AbstractDriveTrainSimulation driveTrainSim, 
                Convex shape, 
                int capacity) {
    
                super(shape);
                super.setDensity(0);
    
                this.targetedGamePieceType = targetedGamePieceType;
                targetedGamePieceType = "Coral";
                this.gamePiecesInsideIntake = 0;
    
                // change max capacity or error message later if needed (LEO NOT ALLOWED TO CHANGE)
                if (capacity > 100) throw new IllegalArgumentException("no more, stop being big like Leo (max 100)");
                this.capacity = capacity;
    
                this.gamePiecesToRemove = new ArrayDeque<>(capacity);
    
                this.intakeRunning = false;
                this.driveTrainSim = driveTrainSim;
    
                // register();
            }
    
            // may have several redundant commands/functions
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
    
            @Override
            public void runVolts(double volts) {
                appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
                sim.setInputVoltage(appliedVolts);
            }
    
            @Override
            public void stop() {
                runVolts(0.0);
            }
    
            // no work yet cuz sim messed up rn
            public void updateInputs(IntakeIOInputs inputs) {
            if(DriverStation.isDisabled())
                runVolts(0.0);
    
            sim.update(Constants.robot.loopPeriodSecs);
            inputs.appliedVolts = appliedVolts;
            inputs.posRads = sim.getAngularPositionRad();
            inputs.velRadsPerSec = sim.getAngularVelocityRadPerSec();
            }   
    
            public final class GamePieceContactListener implements ContactListener<Body> {
                
                private void indicateGamePieceRemoval(GamePieceOnFieldSimulation gamePiece) {
                    gamePiecesToRemove.add(gamePiece);
                    gamePiecesInsideIntake++;
                }
    
                @Override
                public void begin(ContactCollisionData collision, Contact contact) {
                    if (!intakeRunning) return;
                    if (gamePiecesInsideIntake >= capacity) return;
    
                    final CollisionBody<?> collisionBody1 = collision.getBody1(), collisionBody2 = collision.getBody2();
                    final Fixture fixture1 = collision.getFixture1(), fixture2 = collision.getFixture2();
    
                    if (collisionBody1 instanceof GamePieceOnFieldSimulation gamePiece
                            && Objects.equals(gamePiece.type, targetedGamePieceType)
                            && fixture2 == IntakeSimulation.this) indicateGamePieceRemoval(gamePiece);
                    else if (collisionBody2 instanceof GamePieceOnFieldSimulation gamePiece
                            && Objects.equals(gamePiece.type, targetedGamePieceType)
                            && fixture1 == IntakeSimulation.this) indicateGamePieceRemoval(gamePiece);
    
                    boolean coralOrAlgaeIntake = "Coral".equals(IntakeSimulation.this.targetedGamePieceType)
                            || "Algae".equals(IntakeSimulation.this.targetedGamePieceType);
                    if (collisionBody1 instanceof ReefscapeCoralAlgaeStack stack
                            && coralOrAlgaeIntake
                            && fixture2 == IntakeSimulation.this) indicateGamePieceRemoval(stack);
                    else if (collisionBody2 instanceof ReefscapeCoralAlgaeStack stack
                            && coralOrAlgaeIntake
                            && fixture1 == IntakeSimulation.this) indicateGamePieceRemoval(stack);
                }
                
                @Override
                public void persist(ContactCollisionData collision, Contact oldContact, Contact newContact) {}
        
                @Override
                public void end(ContactCollisionData collision, Contact contact) {}
        
                @Override
                public void destroyed(ContactCollisionData collision, Contact contact) {}
        
                @Override
                public void collision(ContactCollisionData collision) {}
        
                @Override
                public void preSolve(ContactCollisionData collision, Contact contact) {}
        
                @Override
                public void postSolve(ContactCollisionData collision, SolvedContact contact) {}
            }
    
            public GamePieceContactListener getGamePieceContactListener() {
                return new GamePieceContactListener();
            }
    
            public void removeObtainedGamePieces(SimulatedArena arena) {
                while (!gamePiecesToRemove.isEmpty()) {
                    GamePieceOnFieldSimulation gamePiece = gamePiecesToRemove.poll();
                    gamePiece.onIntake(this.targetedGamePieceType);
                    arena.removeGamePiece(gamePiece);
                }
            }
    
    
        /*
         * don't need register since it automatically registers the intake
         * in the intakesimulation constructor (which is now finished yay)
         * (only problem is it technically isn't io)
         */
            // public void register() {
            //     register(SimulatedArena.getInstance());
            // }
    
            // public void register(SimulatedArena arena) {
            //     arena.addIntakeSimulation(this);
            // }
        }
    
    