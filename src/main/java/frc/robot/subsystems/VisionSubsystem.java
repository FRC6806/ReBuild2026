// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.RawFiducial;

// public class VisionSubsystem extends SubsystemBase {
//   private RawFiducial[] fiducials;

//   public VisionSubsystem() {
//     config();
//   }

//   public static class NoSuchTargetException extends RuntimeException {
//     public NoSuchTargetException(String message) {
//       super(message);
//     }
//   }

//   public void config() {

//     // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
//     LimelightHelpers.setCameraPose_RobotSpace(
//         "",
//         0.2286, 
//         0.0762,
//         0.09525,
//         0,
//         0,
//         0);
//     LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] {0,1,5,8,9,10,11,12}); //CHANGE WHEN WE FIGURE OUT WHAT WE USE
//   }

//   @Override
//   public void periodic() {
//     fiducials = LimelightHelpers.getRawFiducials("");

//   }
//   public RawFiducial getClosestFiducial() {
//     if (fiducials == null || fiducials.length == 0) {
//         throw new NoSuchTargetException("No fiducials found.");


//     }

//     RawFiducial closest = fiducials[0];
//     double minDistance = closest.ta;

//     for (RawFiducial fiducial : fiducials) {
//         if (fiducial.ta > minDistance) {
//             closest = fiducial;
//             minDistance = fiducial.ta;
//         }
//     }

//     return closest;
//   }

//   public RawFiducial getFiducialWithId(int id) {
  
//     for (RawFiducial fiducial : fiducials) {
//         if (fiducial.id == id) {
//             return fiducial;
//         }
//     }
//     throw new NoSuchTargetException("Can't find ID: " + id);
//   }

// public RawFiducial getFiducialWithId(int id, boolean verbose) {
//   StringBuilder availableIds = new StringBuilder();

//   for (RawFiducial fiducial : fiducials) {
//       if (availableIds.length() > 0) {
//           availableIds.append(", ");
//       } //Error reporting
//       availableIds.append(fiducial.id);
      
//       if (fiducial.id == id) {
//           return fiducial;
//       }
//   }
//   throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
//   }

//   public double getTX(){
//     return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
//   }
//   public double getTY(){
//     return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
//   }
//   public double getTA(){
//     return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
//   }
//   public boolean getTV(){
//     return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
//   }

//   public double getClosestTX(){
//     try {
//         return getClosestFiducial().txnc;
//     }catch (Exception e) {
//         return 0;
//     } 
//   }
//   public double getClosestTY(){
//     try {
//         return getClosestFiducial().tync;
//     } catch (Exception e) {
//         return 0.0;
//     }
    
//   }
//   public double getClosestTA(){
//     try {
//         return getClosestFiducial().ta;
//     } catch (Exception e) {
//         return 0;
//     }
//   }
// }