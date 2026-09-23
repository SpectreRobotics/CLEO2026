# AdvantageScope 3D Robot CAD Setup for CLEO 2026

This directory contains the custom 3D asset configuration for AdvantageScope's **3D Field** visualization.

## Files in this Folder

- **`config.json`**: Configures the robot name and articulated moving parts:
  - **Component 0**: Turret (rotates in 3D based on auto-targeting / flywheel aim)
  - **Component 1**: Intake Slide (moves forward/back in 3D)
- **`model.glb`** *(Add your file here)*: The 3D model of your robot chassis / drivetrain.
- **Optional moving parts**:
  - `turret.glb`: 3D model of the turret mechanism.
  - `slide.glb`: 3D model of the sliding intake.

---

## How to Export your CAD to .glb

AdvantageScope requires the binary glTF (`.glb`) format.

### Option A: From Onshape
1. Open your robot assembly in Onshape.
2. Right-click the assembly tab → **Export**.
3. Choose format: **glTF** (binary `.glb`).
4. Set units to **Meters**.
5. Save the chassis as `model.glb`.
6. (Optional) Export the turret and slide subassemblies separately as `.glb` files.

### Option B: From SolidWorks / Inventor
1. Export your robot assembly as a **STEP** file (`.step` or `.stp`).
2. Download the free tool **CAD Assistant** (by Open CASCADE) or **Blender**.
3. Open the STEP file in CAD Assistant and choose **Save As → glTF / glb**.
4. Rename the output file to `model.glb`.

---

## How to Load into AdvantageScope

### Method 1: Point AdvantageScope to this repo (Easiest)
1. In AdvantageScope, click menu: **File → Use Custom Assets Folder...** (or **App → Use Custom Assets Folder...** on macOS/Linux).
2. Select the `c:\Users\rkuzmik\Documents\FRC2026\Cleoround2\assets` folder in this repository.
3. Restart AdvantageScope or switch tabs.
4. In the **3D Field** tab, click the robot dropdown and select **CLEO 2026**!

### Method 2: Copy to Default Assets Folder
1. In AdvantageScope, click **Help → Show Assets Folder** (or **App → Show Assets Folder**).
2. Copy this entire `Robot_CLEO` folder into that assets folder.
3. In the 3D Field tab, select **CLEO 2026** from the robot model list.

---

## Telemetry Fields Published by Robot Code

The robot code publishes the following 3D telemetry:
- `/SmartDashboard/RobotPose3d`: The 3D position and rotation of the chassis on the field.
- `/SmartDashboard/ComponentPoses`: An array of `Pose3d` structs:
  - `[0]`: Turret azimuth rotation around the Z axis.
  - `[1]`: Intake slide position along the X axis.
