# <center>**SN Robot 2023**</center>

![Robot Logo](https://raw.githubusercontent.com/FRCTeam3255/GraphicsYearly/main/Exports/SN%20Logos/Official%20SN%20Logo.png)

<br/>

# <h1 style="color: #ECC412"> Summary </h1>

## **Arm:**
  - ### Double jointed arm mechanism
  - ### Intake mounted onto the arm
  - ### Uses REV through bore encoders to save prevent the need to reset encoders on startup
  - ### Operator can directly move the intake up, down, forwards, or backwards (instead of manually controlling the rotation of the joints)

<br/>

## **Collector:**
  - ### Wide, over the bumper roller to assist in the collection of cubes
  - ### Assists in getting onto the charge station
  - ### Deploys using a motor, instead of pneumatics for easier management

<br/>

## **Drivetrain:**
  - ### Field orientated and robot oriented swerve drive. 
  - ### Contains pose estimation using encoders
  - ### Uses open and closed loop drive motor control.

<br/>

## **Intake:**
  - ### Dual NEO motor control 
  - ### Color sensor that allows for game piece   detection and game piece identification

<br/>

## **LEDs:**
  - ### LEDs that can easily be switched between yellow (cone) and violet (cube) using the switchboard
  - ### LEDs can be used as an indicator to the human player

<br/>

## **Pose Estimation and AprilTags**
  - ### Using a N5095 Beelink Mini PC for our coprocessor
  - ### Two global shutter cameras (AR0144 and OV9281) along with 2.8mm lenses
  - ### Powered by a boost buck converter that accepts an 8-40V input and outputs 12V 3A

<br/>

## **Misc:**
  - ### Optional debug outputs to network tables
  - ### Tunable values on network tables
    - ### Extremely useful for PID tuning
    - ### Locked to hardcoded values by default

<br/>

# <h1 style="color: #ECC412"> Vendor Dependencies </h1>

## Charged Up 2023 Game Links

  - ### [Game And Season Info](https://www.firstinspires.org/robotics/frc/game-and-season)

<br/>

## Falcons

  - ### [Falcon User Manual](https://store.ctr-electronics.com/content/user-manual/Falcon%20500%20User%20Guide.pdf)

  - ### [Falcon Software Documentation](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/using-motor-controllers.html)

  - ### [Falcon Firmware Download 2023](https://github.com/CrossTheRoadElec/Phoenix-Releases/blob/master/ctr-device-firmware.zip?raw=true)

<br/>

## NavX

  - ### [NavX User Manual](https://pdocs.kauailabs.com/navx-mxp/wp-content/uploads/2019/02/navx-mxp_robotics_navigation_sensor_user_guide.pdf)

  - ### [NavX Software Documentation](https://pdocs.kauailabs.com/navx-mxp/software/)

  - ### [NavX Firmware Download 2023](https://www.kauailabs.com/public_files/navx-mxp/navx-mxp.zip)

<br/>

## Limelight

  - ### [Limelight User Manual](https://docs.limelightvision.io/en/latest/getting_started.html#power-over-ethernet-poe-wiring)

  - ### [Limelight Software Documentation](https://docs.limelightvision.io/en/latest/)

  - ### [Limelight Firmware Download 2023.1.0](https://downloads.limelightvision.io/images/limelight2_2023_1.zip)

<br/>

## Power Distribution Panel

  - ### [PDP User Manual](https://store.ctr-electronics.com/content/user-manual/PDP%20User%27s%20Guide.pdf)

  - ### [PDP Software Documentation](https://docs.wpilib.org/en/stable/docs/software/can-devices/power-distribution-module.html)

  - ### [PDP Firmware Download 2023](https://github.com/CrossTheRoadElec/Phoenix-Releases/blob/master/ctr-device-firmware.zip?raw=true)

  <br/>

## Pneumatic Control Module

  - ### [PCM User Manual](https://store.ctr-electronics.com/content/user-manual/PCM%20User%27s%20Guide.pdf)

  - ### [PCM Software Documentation](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html)

  - ### [PCM Firmware Download 2023](https://github.com/CrossTheRoadElec/Phoenix-Releases/blob/master/ctr-device-firmware.zip?raw=true)