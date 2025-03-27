# Train Controller API

## Overview
Train Controller API is a modular system designed for simulating and controlling train operations. It provides functionalities for train movement, track management, and user interactions.
## Demonstration
- https://drive.google.com/file/d/1ETpjapZyQALAHngoJ9QEBcOJEs20-8PK/view?usp=sharing

- https://drive.google.com/file/d/18L-WSaSLDKaRD1rc7efv6fpwWKA2crPs/view?usp=sharing

## Features
- **Train Movement**: Realistic acceleration, deceleration, and braking mechanics.
- **Track Switching**: Dynamic track switching with collision detection.
- **Signal System**: Integrated signaling for train routing and stopping conditions.
- **Extensibility**: Modular components for easy customization.

## Components
### TrackParameters
Handles track logic and calculating curve position.
- **Track limits**: Limits train position between 2 points (from 0 to 1).
- **Control points**: Caluclates track curve within an array of points.
 ![](https://github.com/Wanna-Be-Dev/VRIF_Train/blob/master/README/BezierCurve.gif)  
- **Triggers**: Triggers events when train reaches points
- **Gizmos**: Enables visual representation of diffrent track settings.

## API Methods
### `TrainController.SwitchTrack(int trackID)`
Changes the track path of a train.
- **Parameters**: `trackID` (int) - The ID of the track to switch to.
- **Example**:
    ```csharp
   TrainController.SwitchTrack(3);
    ```
 ![](https://github.com/Wanna-Be-Dev/VRIF_Train/blob/master/README/TrackSwitch.gif)
### `TrainController.SetPosition(float position)`
Set train position on the track percentile from 0 to 1.
- **Parameters**: `position` (float) - Train position.
- **Example**:
    ```csharp
    TrainController.SetPosition(0.4f);
    ```
![](https://github.com/Wanna-Be-Dev/VRIF_Train/blob/master/README/SetPosition.gif)
### `TrainController.SetDirection(float angle)`
Set train direction -1f or 1f.
- **Parameters**:
  - `angle` (float) - Set train movement position.
- **Example**:
    ```csharp
    TrainController.SetDirection(-1f);
    ```
![](https://github.com/Wanna-Be-Dev/VRIF_Train/blob/master/README/SwitchDirection.gif)
