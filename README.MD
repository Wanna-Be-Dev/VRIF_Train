# Train Controller API

## Overview
Train Controller API is a modular system designed for simulating and controlling train operations within a game engine. It provides functionalities for train movement, track management, and user interactions.

## Features
- **Train Movement**: Realistic acceleration, deceleration, and braking mechanics.
- **Track Switching**: Dynamic track switching with collision detection.
- **Signal System**: Integrated signaling for train routing and stopping conditions.
- **Multiplayer Support**: Synchronization for online multiplayer gameplay.
- **Extensibility**: Modular components for easy customization.

## Components
### TrainController
Manages the core train movement and physics.
- **Speed Control**: Adjusts train velocity.
- **Braking System**: Implements friction-based stopping.
- **Direction Handling**: Enables forward and reverse movement.

### TrackManager
Handles track logic and train positioning.
- **Track Switching**: Routes trains dynamically.
- **Track Segments**: Manages individual track pieces.
- **Junctions**: Connects multiple tracks.

### SignalManager
Controls railway signals and permissions.
- **Red/Green Lights**: Dictates train movement.
- **Collision Prevention**: Prevents trains from overlapping.
- **Automated Routing**: Directs trains to available tracks.

## API Methods
### `TrainController.SetSpeed(float speed)`
Sets the speed of the train.
- **Parameters**: `speed` (float) - Desired speed in meters per second.
- **Example**:
    ```csharp
    TrainController.SetSpeed(10.5f);
    ```

### `TrainController.Brake()`
Applies brakes to stop the train.
- **Example**:
    ```csharp
    TrainController.Brake();
    ```

### `TrackManager.SwitchTrack(int trackID)`
Changes the track path of a train.
- **Parameters**: `trackID` (int) - The ID of the track to switch to.
- **Example**:
    ```csharp
    TrackManager.SwitchTrack(3);
    ```

### `SignalManager.SetSignal(int signalID, bool state)`
Sets the signal light state.
- **Parameters**:
  - `signalID` (int) - ID of the signal.
  - `state` (bool) - `true` for green, `false` for red.
- **Example**:
    ```csharp
    SignalManager.SetSignal(1, true);
    ```

## Future Improvements
- **AI-Controlled Trains**: Automated train behavior and traffic management.
- **Expanded Multiplayer Features**: Improved synchronization and networking.
- **Advanced Physics Simulation**: More realistic train dynamics.
- **Customizable Tracks**: User-generated track layouts.