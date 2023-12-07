# Photo Service

This service is designed to connect to a drone and command it to take a photo. It does not include any functionality for video recording.

## Overview

The service uses a state machine to manage the flow of the program. When the mission controller interface gets connected, it sets the state to `PHOTO_CONFIG_DONE` and sends the photo configuration to the drone. Once the drone confirms that it has switched to photo mode, the program sets the state to `PHOTO_SHOOT_DONE` and sends the command to take a photo. The mission ends once the drone confirms that it has taken a photo.

## Changes Made

The following changes were made to the original `video_photo_cpp` service:

1. Removed the `RECORDING_CONFIG_DONE`, `RECORDING_STARTED`, and `RECORDING_STOPPED` states from the `VideoPhotoStateMachine` enum.

2. Modified the `onConnected` function to set the state to `PHOTO_CONFIG_DONE` and send the photo configuration when the mission controller interface gets connected.

3. Modified the `reactInSmToCameraMode` function to handle only the `CAMERA_MODE_PHOTO` case.

4. Removed the `cmdFcamStartRecording` and `cmdFcamSetConfigRecording` functions.

5. Removed other unused code.