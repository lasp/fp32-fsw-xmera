#ifndef CAMERA_MODEL_MESSAGE_F32_H
#define CAMERA_MODEL_MESSAGE_F32_H

/*! @brief Structure used to define the camera model*/

typedef struct {
    int cameraId;          //!< [-]   ID of the camera that took the snapshot*/
    float fieldOfView[2];  //!< [rad]   Camera Field of View, edge-to-edge along camera y-axis */
    int resolution[2];  //!< [-] Camera resolution, width/height in pixels (pixelWidth/pixelHeight in Unity) in pixels*/
    float bodyToCameraMrp[3];  //!< [-] MRP defining the orientation of the camera frame relative to the body frame */
} CameraModelMsgF32Payload;

#endif
