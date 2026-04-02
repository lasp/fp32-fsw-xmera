#ifndef F32XMERA_SOLAR_ARRAY_REFERENCE_TYPES_H
#define F32XMERA_SOLAR_ARRAY_REFERENCE_TYPES_H

/*! @brief Output of the solar array reference algorithm. */
struct SolarArrayReferenceOutput {
    float theta{};     //!< [rad] reference rotation angle
    float thetaDot{};  //!< [rad/s] reference rotation angle rate
};

#endif  // F32XMERA_SOLAR_ARRAY_REFERENCE_TYPES_H
