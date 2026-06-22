#ifndef sunSearchPointFaultMsg_h
#define sunSearchPointFaultMsg_h

/*! @brief Structure used to define the sunSearchPoint search-failure fault message */
typedef struct {
    bool faultDetected;  //!< true once the sun search fails to acquire the sun and forces pointing
} SunSearchPointFaultMsgPayload;

#endif /* sunSearchPointFaultMsg_h */
