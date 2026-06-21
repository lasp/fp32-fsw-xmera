#ifndef sunSafePointFaultMsg_h
#define sunSafePointFaultMsg_h

/*! @brief Structure used to define the sunSafePoint search-failure fault message */
typedef struct {
    bool faultDetected;  //!< true once the sun search fails to acquire the sun and forces pointing
} SunSafePointFaultMsgPayload;

#endif /* sunSafePointFaultMsg_h */
