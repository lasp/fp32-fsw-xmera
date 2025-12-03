#ifndef mimuFaultMsg_h
#define mimuFaultMsg_h

#include <cstdint>

/*! @brief Structure used to define the mimu fault message */
typedef struct {
    bool faultDetected;    //!< fault detected bool
    int mimuIndexFaulted;  //!< mimu faulted index
} MimuFaultMsgPayload;

#endif /* mimuFaultMsg_h */
