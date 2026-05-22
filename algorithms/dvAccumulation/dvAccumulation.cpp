#include "dvAccumulation.h"
#include <architecture/utilities/macroDefinitions.h>
#include <stdexcept>

/* Experimenting QuickSort START */
static void dvAccumulation_swap(AccPktDataMsgF32Payload* p, AccPktDataMsgF32Payload* q) {
    AccPktDataMsgF32Payload t;
    t = *p;
    *p = *q;
    *q = t;
}

static int dvAccumulation_partition(AccPktDataMsgF32Payload* A, int start, int end) {
    int i;
    uint64_t pivot = A[end].measTime;
    int partitionIndex = start;
    for (i = start; i < end; i++) {
        if (A[i].measTime <= pivot) {
            dvAccumulation_swap(&(A[i]), &(A[partitionIndex]));
            partitionIndex++;
        }
    }
    dvAccumulation_swap(&(A[partitionIndex]), &(A[end]));
    return partitionIndex;
}

/*! Sort the AccPktDataMsgPaylaod by the measTime with an iterative quickSort.
  @return void
  @param A --> Array to be sorted,
  @param start  --> Starting index,
  @param end  --> Ending index */
void dvAccumulation_QuickSort(AccPktDataMsgF32Payload* A, int start, int end) {
    /*! - Create an auxiliary stack array. This contains indicies. */
    int stack[MAX_ACC_BUF_PKT];

    /*! - initialize the index of the top of the stack */
    int top = -1;

    /*! - push initial values of l and h to stack */
    stack[++top] = start;
    stack[++top] = end;

    /*! - Keep popping from stack while is not empty */
    while (top >= 0) {
        /* Pop h and l */
        end = stack[top--];
        start = stack[top--];

        /*! - Set pivot element at its correct position in sorted array */
        int partitionIndex = dvAccumulation_partition(A, start, end);

        /*! - If there are elements on left side of pivot, then push left side to stack */
        if (partitionIndex - 1 > start) {
            stack[++top] = start;
            stack[++top] = partitionIndex - 1;
        }

        /*! - If there are elements on right side of pivot, then push right side to stack */
        if (partitionIndex + 1 < end) {
            stack[++top] = partitionIndex + 1;
            stack[++top] = end;
        }
    }
}
/* Experimenting QuickSort END */

void DVAccumulation::reset(uint64_t callTime) {
    // check if the required message has not been connected
    if (!this->accPktInMsg.isLinked()) {
        throw std::invalid_argument("dvAccumulation.accPktInMsg wasn't connected.");
    }

    /*! - read in the accelerometer data message */
    AccDataMsgF32Payload inputAccData = this->accPktInMsg();

    /*! - stacks data in time order*/
    dvAccumulation_QuickSort(&(inputAccData.accPkts[0]), 0, MAX_ACC_BUF_PKT - 1);

    /*! - reset accumulated DV vector to zero */
    this->vehAccumDV_B[0] = 0.0;
    this->vehAccumDV_B[1] = 0.0;
    this->vehAccumDV_B[2] = 0.0;

    /*! - reset previous time value to zero */
    this->previousTime = 0;

    /* - reset initialization flag */
    this->dvInitialized = 0;

    /*! - If we find valid timestamp, ensure that no "older" meas get ingested*/
    for (int i = (MAX_ACC_BUF_PKT - 1); i >= 0; i--) {
        if (inputAccData.accPkts[i].measTime > 0) {
            /* store the newest time tag found as the previous time tag */
            this->previousTime = inputAccData.accPkts[i].measTime;
            break;
        }
    }
}

/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void DVAccumulation::updateState(uint64_t callTime) {
    int i;
    double dt;
    double frameDV_B[3];                                        /* [m/s] The DV of an integrated acc measurement */
    NavTransMsgF32Payload outputData = NavTransMsgF32Payload(); /* [-] The local storage of the outgoing message data */

    /*! - read accelerometer input message */
    AccDataMsgF32Payload inputAccData = this->accPktInMsg();

    /*! - stack data in time order */

    dvAccumulation_QuickSort(
        &(inputAccData.accPkts[0]),
        0,
        MAX_ACC_BUF_PKT - 1); /* measTime is the array we want to sort. We're sorting the time calculated for each
                                 measurement taken from the accelerometer in order in terms of time. */

    /*! - Ensure that the computed dt doesn't get huge.*/
    if (this->dvInitialized == 0) {
        for (i = 0; i < MAX_ACC_BUF_PKT; i++) {
            if (inputAccData.accPkts[i].measTime > this->previousTime) {
                this->previousTime = inputAccData.accPkts[i].measTime;
                this->dvInitialized = 1;
                break;
            }
        }
    }

    /*! - process new accelerometer data to accumulate Delta_v */
    for (i = 0; i < MAX_ACC_BUF_PKT; i++) {
        /*! - see if data is newer than last data time stamp */
        if (inputAccData.accPkts[i].measTime > this->previousTime) {
            dt = (inputAccData.accPkts[i].measTime - this->previousTime) * NANO2SEC;
            frameDV_B[0] = dt * static_cast<double>(inputAccData.accPkts[i].accel_B[0]);
            frameDV_B[1] = dt * static_cast<double>(inputAccData.accPkts[i].accel_B[1]);
            frameDV_B[2] = dt * static_cast<double>(inputAccData.accPkts[i].accel_B[2]);
            this->vehAccumDV_B[0] += frameDV_B[0];
            this->vehAccumDV_B[1] += frameDV_B[1];
            this->vehAccumDV_B[2] += frameDV_B[2];
            this->previousTime = inputAccData.accPkts[i].measTime;
        }
    }

    /*! - Create output message */

    outputData.timeTag = this->previousTime * NANO2SEC;
    outputData.vehAccumDV[0] = static_cast<float>(this->vehAccumDV_B[0]);
    outputData.vehAccumDV[1] = static_cast<float>(this->vehAccumDV_B[1]);
    outputData.vehAccumDV[2] = static_cast<float>(this->vehAccumDV_B[2]);

    /*! - write accumulated Dv message */
    this->dvAcumOutMsg.write(&outputData, this->moduleID, callTime);
}
