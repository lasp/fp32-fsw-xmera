// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_SRUKF_HPP
#define FILTERING_CORE_SRUKF_HPP

#include <utilities/fsw/safeMath.h>
#include <filteringCore/concepts.hpp>
#include <filteringCore/dynamicsModel.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>

#include <math.h>
#include <array>
#include <limits>
#include <optional>

namespace filtering {

/*! Square-root UKF (van der Merwe & Wan, ICASSP 2001). Owns all filter state
 *  plus a settable dynamics functor; reset / timeUpdate / measurementUpdate
 *  mutate state in place. */
template <FilterState State, Dynamics<State> Dyn>
class SRuKF {
   public:
    using Covariance = Eigen::Matrix<float, State::size, State::size>;
    using SigmaWeights = Eigen::Vector<float, 2 * State::size + 1>;

    /*! Pre- and post-fit residuals returned from measurementUpdate(). */
    template <class M>
    struct UpdateResult {
        Eigen::Vector<float, M::size> preFit;
        Eigen::Vector<float, M::size> postFit;
    };

    /*! QR decomposition returning only the leading Rows×Rows block of R
     *  (transposed). Input must be wider than tall.
     *  @return Rows×Rows lower-triangular factor
     *  @param input [-] Rows×Cols matrix (Cols >= Rows) */
    template <int Rows, int Cols>
    static Eigen::Matrix<float, Rows, Rows> qrDecompositionJustR(Eigen::Matrix<float, Rows, Cols> const& input) {
        Eigen::Matrix<float, Cols, Rows> const inputT = input.transpose();
        Eigen::HouseholderQR<Eigen::Matrix<float, Cols, Rows>> const qr(inputT);
        // matrixQR() already holds R in its upper triangle. Materializing Q and
        // forming Q^T * inputT to recover it costs a Cols x Cols temporary and
        // pulls in Eigen's heap-allocating blocked product kernels.
        Eigen::Matrix<float, Rows, Rows> const rTilde =
            qr.matrixQR().template topLeftCorner<Rows, Rows>().template triangularView<Eigen::Upper>();

        return rTilde.transpose();
    }

    /*! Semidefinite Cholesky decomposition, reading the lower triangle. A positive semi-definite matrix
     *  may be singular, e.g. a process noise with zero noise on some states.
     *  @return lower-triangular L with input = L * L^T and a non-negative diagonal
     *  @param input [-] symmetric positive semi-definite matrix */
    template <int Size>
    static Eigen::Matrix<float, Size, Size> choleskyDecomposition(Eigen::Matrix<float, Size, Size> const& input) {
        constexpr float kPivotTolerance = static_cast<float>(Size) * std::numeric_limits<float>::epsilon();
        Eigen::Matrix<float, Size, Size> L = Eigen::Matrix<float, Size, Size>::Zero();
        for (int j = 0; j < Size; ++j) {
            float pivot = input(j, j);
            for (int k = 0; k < j; ++k) {
                pivot -= L(j, k) * L(j, k);
            }
            if (!(pivot > kPivotTolerance * input(j, j))) {
                continue;  // zero pivot: the state's variance is already carried by the previous states
            }
            float const ljj = sqrtf(pivot);
            L(j, j) = ljj;
            for (int i = j + 1; i < Size; ++i) {
                float sum = input(i, j);
                for (int k = 0; k < j; ++k) {
                    sum -= L(i, k) * L(j, k);
                }
                L(i, j) = sum / ljj;
            }
        }
        return L;
    }

    /*! Forward substitution: solve L*x = b with L lower triangular.
     *  @return x
     *  @param L [-] Rows×Rows lower-triangular matrix
     *  @param b [-] Rows×Cols RHS */
    template <int Rows, int Cols>
    static Eigen::Matrix<float, Rows, Cols> forwardSubstitution(Eigen::Matrix<float, Rows, Rows> const& L,
                                                                Eigen::Matrix<float, Rows, Cols> const& b) {
        Eigen::Matrix<float, Rows, Cols> x = Eigen::Matrix<float, Rows, Cols>::Zero();
        for (int col = 0; col < Cols; ++col) {
            for (int i = 0; i < Rows; ++i) {
                x(i, col) = b(i, col);
                for (int j = 0; j < i; ++j) {
                    x(i, col) -= L(i, j) * x(j, col);
                }
                x(i, col) /= L(i, i);
            }
        }
        return x;
    }

    /*! Back substitution: solve U*x = b with U upper triangular.
     *  @return x
     *  @param U [-] Rows×Rows upper-triangular matrix
     *  @param b [-] Rows×Cols RHS */
    template <int Rows, int Cols>
    static Eigen::Matrix<float, Rows, Cols> backSubstitution(Eigen::Matrix<float, Rows, Rows> const& U,
                                                             Eigen::Matrix<float, Rows, Cols> const& b) {
        Eigen::Matrix<float, Rows, Cols> x = Eigen::Matrix<float, Rows, Cols>::Zero();
        for (int col = 0; col < Cols; ++col) {
            for (int i = Rows - 1; i >= 0; --i) {
                x(i, col) = b(i, col);
                for (int j = i + 1; j < Rows; ++j) {
                    x(i, col) -= U(i, j) * x(j, col);
                }
                x(i, col) /= U(i, i);
            }
        }
        return x;
    }

    void setAlpha(float newAlpha) { this->alpha = newAlpha; }
    void setBeta(float newBeta) { this->beta = newBeta; }
    void setProcessNoise(Covariance const& newProcessNoise) { this->processNoise = newProcessNoise; }
    void setInitialState(State const& newInitialState) { this->stateInitial = newInitialState; }
    void setInitialCovariance(Covariance const& newInitialCovariance) {
        this->covarianceInitial = newInitialCovariance;
    }

    float getAlpha() const { return this->alpha; }
    float getBeta() const { return this->beta; }

    Dyn dynamics{};  //!< concrete dynamics functor; set by the owning algorithm

    // ---- Validity checks ----------------------------------------------------

    /*! Sigma-point tuning: alpha in (0, 1] and beta in [alpha^2, 2]. The lower bound on beta keeps the
     *  mean-offset weight beta - alpha^2 (wMeanOffset) from being negative, so every term of the covariance
     *  expanded about the central sigma point is a weighted square (see "Statistics about the central sigma
     *  point" in filteringCore.rst).
     *  @return true iff 0 < alpha <= 1 and alpha^2 <= beta <= 2 */
    static bool alphaAndBetaAreValid(float alpha, float beta) {
        return alpha > 0.0F && alpha <= 1.0F && beta >= alpha * alpha && beta <= 2.0F;
    }

    /*! A measurement noise standard deviation must be positive, with a positive finite variance, so the
     *  measurement noise R = std^2 * I is positive definite (a std that squares to zero or overflows fails).
     *  @return true iff std > 0 and 0 < std^2 <= FLT_MAX */
    static bool measurementNoiseStdIsValid(float noiseStd) {
        float const variance = noiseStd * noiseStd;
        return noiseStd > 0.0F && variance > 0.0F && variance <= std::numeric_limits<float>::max();
    }

    /*! Derive the sigma-point spread (lambda, eta), the sigma weights, and the
     *  process-noise Cholesky from alpha, beta, and the process noise. The estimate
     *  (state and covariance) is left untouched. */
    void configure() {
        constexpr int N = State::size;
        constexpr int numSigmaPoints = 2 * N + 1;
        constexpr auto Nfloat = static_cast<float>(N);

        this->cholProcessNoise = choleskyDecomposition<N>(this->processNoise);

        this->lambda = Nfloat * (this->alpha * this->alpha - 1.0F);
        this->eta = safeSqrtf(Nfloat + this->lambda);

        this->wM(0) = this->lambda / (Nfloat + this->lambda);
        for (int i = 1; i < numSigmaPoints; ++i) {
            this->wM(i) = 1.0F / (2.0F * (Nfloat + this->lambda));
        }
        this->wMeanOffset = fmaxf(this->beta - this->alpha * this->alpha, 0.0F);
    }

    /*! Seed the estimate from initial conditions and re-derive the filter parameters
     *  (configure() plus a state/covariance seed). */
    void reInitialize() {
        constexpr int N = State::size;

        this->state = this->stateInitial;
        this->covariance = this->covarianceInitial;
        this->sqrtCovar = choleskyDecomposition<N>(this->covarianceInitial);

        this->stateLastMeasurement = this->state;
        this->covarianceLastMeasurement = this->covariance;
        this->sqrtCovarLastMeasurement = this->sqrtCovar;
    }

    /*! Re-seed one state component from the initial conditions, in the working estimate and in the
     *  last-measurement anchor: its value from the initial state, its covariance block from the initial
     *  covariance, and its correlations with the other components zeroed. The other components keep their
     *  estimate and covariance. Use it to recover one component (a bias that left its bounds) without
     *  restarting the filter.
     *  @tparam Component tag type of the component to re-seed */
    template <class Component>
    void reInitializeComponent() {
        constexpr int offset = State::template offsetOf<Component>();
        this->state.template set<Component>(this->stateInitial.template get<Component>());
        this->stateLastMeasurement.template set<Component>(this->stateInitial.template get<Component>());
        this->reseedCovarianceBlock<offset, Component::size>(this->covariance, this->sqrtCovar);
        this->reseedCovarianceBlock<offset, Component::size>(this->covarianceLastMeasurement,
                                                             this->sqrtCovarLastMeasurement);
    }

    /*! Return the filter state to the previous measurement in the case of a bad update.
     * Re-configure in case a value became corrupted.
     */
    void clear() {
        this->state = this->stateLastMeasurement;
        this->covariance = this->covarianceLastMeasurement;
        this->sqrtCovar = this->sqrtCovarLastMeasurement;

        this->configure();
    }

    /*! Rewind to the last-measurement state and propagate by dt. If dt = 0
     *  the sigma points are populated around the last-measurement state for the
     *  next measurementUpdate but the timeUpdate math isn't done
     *  @param dt [s] elapsed time since the last measurement */
    bool timeUpdate(const float dt) {
        constexpr int N = State::size;
        constexpr int numSigmaPoints = 2 * N + 1;

        //! > Read the last state/covariance from the last measurement
        this->state = this->stateLastMeasurement;
        this->covariance = this->covarianceLastMeasurement;
        this->sqrtCovar = this->sqrtCovarLastMeasurement;

        /*! > If the time update doesn't move us forward in time, just populate the sigma points and their
        deviations for the measurement update but skip the hard math */
        if (dt == 0.0F) {
            this->sigmaPoints[0] = this->state;
            for (int i = 1; i <= N; ++i) {
                typename State::Storage const offset = this->eta * this->sqrtCovar.col(i - 1);
                this->sigmaPoints[i] = this->state.add(State(offset));
                this->sigmaPoints[i + N] = this->state.subtract(State(offset));
                this->sigmaDeviations[i] = State(offset);
                this->sigmaDeviations[i + N] = State(-offset);
            }
            this->meanOffset = State::Storage::Zero();
            this->sqrtNoiseOutsideSigmaPoints = Covariance::Zero();
        } else {
            std::array<float, 2> const interval = {0.0F, dt};

            //! > Propagate the central sigma point, and each other sigma point as its deviation from the central one
            //! > (eq 17-18), so the spread is not absorbed into a large state (see "Propagating the sigma points as
            //! > deviations" in filteringCore.rst). The central point is integrated once for all the deviations.
            std::array<State, 2 * N> deviations;
            for (int i = 0; i < N; ++i) {
                typename State::Storage const offset = this->eta * this->sqrtCovar.col(i);
                deviations[i] = State(offset);
                deviations[i + N] = State(-offset);
            }
            this->sigmaPoints[0] = filtering::propagateDeviations(this->dynamics, this->state, deviations, interval);
            for (int i = 1; i < numSigmaPoints; ++i) {
                this->sigmaDeviations[i] = deviations[i - 1];
            }
            for (int i = 1; i < numSigmaPoints; ++i) {
                this->sigmaPoints[i] = this->sigmaPoints[0].add(this->sigmaDeviations[i]);
            }

            //! > The weighted sum of the sigma points (eq 19) is kept as its offset from the central point, accumulated
            //! > from the deviations (see "Statistics about the central sigma point" in filteringCore.rst)
            this->meanOffset = State::Storage::Zero();
            for (int i = 1; i < numSigmaPoints; ++i) {
                this->meanOffset += this->wM(i) * this->sigmaDeviations[i].raw();
            }

            //! > Construct the A matrix (to be QR'ed) from the sigma point deviations, the process noise and the mean
            //! > offset (eq 20-21): every term of the covariance is a weighted square, since beta >= alpha^2 (see
            //! > "Mean-offset term" in filteringCore.rst)
            Eigen::Matrix<float, N, 3 * N + 1> A;
            for (int i = 1; i < numSigmaPoints; ++i) {
                A.col(i - 1) = sqrtf(this->wM(i)) * this->sigmaDeviations[i].raw();
            }
            A.template block<N, N>(0, numSigmaPoints - 1) = this->cholProcessNoise;
            A.col(3 * N) = sqrtf(this->wMeanOffset) * this->meanOffset;
            this->sqrtCovar = qrDecompositionJustR<N, 3 * N + 1>(A);
            this->covariance = this->sqrtCovar * this->sqrtCovar.transpose();
            this->state = this->sigmaPoints[0];
            this->sqrtNoiseOutsideSigmaPoints = this->cholProcessNoise;

            if (!this->state.allFinite() || !this->sqrtCovar.allFinite() || !this->covariance.allFinite()) {
                return false;
            }
        }
        return true;
    }

    /*! Process a measurement. Update the last-measurement state and the time-state
     *  @return pre- and post-fit residuals
     *  @param measurement [-] satisfies Measurement<M, State> */
    template <Measurement<State> M>
    std::optional<UpdateResult<M>> measurementUpdate(M const& measurement) {
        constexpr int N = State::size;
        constexpr int numSigma = 2 * N + 1;
        constexpr int MSize = M::size;

        std::optional<UpdateResult<M>> residuals = {};

        //! > Read the observation
        auto const observation = measurement.observation();

        //! > Skip the update on non-finite inputs so a bad measurement cannot propagate NaNs/Infs into the state.
        bool const inputsFinite = observation.allFinite() and measurement.noise().allFinite() and
                                  this->state.allFinite() and this->sqrtCovar.allFinite();

        if (inputsFinite) {
            //! > Using the measurement model, get the Y matrix (expected measurement) of each sigma point (eq 22)
            Eigen::Matrix<float, MSize, numSigma> yMeasPre;
            for (int j = 0; j < numSigma; ++j) {
                yMeasPre.col(j) = measurement.model(this->sigmaPoints[j]);
            }
            //! > Like eq 19, the measurement expected is the weighted sum of the Sigma points transformed (eq 23),
            //! > accumulated as deviations from the central point
            Eigen::Vector<float, MSize> yBarPre = yMeasPre.col(0);
            for (int i = 1; i < numSigma; ++i) {
                yBarPre += this->wM(i) * (yMeasPre.col(i) - yMeasPre.col(0));
            }

            //! > Sigma point deviations and mean offsets from the central point, as in the time update
            Eigen::Matrix<float, N, numSigma - 1> xDev;
            Eigen::Matrix<float, MSize, numSigma - 1> yDev;
            for (int i = 1; i < numSigma; ++i) {
                xDev.col(i - 1) = this->sigmaDeviations[i].raw();
                yDev.col(i - 1) = measurement.subtract(yMeasPre.col(i), yMeasPre.col(0));
            }
            Eigen::Vector<float, N> const& xMeanOffset = this->meanOffset;
            Eigen::Vector<float, MSize> const yMeanOffset = measurement.subtract(yBarPre, yMeasPre.col(0));

            //! > Before moving to the update, computed the prefits
            Eigen::Vector<float, MSize> const preFit = measurement.subtract(observation, yBarPre);

            //! > Sqrt the measurement noise, this could theoretically be different at every time step
            Eigen::Matrix<float, MSize, MSize> const cholMeasNoise = choleskyDecomposition<MSize>(measurement.noise());
            if (!(cholMeasNoise.diagonal().minCoeff() > 0.0F)) {
                return residuals;  // R is not positive definite: a bad update (no residuals), for the caller to clear
            }

            //! > Symmetrically to the time update, populate the A matrix for a qr decomposition (eq 24-25), with the
            //! > mean offset as its last column
            float const sqrtMeanOffsetWeight = sqrtf(this->wMeanOffset);
            Eigen::Matrix<float, MSize, 2 * N + MSize + 1> A;
            A.template leftCols<numSigma - 1>() = sqrtf(this->wM(1)) * yDev;
            A.template block<MSize, MSize>(0, numSigma - 1) = cholMeasNoise;
            A.col(2 * N + MSize) = sqrtMeanOffsetWeight * yMeanOffset;
            Eigen::Matrix<float, MSize, MSize> const sy = qrDecompositionJustR<MSize, 2 * N + MSize + 1>(A);
            if (!(sy.diagonal().cwiseAbs().minCoeff() > 0.0F)) {
                return residuals;  // S_y is singular: the substitutions would divide by zero; a bad update to clear
            }

            //! > Covariance of prior prediction computation eq 26
            Eigen::Matrix<float, N, MSize> const pXY =
                this->wM(1) * (xDev * yDev.transpose()) + this->wMeanOffset * (xMeanOffset * yMeanOffset.transpose());

            //! > Calculate Kalman gain eq 27
            Eigen::Matrix<float, MSize, N> const pXYT = pXY.transpose();
            Eigen::Matrix<float, MSize, N> const stKt = forwardSubstitution<MSize, N>(sy, pXYT);
            Eigen::Matrix<float, MSize, MSize> const syT = sy.transpose();
            Eigen::Matrix<float, N, MSize> const kMat = backSubstitution<MSize, N>(syT, stKt).transpose();

            //! > Calculate surprise factor and update state
            Eigen::Vector<float, MSize> const innovation = measurement.subtract(observation, yBarPre);
            this->state = this->sigmaPoints[0].add(State(xMeanOffset + kMat * innovation));

            //! > Updated covariance (eq 29) in Joseph form, expanded about the central point, as one QR decomposition
            //! > of a sum of squares (see "Measurement update in Joseph form" in filteringCore.rst)
            constexpr int numCols = 2 * N + MSize + N + 1;
            Eigen::Matrix<float, N, numCols> B;
            B.template leftCols<numSigma - 1>() = sqrtf(this->wM(1)) * (xDev - kMat * yDev);
            B.template block<N, MSize>(0, numSigma - 1) = kMat * cholMeasNoise;
            B.template block<N, N>(0, numSigma - 1 + MSize) = this->sqrtNoiseOutsideSigmaPoints;
            B.col(numCols - 1) = sqrtMeanOffsetWeight * (xMeanOffset - kMat * yMeanOffset);
            this->sqrtCovar = qrDecompositionJustR<N, numCols>(B);
            this->covariance = this->sqrtCovar * this->sqrtCovar.transpose();

            if (this->state.allFinite() and this->sqrtCovar.allFinite() and this->covariance.allFinite()) {
                //! > Move the state and covariance of last measuremnts up
                this->stateLastMeasurement = this->state;
                this->covarianceLastMeasurement = this->covariance;
                this->sqrtCovarLastMeasurement = this->sqrtCovar;

                //! > Now that update is done compute post fit residuals
                Eigen::Vector<float, MSize> const postFit =
                    measurement.subtract(observation, measurement.model(this->state));

                residuals = {.preFit = preFit, .postFit = postFit};
            }
        }
        return residuals;
    }

    /*! @return current filter state */
    State getState() const { return this->state; }
    /*! @return last-measurement state (post-last-measurement) */
    State getStateAtLastMeasurement() const { return this->stateLastMeasurement; }
    /*! @return current full covariance */
    Covariance getCovariance() const { return this->covariance; }

    /*! Overwrite the working state.
     *  @param newState [-] new working state */
    void setState(State const& newState) { this->state = newState; }
    /*! Overwrite the last-measurement state
     *  @param newStateLastMeas [-] new last-measurement state */
    void setStateLastMeasurement(State const& newStateLastMeas) { this->stateLastMeasurement = newStateLastMeas; }

   private:
    /*! Replace the covariance block of one component by its initial block, zero its correlations with the other
     *  components, and refactor the square root. A block-diagonal split of positive semi-definite blocks stays
     *  positive semi-definite.
     *  @param covar [-] covariance to edit
     *  @param sqrtCovarOut [-] its lower Cholesky factor, recomputed */
    template <int Offset, int Size>
    void reseedCovarianceBlock(Covariance& covar, Covariance& sqrtCovarOut) const {
        covar.template middleRows<Size>(Offset).setZero();
        covar.template middleCols<Size>(Offset).setZero();
        covar.template block<Size, Size>(Offset, Offset) =
            this->covarianceInitial.template block<Size, Size>(Offset, Offset);
        sqrtCovarOut = choleskyDecomposition<State::size>(covar);
    }

    /*!  Working state at the current call time. */
    State state;
    Covariance covariance;
    Covariance sqrtCovar;

    /*!  Process noise and Cholesky decomposition of process noise. */
    Covariance processNoise;
    Covariance cholProcessNoise;

    /*!  Initial conditions */
    State stateInitial;
    Covariance covarianceInitial;

    /*!  Rolling last-measurement state. */
    State stateLastMeasurement;
    Covariance covarianceLastMeasurement;
    Covariance sqrtCovarLastMeasurement;

    /*!  Filter parameters.  */
    float alpha = 0;
    float beta = 0;
    float lambda = 0;
    float eta = 0;

    Covariance sqrtNoiseOutsideSigmaPoints = Covariance::Zero();

    /*!  Sigma weights, latest propagated sigma points, their deviations from the central one (entry 0 unused), and
     *   the weighted mean as its offset from the central point. The deviations and the mean offset are kept
     *   separately so they are not absorbed into a large state in float. */
    SigmaWeights wM = SigmaWeights::Zero();
    /*!  Covariance weight of the mean offset, beta - alpha^2 (see filteringCore.rst). */
    float wMeanOffset = 0.0F;
    std::array<State, 2 * State::size + 1> sigmaPoints = {};
    std::array<State, 2 * State::size + 1> sigmaDeviations = {};
    typename State::Storage meanOffset = State::Storage::Zero();
};

}  // namespace filtering

#endif
