#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"
#include <atomic>
#include <string>
#include <unordered_map>

// String names for parameters in asyn parameter library
static constexpr char READ_PARAMS_STRING[] = "READ_PARAMS";
static constexpr char FREQUENCY1_STRING[] = "FREQUENCY1";
static constexpr char FREQUENCY2_STRING[] = "FREQUENCY2";
static constexpr char CONTROL_FREQUENCY_STRING[] = "CONTROL_FREQUENCY";
static constexpr char POS_TOLERANCE_STRING[] = "POSITION_TOLERANCE";
static constexpr char POS_TOLERANCE2_STRING[] = "POSITION_TOLERANCE2";
static constexpr char CONTROL_TIMEOUT_STRING[] = "CONTROL_TIMEOUT";
static constexpr char CONTROL_TIMEOUT2_STRING[] = "CONTROL_TIMEOUT2";
static constexpr char ZONE1_STRING[] = "ZONE1";
static constexpr char ZONE2_STRING[] = "ZONE2";
static constexpr char STATUS_BITS_STRING[] = "STATUS_BITS";
static constexpr char PHASE_CORRECTION_STRING[] = "PHASE_CORRECTION";
static constexpr char OPEN_LOOP_AMPLITUDE_STRING[] = "OPEN_LOOP_AMPLITUDE";
static constexpr char OPEN_LOOP_PHASE_OFFSET_STRING[] = "OPEN_LOOP_PHASE_OFFSET";
static constexpr char OPEN_LOOP_JOG_STRING[] = "OPEN_LOOP_JOG";
static constexpr char SCAN_JOG_STRING[] = "SCAN_JOG";

// Convenience struct to store status bits from controller
struct StatusBits {
    bool AmplifiersEnabled;
    bool EndStop;
    bool ThermalProtection1;
    bool ThermalProtection2;
    bool ForceZero;
    bool MotorOn;
    bool ClosedLoop;
    bool EncoderAtIndex;
    bool EncoderValid;
    bool SearchingIndex;
    bool PositionReached;
    bool ErrorCompensation;
};

class epicsShareClass XeryonMotorAxis : public asynMotorAxis {
  public:
    XeryonMotorAxis(class XeryonMotorController *pC, int axisNo);
    void report(FILE *fp, int level) override;
    asynStatus stop(double acceleration) override;
    asynStatus poll(bool *moving) override;
    asynStatus setClosedLoop(bool closedLoop) override;
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards) override;
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity,
                    double acceleration) override;

    // Gets the latest values from the controller and updates the values
    // in the parameter library
    asynStatus update_params();

  private:
    // Re-enable and start an index search (used for auto-home on reconnect).
    asynStatus reHome();
    XeryonMotorController *pC_;
    int axisIndex_;
    friend class XeryonMotorController;
};

class epicsShareClass XeryonMotorController : public asynMotorController {
  public:
    XeryonMotorController(const char *portName, const char *XeryonMotorController, int numAxes,
                          double movingPollPeriod, double idlePollPeriod,
                          const char *stageTypeCmd, double resolutionNm, double homeVelocity);
    void report(FILE *fp, int level) override;
    XeryonMotorAxis *getAxis(asynUser *pasynUser) override;
    XeryonMotorAxis *getAxis(int axisNo) override;
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value) override;

    // Send the controller init sequence (silence the status stream, set the
    // stage type). Called once at construction and re-sent on every reconnect.
    asynStatus initController();

    // asyn exception callback (registered on the communications port). Fires on
    // every connect/disconnect transition -- including ones too brief for the
    // poller to sample -- and flags a reconnect so poll() re-runs initController.
    static void connectionCallback(asynUser *pasynUser, asynException exception);

  private:
    // Map of extra controller commands we expose through asyn parameters.
    // Must be defined after calling createParam
    std::unordered_map<int, std::string> cmd_param_map_;

  protected:
    static constexpr int NUM_PARAMS = 16;
    // Linear-stage support: when resolutionNm_ > 0 the controller is treated
    // as driving a linear stage (XLS/XLA/XVS): speeds are sent in um/s, no
    // rotary wrap-around, ENBL=1. resolutionNm_ == 0 keeps the original
    // XRTA rotary behavior.
    std::string stageTypeCmd_;
    double resolutionNm_ = 0;
    bool isLinear() const { return resolutionNm_ > 0; }
    // Dedicated asynUser used only to receive connect/disconnect exceptions from
    // the communications port (kept separate from pasynUserController_, whose
    // userPvt belongs to asynOctetSyncIO).
    asynUser *pasynUserCommon_ = nullptr;
    // Set by connectionCallback() when the port (re)connects; consumed by poll()
    // which then re-runs initController(). Atomic: written from the asyn port
    // thread, read/cleared from the poller thread.
    std::atomic<bool> needsReinit_{false};
    // Auto-home support. The XVS/XLS stages are index-referenced: after a
    // power-cycle the controller comes up with EncoderValid=0 and refuses
    // closed-loop moves until it finds its index. homeVelocity_ (mm/s linear,
    // deg/s rotary) > 0 enables an automatic index search after every
    // (re)connect; 0 disables it. autoHomePending_ requests that search;
    // closedLoopEnabled_ (last setClosedLoop state) gates it so we never drive a
    // stage the operator has disabled. All three are touched only under the
    // asynPortDriver lock (poller / handler threads), so no atomics are needed.
    double homeVelocity_ = 0;
    bool autoHomePending_ = false;
    bool closedLoopEnabled_ = false;
    int readParamsIndex_;
    int frequency1Index_;
    int frequency2Index_;
    int controlFreqIndex_;
    int posToleranceIndex_;
    int posTolerance2Index_;
    int controlTimeoutIndex_;
    int controlTimeout2Index_;
    int statusBitsIndex_;
    int zone1Index_;
    int zone2Index_;
    int phaseCorrectionIndex_;
    int openLoopAmplIndex_;
    int openLoopPhaseOffsetIndex_;
    int openLoopJogIndex_;
    int scanJogIndex_;

    friend class XeryonMotorAxis;
};
