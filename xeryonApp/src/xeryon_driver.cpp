#include <asynOctetSyncIO.h>
#include <cstdio>
#include <cstdlib>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <optional>

#include "xeryon_driver.hpp"

constexpr int ENCODER_COUNT_MAX = 57600;
constexpr double DRIVER_RESOLUTION = 0.00625; // deg/count

// Parse reply from commands sent by the controller
// E.g. send "EPOS=?", receive "EPOS=1000".
// parse_reply("EPOS=1000") returns 1000 as an optional<int>
std::optional<int> parse_reply(const std::string &str) {
    if (auto ind = str.find('='); ind != std::string::npos) {
        try {
            return std::stoi(str.substr(ind + 1));
        } catch (...) {
            return std::nullopt; // invalid integer
        }
    }
    return std::nullopt; // '=' not found
}

XeryonMotorController::XeryonMotorController(const char *portName, const char *XeryonMotorPortName,
                                             int numAxes, double movingPollPeriod,
                                             double idlePollPeriod, const char *stageTypeCmd,
                                             double resolutionNm, double homeVelocity)
    : asynMotorController(portName, numAxes, NUM_PARAMS,
                          0, // No additional interfaces beyond the base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    asynStatus status;
    int axis;
    static const char *functionName = "XeryonMotorController::XeryonMotorController";

    stageTypeCmd_ = stageTypeCmd ? stageTypeCmd : "";
    resolutionNm_ = resolutionNm;
    homeVelocity_ = homeVelocity;
    // Arm an index search for the first poll once the axis is enabled: an
    // index-referenced stage powers up unhomed, so the IOC must home it before
    // closed-loop moves work. Only meaningful when auto-home is configured.
    autoHomePending_ = (homeVelocity_ > 0);

    createParam(FREQUENCY1_STRING, asynParamInt32, &frequency1Index_);
    createParam(FREQUENCY2_STRING, asynParamInt32, &frequency2Index_);
    createParam(READ_PARAMS_STRING, asynParamInt32, &readParamsIndex_);
    createParam(CONTROL_TIMEOUT_STRING, asynParamInt32, &controlTimeoutIndex_);
    createParam(CONTROL_TIMEOUT2_STRING, asynParamInt32, &controlTimeout2Index_);
    createParam(CONTROL_FREQUENCY_STRING, asynParamInt32, &controlFreqIndex_);
    createParam(POS_TOLERANCE_STRING, asynParamInt32, &posToleranceIndex_);
    createParam(POS_TOLERANCE2_STRING, asynParamInt32, &posTolerance2Index_);
    createParam(STATUS_BITS_STRING, asynParamInt32, &statusBitsIndex_);
    createParam(ZONE1_STRING, asynParamInt32, &zone1Index_);
    createParam(ZONE2_STRING, asynParamInt32, &zone2Index_);
    createParam(PHASE_CORRECTION_STRING, asynParamInt32, &phaseCorrectionIndex_);
    createParam(OPEN_LOOP_JOG_STRING, asynParamInt32, &openLoopJogIndex_);
    createParam(OPEN_LOOP_AMPLITUDE_STRING, asynParamInt32, &openLoopAmplIndex_);
    createParam(OPEN_LOOP_PHASE_OFFSET_STRING, asynParamInt32, &openLoopPhaseOffsetIndex_);
    createParam(SCAN_JOG_STRING, asynParamInt32, &scanJogIndex_);

    // Map asyn parameter indices to their associated controller command strings
    cmd_param_map_ = std::unordered_map<int, std::string>{
        {frequency1Index_, "FREQ"},      {frequency2Index_, "FRQ2"},
        {zone1Index_, "ZON1"},           {zone2Index_, "ZON2"},
        {controlFreqIndex_, "CFRQ"},     {posToleranceIndex_, "PTOL"},
        {posTolerance2Index_, "PTO2"},   {controlTimeoutIndex_, "TOUT"},
        {phaseCorrectionIndex_, "PHAC"}, {controlTimeout2Index_, "TOU2"},
        {openLoopAmplIndex_, "AMPL"},    {openLoopPhaseOffsetIndex_, "PHAS"}};

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(XeryonMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to Xeryon controller\n", functionName);
    }

    // Create XeryonMotorAxis object for each axis
    for (axis = 0; axis < numAxes; axis++) {
        new XeryonMotorAxis(this, axis);
    }

    // Run the one-time init sequence (silence the status stream, set the stage
    // type). The same sequence is re-sent automatically whenever the asyn port
    // reconnects -- see connectionCallback() / XeryonMotorAxis::poll().
    initController();

    // Listen for connect/disconnect transitions on the communications port so
    // we can re-init after a controller power-cycle / USB re-enumeration. A
    // dedicated asynUser is used because pasynUserController_->userPvt is owned
    // by asynOctetSyncIO; here we stash "this" to recover it in the callback.
    pasynUserCommon_ = pasynManager->createAsynUser(0, 0);
    pasynUserCommon_->userPvt = this;
    status = pasynManager->connectDevice(pasynUserCommon_, XeryonMotorPortName, 0);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect common asynUser for exception callback\n", functionName);
    } else {
        pasynManager->exceptionCallbackAdd(pasynUserCommon_, connectionCallback);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 0);
}

// asyn invokes this (on the port thread) whenever a port exception fires. We
// only care about connect-state changes: on a (re)connect we flag a re-init for
// poll() to perform. We must NOT do port I/O here -- the poller thread owns that.
void XeryonMotorController::connectionCallback(asynUser *pasynUser, asynException exception) {
    if (exception != asynExceptionConnect) {
        return;
    }
    auto *self = static_cast<XeryonMotorController *>(pasynUser->userPvt);
    int connected = 0;
    pasynManager->isConnected(pasynUser, &connected);
    if (connected) {
        self->needsReinit_ = true;
    }
}

// Send the controller initialization sequence. Called once at construction and
// again from XeryonMotorAxis::poll() whenever the asyn port reconnects: the
// XD-C controller reboots from flash at INFO=4 (continuous status streaming),
// so after a power-cycle / USB re-enumeration we must re-silence it or the
// streamed frames corrupt our EPOS=?/STAT=? poll parsing.
asynStatus XeryonMotorController::initController() {
    asynStatus status;

    // Silence the controller's unsolicited status stream.
    sprintf(this->outString_, "INFO=0");
    status = writeController();
    // Discard any frames streamed before INFO=0 took effect so they are not
    // mistaken for query replies by the next poll.
    pasynOctetSyncIO->flush(pasynUserController_);

    // Stage type: for linear stages we trust the controller's flash
    // configuration unless an explicit type command (e.g. "XLS3=1250") was
    // given. Rotary (legacy) behavior keeps the hardcoded XRTA stage.
    if (isLinear()) {
        if (!stageTypeCmd_.empty()) {
            sprintf(this->outString_, "%s", stageTypeCmd_.c_str());
            status = writeController();
        }
    } else {
        sprintf(this->outString_, "XRTA=109");
        status = writeController();
    }
    return status;
}

extern "C" int XeryonMotorCreateController(const char *portName, const char *XeryonMotorPortName,
                                           int numAxes, int movingPollPeriod, int idlePollPeriod,
                                           const char *stageTypeCmd, double resolutionNm,
                                           double homeVelocity) {
    new XeryonMotorController(portName, XeryonMotorPortName, numAxes, movingPollPeriod / 1000.,
                              idlePollPeriod / 1000., stageTypeCmd, resolutionNm, homeVelocity);
    return (asynSuccess);
}

void XeryonMotorController::report(FILE *fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "Xeryon Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f sec\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f sec\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

XeryonMotorAxis *XeryonMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<XeryonMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

XeryonMotorAxis *XeryonMotorController::getAxis(int axisNo) {
    return static_cast<XeryonMotorAxis *>(asynMotorController::getAxis(axisNo));
}

asynStatus XeryonMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;
    XeryonMotorAxis *pAxis;

    pAxis = this->getAxis(pasynUser);
    if (!pAxis) {
        return asynError;
    };

    if (function == readParamsIndex_) {
        asyn_status = pAxis->update_params();
    } else if (function == openLoopJogIndex_) {
        sprintf(outString_, "MOVE=%d", value == 1 ? 1 : -1);
        asyn_status = writeController();
    } else if (function == scanJogIndex_) {
        sprintf(outString_, "SCAN=%d", value == 1 ? 1 : -1);
        asyn_status = writeController();
    } else if (cmd_param_map_.count(function)) {
	if (auto it = cmd_param_map_.find(function); it != cmd_param_map_.end()) {
	    sprintf(outString_, "%s=%d", it->second.c_str(), value);
	    asyn_status = this->writeController();
	    if (asyn_status) {
		goto skip;
	    }
	}
    } else {
        asyn_status = asynMotorController::writeInt32(pasynUser, value);
    }

skip:
    pAxis->callParamCallbacks();
    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::update_params() {
    asynStatus asyn_status = asynSuccess;

    for (const auto &[param_index, cmd] : pC_->cmd_param_map_) {
        sprintf(pC_->outString_, "%s=?", cmd.c_str());
        asyn_status = pC_->writeReadController();
        if (asyn_status) {
            return asyn_status;
        }
        auto ret = parse_reply(pC_->inString_);
        if (ret.has_value()) {
            pC_->setIntegerParam(param_index, ret.value());
        }
    }
    // assume caller calls callParamCallbacks()
    return asyn_status;
}

XeryonMotorAxis::XeryonMotorAxis(XeryonMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "XeryonMotorAxis created with axis index %d\n",
              axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    callParamCallbacks();
}

void XeryonMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}

asynStatus XeryonMotorAxis::stop(double acceleration) {
    asynStatus asyn_status = asynSuccess;

    sprintf(pC_->outString_, "STOP=0");
    asyn_status = pC_->writeController();

    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::move(double position, int relative, double minVelocity,
                                 double maxVelocity, double acceleration) {
    asynStatus asyn_status = asynSuccess;

    // set the speed: linear stages take SSPD in um/s, rotary in 0.01 deg/s.
    // maxVelocity arrives in encoder counts/s.
    const int velo = pC_->isLinear()
                         ? static_cast<int>(maxVelocity * pC_->resolutionNm_ / 1000.)
                         : static_cast<int>(maxVelocity * DRIVER_RESOLUTION * 100);
    sprintf(pC_->outString_, "SSPD=%d", velo);
    asyn_status = pC_->writeController();
    if (asyn_status) {
        goto skip;
    }

    // Move to this target position in closed loop
    sprintf(pC_->outString_, "DPOS=%d", static_cast<int>(position));
    asyn_status = pC_->writeController();
    if (asyn_status) {
        goto skip;
    }

skip:
    callParamCallbacks();
    return asyn_status;
}

StatusBits get_status(int status) {
    StatusBits s{};
    s.AmplifiersEnabled = status & (1 << 0);
    s.EndStop = status & (1 << 1);
    s.ThermalProtection1 = status & (1 << 2);
    s.ThermalProtection2 = status & (1 << 3);
    s.ForceZero = status & (1 << 4);
    s.MotorOn = status & (1 << 5);
    s.ClosedLoop = status & (1 << 6);
    s.EncoderAtIndex = status & (1 << 7);
    s.EncoderValid = status & (1 << 8);
    s.SearchingIndex = status & (1 << 9);
    s.PositionReached = status & (1 << 10);
    s.ErrorCompensation = status & (1 << 11);
    return s;
}

asynStatus XeryonMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;

    std::optional<int> epos = 0;
    std::optional<int> stat = 0;
    StatusBits status_bits;

    // Re-initialize on reconnect. When the XD-C controller is power-cycled the
    // USB serial device re-enumerates and asyn auto-reconnects the port, but the
    // controller reboots from flash at INFO=4 (continuous status streaming) and
    // the one-time constructor init no longer applies. connectionCallback() flags
    // every (re)connect; re-send the init sequence here -- before the EPOS=?/
    // STAT=? reads below -- so the stream is silenced before we trust any reply.
    // (We do this in poll(), not the callback, because port I/O must run on the
    // poller thread; we also do not skip the reads while disconnected, since the
    // queued I/O is what drives asyn's auto-reconnect.)
    if (pC_->needsReinit_.exchange(false)) {
        pC_->initController();
        // The controller power-cycled: it has lost its index reference, so
        // request a fresh index search (performed below once we confirm the
        // stage is enabled and not already searching).
        pC_->autoHomePending_ = (pC_->homeVelocity_ > 0);
    }
    int connected = 0;
    pasynManager->isConnected(pC_->pasynUserController_, &connected);
    setIntegerParam(pC_->motorStatusCommsError_, connected ? 0 : 1);

    // Encoder position
    sprintf(pC_->outString_, "EPOS=?");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    epos = parse_reply(pC_->inString_);
    if (epos.has_value()) {
        int rbv = epos.value();
        if (!pC_->isLinear() && rbv > ENCODER_COUNT_MAX / 2) {
            // rotary stage: keep readback in range -180, 180
            rbv = rbv - ENCODER_COUNT_MAX;
        }
        setDoubleParam(pC_->motorPosition_, rbv);
        setDoubleParam(pC_->motorEncoderPosition_, rbv);
    }

    // get status word
    sprintf(pC_->outString_, "STAT=?");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    stat = parse_reply(pC_->inString_);
    if (stat.has_value()) {
        setIntegerParam(pC_->statusBitsIndex_, stat.value());
        status_bits = get_status(stat.value());
        setIntegerParam(pC_->motorStatusDone_, !status_bits.MotorOn);
        setIntegerParam(pC_->motorStatusMoving_, status_bits.MotorOn);
        *moving = status_bits.MotorOn;
        setIntegerParam(pC_->motorStatusPowerOn_, status_bits.AmplifiersEnabled);
        setIntegerParam(pC_->motorStatusHomed_, status_bits.EncoderValid);

        // Auto-home after a power-cycle/reconnect (or at startup). The stage is
        // index-referenced: until it finds its index EncoderValid stays 0 and
        // closed-loop moves silently do nothing. Re-home once, only when the
        // operator has the stage enabled (closedLoopEnabled_) and it isn't
        // already searching. Cleared after one attempt so a failed search does
        // not loop; a subsequent reconnect re-arms it.
        if (pC_->autoHomePending_) {
            // Gate on EITHER the operator's tracked enable intent
            // (closedLoopEnabled_) OR the controller's actually-reported
            // amplifier state. The intent flag covers a reconnect, where the
            // power-cycled controller boots with the amplifier off but the
            // operator wants it enabled (reHome re-asserts ENBL). The reported
            // bit covers a cold IOC start, where the motor record does not
            // propagate the enable through setClosedLoop, so closedLoopEnabled_
            // is still false even though the amplifier is on. A stage the
            // operator has disabled has the amplifier off and a false intent, so
            // it is never auto-homed.
            const bool enabled = pC_->closedLoopEnabled_ || status_bits.AmplifiersEnabled;
            if (status_bits.EncoderValid) {
                pC_->autoHomePending_ = false; // already referenced, nothing to do
            } else if (enabled && !status_bits.SearchingIndex) {
                reHome();
                pC_->autoHomePending_ = false;
            }
        }
    }

skip:
    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::home(double minVelocity, double maxVelocity, double acceleration,
                                 int forwards) {
    asynStatus asyn_status = asynSuccess;

    // set the homing speed: linear stages take ISPD in um/s, rotary in 0.01 deg/s
    const int velo = pC_->isLinear()
                         ? static_cast<int>(maxVelocity * pC_->resolutionNm_ / 1000.)
                         : static_cast<int>(maxVelocity * DRIVER_RESOLUTION * 100);
    sprintf(pC_->outString_, "ISPD=%d", velo);
    asyn_status = pC_->writeController();
    if (asyn_status) {
        goto skip;
    }

    // find the index
    sprintf(pC_->outString_, "INDX=%d", static_cast<bool>(forwards) ? 1 : 0);
    asyn_status = pC_->writeController();
    if (asyn_status) {
        goto skip;
    }

skip:
    callParamCallbacks();
    return asyn_status;
}

asynStatus XeryonMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;

    if (closedLoop) {
        // linear stages have one amplifier (ENBL=1, also clears latched
        // errors); the XRTA rotary stage enables both amplifiers (ENBL=3)
        sprintf(pC_->outString_, "ENBL=%d", pC_->isLinear() ? 1 : 3);
    } else {
        // disables the amplifiers
        sprintf(pC_->outString_, "ENBL=0");
    }
    asyn_status = pC_->writeController();
    // Remember the operator's enable intent so auto-home only ever drives a
    // stage that is meant to be enabled.
    pC_->closedLoopEnabled_ = closedLoop;

    callParamCallbacks();
    return asyn_status;
}

// Re-enable the amplifier and launch an index search. Used to auto-home after a
// reconnect/startup; the homing speed comes from the controller's configured
// homeVelocity_ (the controller power-cycle also drops ENBL, so re-assert it).
asynStatus XeryonMotorAxis::reHome() {
    asynStatus asyn_status;

    sprintf(pC_->outString_, "ENBL=%d", pC_->isLinear() ? 1 : 3);
    asyn_status = pC_->writeController();

    // Linear: homeVelocity_ in mm/s -> ISPD in um/s. Rotary: deg/s -> 0.01deg/s.
    const int velo = pC_->isLinear() ? static_cast<int>(pC_->homeVelocity_ * 1000)
                                      : static_cast<int>(pC_->homeVelocity_ * 100);
    sprintf(pC_->outString_, "ISPD=%d", velo);
    asyn_status = pC_->writeController();

    sprintf(pC_->outString_, "INDX=1"); // search toward the index (forward)
    asyn_status = pC_->writeController();

    return asyn_status;
}

// ==================
// iosch registration
// ==================
static const iocshArg XeryonMotorCreateControllerArg0 = {"asyn port name", iocshArgString};
static const iocshArg XeryonMotorCreateControllerArg1 = {"Controller port name", iocshArgString};
static const iocshArg XeryonMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg XeryonMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg XeryonMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg XeryonMotorCreateControllerArg5 = {
    "Stage type cmd, e.g. XLS3=1250 (empty: use controller flash config)", iocshArgString};
static const iocshArg XeryonMotorCreateControllerArg6 = {
    "Encoder resolution (nm/count) for linear stages, 0 = rotary XRTA", iocshArgDouble};
static const iocshArg XeryonMotorCreateControllerArg7 = {
    "Home velocity (mm/s linear, deg/s rotary); >0 auto-homes on (re)connect, 0 disables",
    iocshArgDouble};
static const iocshArg *const XeryonMotorCreateControllerArgs[] = {
    &XeryonMotorCreateControllerArg0, &XeryonMotorCreateControllerArg1,
    &XeryonMotorCreateControllerArg2, &XeryonMotorCreateControllerArg3,
    &XeryonMotorCreateControllerArg4, &XeryonMotorCreateControllerArg5,
    &XeryonMotorCreateControllerArg6, &XeryonMotorCreateControllerArg7};
static const iocshFuncDef XeryonMotorCreateControllerDef = {"XeryonMotorCreateController", 8,
                                                            XeryonMotorCreateControllerArgs};

static void XeryonMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    XeryonMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
                                args[4].ival, args[5].sval, args[6].dval, args[7].dval);
}

static void XeryonMotorRegister(void) {
    iocshRegister(&XeryonMotorCreateControllerDef, XeryonMotorCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(XeryonMotorRegister);
}
