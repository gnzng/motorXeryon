# motorXeryon Releases

## __next (unreleased)__
- Re-initialize the controller automatically on reconnect. The XD-C reboots
  from flash at `INFO=4` (continuous status streaming), so if the controller is
  power-cycled while the IOC keeps running, the re-enumerated USB serial port
  auto-reconnects but the controller is left streaming, corrupting the
  `EPOS=?`/`STAT=?` poll parsing and forcing an IOC restart. The driver now
  registers an asyn connect/disconnect exception callback on the communications
  port; on every (re)connect it re-runs its init sequence from `poll()`
  (`INFO=0`, plus the stage-type command for linear stages) and flushes any
  streamed backlog. An event callback rather than poll-side connection sampling
  is used deliberately: a fast disconnect+reconnect can complete within a single
  poll cycle and be missed by level sampling. The motor record's comms-error bit
  is set while the port is disconnected.
- Auto-home on (re)connect. The XLS/XVS stages are index-referenced: after a
  power-cycle the controller comes up disabled and unhomed (`EncoderValid=0`)
  and silently ignores closed-loop `DPOS` moves, so restoring comms alone does
  not make the stage movable. `XeryonMotorCreateController` takes a new 8th
  argument, the home velocity (mm/s linear, deg/s rotary). When > 0, after every
  (re)connect (and at startup) the driver re-enables the amplifier and runs an
  index search (`ENBL`/`ISPD`/`INDX`) once the stage is enabled and not already
  searching, so the stage re-homes itself with no operator action. It is gated
  on the stage being enabled -- either the operator's tracked `setClosedLoop`
  intent OR the controller's reported `AmplifiersEnabled` bit. The intent covers
  a reconnect (the power-cycled controller boots with the amplifier off but the
  operator wanted it enabled, and `reHome` re-asserts `ENBL`); the reported bit
  covers a cold IOC start (the motor record does not propagate the enable through
  `setClosedLoop` at init, so the intent flag is still false even though the
  amplifier is on). A stage the operator disabled has the amplifier off and a
  false intent, so it is never auto-homed. Fires once per reconnect (a failed
  search does not loop). 0 disables it
  (backward-compatible: a 7-arg call leaves auto-home off). The poll maps the
  controller's `MotorOn`/`EncoderValid` bits to the motor record's
  moving/done/homed status so the record tracks the auto-home.

## __R1-0 (2026-02-05)__
Initial release
