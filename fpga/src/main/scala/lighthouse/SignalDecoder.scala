package lighthouse

import spinal.core._
import spinal.lib.fsm._
import spinal.lib._

class SignalDecoder(channels: Int) extends Component {

  // ==== Constants ====

  val frequencyValue = clockDomain.frequency.getValue

  // ___‾__‾_______/........................\_______‾_
  //    ^ First sync (Master lighthouse)
  //       ^ Second sync (Slave lighthouse)
  //                   ^ Valid period of sweep
  //                                                ^ Next sweep sync

  // Period (Oridinary sync to sync): 1/120Hz = 8.333ms
  // Sync pulses:
  // - Nominal tick: 0.062ms / 30 = 2.083us = 0.002083ms
  // - Min length: (25 "ticks") 52.08us = 0.05208ms
  // - Max length: (65 "ticks") 135us = 0.135ms
  // - Start of first to start of second sync: ~0.4ms
  // - Start of first to start of sweep period: ~1ms

  // === Sweep counter constants ===
  val sweepSyncRegionEnd = (frequencyValue * (0.6 ms)).toBigInt
  val sweepSweepRegionStart = (frequencyValue * (0.8 ms)).toBigInt
  val sweepSweepRegionEnd = (frequencyValue * (7.2 ms)).toBigInt
  val sweepNextSyncStart = (frequencyValue * (7.2 ms)).toBigInt
  val sweepNextSyncEnd = (frequencyValue * (9.5 ms)).toBigInt

  val sweepExactCycles = (frequencyValue / (120 Hz)).toBigInt
  assert(sweepNextSyncStart < sweepExactCycles)
  assert(sweepExactCycles < sweepNextSyncEnd)

  val sweepCounterBits = log2Up(sweepNextSyncEnd) bits

  // === Tick counter constants
  // each tick is 10.42us, 96kHz
  // j0: 6 ticks
  // k0: 7 ticks
  // j1: 8 ticks
  // k1: 9 ticks
  // j2: 10 ticks
  // k2: 11 ticks
  // j3: 12 ticks
  // k3: 13 ticks
  // Used for measuring the length of the sync pulse.
  val tickCounterDividerCycles = (clockDomain.frequency.getValue / (0.096 MHz)).toBigInt
  val tickCounterDividerBits = log2Up(tickCounterDividerCycles) bits
  val tickCounterMinTicks = 6
  val tickCounterMaxTicks = 13
  val tickCounterBits = log2Up(tickCounterMaxTicks + 1) bits

  //// The exact period of each laser sweep, sync pulse to sync pulse.
  //val exactSweepLengthCycles: Int = (frequencyValue / (120 Hz)).toInt
  //// The amount of cycles the sweep counter will count
  //val maxSweepCounterCycles = (exactSweepLengthCycles - (frequencyValue * (0.8 ms))).toInt
  //val sweepCounterBeginWaitNextCycle = frequencyValue * (7.5 ms)
  //val sweepCounterEndWaitNextCycle = frequencyValue * (9 ms)
  //// The amount of cycles the sweep counter will wait before listening to sweep.
  //// This is the wait time for the sync pulse from the slave lighthouse
  //val minSweepCounterCycles = (frequencyValue * (0.5 ms)).toInt
  //val sweepCounterBits = log2Up(maxSweepCounterCycles) bits

  // ==== IO ====

  val io = new Bundle {
    val envelope = in Vec(Bool, channels)

    val sweepResult = master Flow new OutData
    val timeout = out Bool
    val sweepCont = master Flow UInt(sweepCounterBits)

    val debug = out Bool
  }

  class OutData extends Bundle {
    val firstSyncTicks = UInt(tickCounterBits)
    val secondSyncStart = UInt(sweepCounterBits)
    val secondSyncTicks = UInt(tickCounterBits)
    val sweepCycles = Vec(UInt(sweepCounterBits), channels)
  }

  val envelopeMajority = MajorityVote(io.envelope)

  // ==== Registers ====

  val firstSyncTicks = Reg(UInt(tickCounterBits)) init 0
  val secondSyncStart = Reg(UInt(sweepCounterBits)) init 0
  val secondSyncTicks = Reg(UInt(tickCounterBits)) init 0
  val sweepCaptured = Vec(Reg(Bool) init False, channels)
  val sweepCycles = Vec(Reg(UInt(sweepCounterBits)) init 0, channels)

  firstSyncTicks <> io.sweepResult.payload.firstSyncTicks
  secondSyncStart <> io.sweepResult.payload.secondSyncStart
  secondSyncTicks <> io.sweepResult.payload.secondSyncTicks
  sweepCycles <> io.sweepResult.payload.sweepCycles

  // ==== Timers ====

  val tickTimer = new Area {
    val reset = Bool

    val divider = Reg(UInt(tickCounterDividerBits)) init 0
    val dividerEnd = divider === (tickCounterDividerCycles - 1)

    val ticks = Reg(UInt(tickCounterBits)) init 0

    divider := divider + 1
    when(dividerEnd) {
      divider := 0
      ticks := ticks + 1
    }

    when(reset) {
      divider := 0
      ticks := 0
    }
  }

  val sweepTimer = new Area {
    val reset = Bool

    val timer = Reg(UInt(sweepCounterBits)) init 0
    timer := timer + 1

    when(reset) {
      timer := 0
    }
  }

  val controlFsm = new StateMachine {

    tickTimer.reset := True
    sweepTimer.reset := False
    io.sweepResult.valid := False
    io.sweepCont.valid := False
    io.sweepCont.payload <> sweepTimer.timer
    io.timeout := False
    //io.debug <> tickTimer.ticks(0)
    io.debug := True

    // Ready state, wait for the envelope detector to be asserted, jump to the first pulse timing state.
    // Keep all state registers in reset state.
    val readyState: State = new State with EntryPoint {
      whenIsActive {
        io.debug := False
        sweepTimer.reset := True
        for (n <- 0 until channels) {
          sweepCaptured(n) := False
          sweepCycles(n) := 0
        }
        when(envelopeMajority) {
          goto(firstSyncActiveState)
        }
      }
    }

    // Reset wait state.
    // Wait for envelope to be deasserted, then goto ready.
    val waitNoEnvelopeResetState = new State {
      whenIsActive {
        sweepTimer.reset := True
        when(!envelopeMajority) {
          goto(readyState)
        }
      }
    }

    // Start the sync pulse timer.
    // Premature or late deassertion of the envelope timer goes back to ready.
    // If we got a valid sync pulse, start waiting for a second one.
    val firstSyncActiveState: State = new State {
      whenIsActive {
        tickTimer.reset := False
        // Pulse has exceeded the length of a valid sync pulse. Goto reset.
        when(tickTimer.ticks > tickCounterMaxTicks) {
          goto(waitNoEnvelopeResetState)
        }
        when(!envelopeMajority) {
          when(tickTimer.ticks < tickCounterMinTicks) {
            // Pulse too short. Goto reset.
            goto(readyState)
          } otherwise {
            // Valid sync, capture length and wait for second sync.
            firstSyncTicks := tickTimer.ticks
            goto(waitSecondSyncState)
          }
        }
      }
    }

    val waitSecondSyncState = new State {
      whenIsActive {
        // If we are out of the time frame the second sync pulse can occur, go to sweep state.
        when(sweepTimer.timer >= sweepSyncRegionEnd) {
          goto(timeSweepState)
        }
        // Second sync pulse detected
        when(envelopeMajority) {
          goto(secondSyncActiveState)
        }
      }
    }

    val secondSyncActiveState = new State {
      onEntry {
        secondSyncStart := sweepTimer.timer
      }
      whenIsActive {
        tickTimer.reset := False
        when(tickTimer.ticks > tickCounterMaxTicks) {
          // Pulse has exceeded the length of a valid sync pulse. Log and continue
          secondSyncTicks := tickTimer.ticks
          goto(timeSweepState)
        }
        when(!envelopeMajority) {
          // Valid sync, capture length and goto sweep. Too short is handled by MCU
          secondSyncTicks := tickTimer.ticks
          goto(timeSweepState)
        }
      }
    }

    val timeSweepState = new State {
      whenIsActive {
        for (n <- 0 until channels) {
          when(!sweepCaptured(n) && io.envelope(n)) {
            sweepCaptured(n) := True
            sweepCycles(n) := sweepTimer.timer
          }
        }
        when(sweepTimer.timer >= sweepSweepRegionEnd) {
          goto(waitNextSyncState)
        }
      }
      onExit {
        io.sweepResult.valid := True
      }
    }

    val waitNextSyncState = new State {
      whenIsActive {
        when(envelopeMajority) {
          io.sweepCont.valid := True
          sweepTimer.reset := True
          goto(firstSyncActiveState)
        } otherwise when(sweepTimer.timer >= sweepNextSyncEnd - 1) {
          io.timeout := True
          goto(waitNoEnvelopeResetState)
        }
        for (n <- 0 until channels) {
          sweepCaptured(n) := False
          sweepCycles(n) := 0
        }
      }
      onExit {
        sweepTimer.reset := True
      }
    }

  }

}
