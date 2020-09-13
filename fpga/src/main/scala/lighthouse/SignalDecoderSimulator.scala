package lighthouse

import spinal.core._
import spinal.sim._
import spinal.core.sim._

import scala.util.Random

//noinspection FieldFromDelayedInit
object SignalDecoderSimulator {
  def main(args: Array[String]) {
    SimConfig.withConfig(new SpinalConfig(defaultClockDomainFrequency = FixedFrequency(10 MHz))).withWave.compile {
      //val dut = new SignalDecoder(1)
      //dut.controlFsm.stateReg.simPublic()
      //dut
      val dut = new LighthouseToplevel
      dut.signalDecoder.controlFsm.stateReg.simPublic()
      dut
    }.doSim {dut =>
      ////Fork a process to generate the reset and the clock on the dut
      //dut.clockDomain.forkStimulus(period = 10)
      //dut.io.envelope(0) #= false

      //// Dual sync

      //// Start first sync pulse
      //dut.clockDomain.waitSampling(100)
      //dut.io.envelope(0) #= true
      //dut.clockDomain.waitSampling(dut.tickLengthCycles*7 + 10)
      //dut.io.envelope(0) #= false
      //dut.clockDomain.waitSampling(10)
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.waitSecondSyncState) == dut.controlFsm.stateReg.toEnum)

      //// Start second sync pulse
      //dut.clockDomain.waitSampling(dut.minSweepCounterCycles / 2)
      //dut.io.envelope(0) #= true
      //dut.clockDomain.waitSampling(dut.tickLengthCycles*9 + 10)
      //dut.io.envelope(0) #= false
      //dut.clockDomain.waitSampling(10)
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.timeSweepState) == dut.controlFsm.stateReg.toEnum)

      //// Laser pulse
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.timeSweepState) == dut.controlFsm.stateReg.toEnum)
      //dut.clockDomain.waitSampling(20000)
      //dut.io.envelope(0) #= true
      //dut.clockDomain.waitSampling(500)
      //dut.io.envelope(0) #= false
      //dut.clockDomain.waitSampling(10)
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.timeSweepState) == dut.controlFsm.stateReg.toEnum)

      //// Wait for data ready
      //dut.clockDomain.waitSamplingWhere(dut.io.outData.valid.toBoolean)
      //dut.clockDomain.waitSampling(10)
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.readyState) == dut.controlFsm.stateReg.toEnum)

      //// Single sync

      //// Start first sync pulse
      //dut.clockDomain.waitSampling(100)
      //dut.io.envelope(0) #= true
      //dut.clockDomain.waitSampling(dut.tickLengthCycles*7 + 10)
      //dut.io.envelope(0) #= false
      //dut.clockDomain.waitSampling(10)
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.waitSecondSyncState) == dut.controlFsm.stateReg.toEnum)

      //// Laser pulse
      //dut.clockDomain.waitSampling(20000)
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.timeSweepState) == dut.controlFsm.stateReg.toEnum)
      //dut.io.envelope(0) #= true
      //dut.clockDomain.waitSampling(500)
      //dut.io.envelope(0) #= false
      //dut.clockDomain.waitSampling(10)
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.timeSweepState) == dut.controlFsm.stateReg.toEnum)

      //// Wait for data ready
      //dut.clockDomain.waitSamplingWhere(dut.io.outData.valid.toBoolean)
      //dut.clockDomain.waitSampling(10)
      //assert(dut.controlFsm.stateToEnumElement(dut.controlFsm.readyState) == dut.controlFsm.stateReg.toEnum)


      def waitTime(time: TimeNumber) = {
        dut.clockDomain.waitRisingEdge((dut.clockDomain.frequency.getValue * time).toInt)
      }


      //Fork a process to generate the reset and the clock on the dut
      val a = dut.clockDomain.forkStimulus(period = 10)
      dut.io.detectorPin #= false

      // Dual sync

      //// Start first sync pulse
      //dut.clockDomain.waitSampling(100)
      //dut.io.detectorPin #= true
      //dut.clockDomain.waitSampling(dut.signalDecoder.tickCounterDividerCycles.toInt*7 + 10)
      //dut.io.detectorPin #= false
      //dut.clockDomain.waitSampling(10)
      //assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.waitSecondSyncState) == dut.signalDecoder.controlFsm.stateReg.toEnum)

      //// Start second sync pulse
      //dut.clockDomain.waitSampling(dut.signalDecoder.sweepSweepRegionEnd - 5000)
      //dut.io.detectorPin #= true
      ////dut.clockDomain.waitSampling(dut.signalDecoder.tickLengthCycles*9 + 10)
      //dut.io.detectorPin #= false
      //dut.clockDomain.waitSampling(10)
      //assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.timeSweepState) == dut.signalDecoder.controlFsm.stateReg.toEnum)

      //// Laser pulse
      //assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.timeSweepState) == dut.signalDecoder.controlFsm.stateReg.toEnum)
      //dut.clockDomain.waitSampling(20000)
      //dut.io.detectorPin #= true
      //dut.clockDomain.waitSampling(500)
      //dut.io.detectorPin #= false
      //dut.clockDomain.waitSampling(10)
      //assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.timeSweepState) == dut.signalDecoder.controlFsm.stateReg.toEnum)

      //// Wait for data ready
      //dut.clockDomain.waitSamplingWhere(!dut.io.cs.toBoolean)
      //dut.clockDomain.waitSampling(10)
      //assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.readyState) == dut.signalDecoder.controlFsm.stateReg.toEnum)

      // ==== Realistic single ====
      dut.clockDomain.waitSampling(100)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.readyState) == dut.signalDecoder.controlFsm.stateReg.toEnum)
      dut.io.detectorPin #= true
      waitTime(64 us)
      dut.io.detectorPin #= false
      waitTime(3.44 ms)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.timeSweepState) == dut.signalDecoder.controlFsm.stateReg.toEnum)
      dut.io.detectorPin #= true
      waitTime(10 us)
      dut.io.detectorPin #= false
      waitTime(4.84 ms)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.waitNextSyncState) == dut.signalDecoder.controlFsm.stateReg.toEnum)

      dut.io.detectorPin #= true
      waitTime(64 us)
      dut.io.detectorPin #= false
      waitTime(3.44 ms)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.timeSweepState) == dut.signalDecoder.controlFsm.stateReg.toEnum)
      dut.io.detectorPin #= true
      waitTime(10 us)
      dut.io.detectorPin #= false
      waitTime(4.84 ms)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.waitNextSyncState) == dut.signalDecoder.controlFsm.stateReg.toEnum)

      // Anomalous pulse
      dut.io.detectorPin #= true
      waitTime(138 us)
      dut.io.detectorPin #= false
      waitTime(3.44 ms)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.timeSweepState) == dut.signalDecoder.controlFsm.stateReg.toEnum)
      dut.io.detectorPin #= true
      waitTime(10 us)
      dut.io.detectorPin #= false
      waitTime(4.84 ms)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.waitNextSyncState) == dut.signalDecoder.controlFsm.stateReg.toEnum)



      // ==== Single sync ====

      // Start first sync pulse
      dut.clockDomain.waitSampling(100)
      dut.io.detectorPin #= true
      waitTime(80 us)
      //dut.clockDomain.waitSampling(dut.signalDecoder.tickCounterDividerCycles.toInt*7 + 10)
      dut.io.detectorPin #= false
      dut.clockDomain.waitSampling(10)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.waitSecondSyncState) == dut.signalDecoder.controlFsm.stateReg.toEnum)

      // Laser pulse
      waitTime(6 ms)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.timeSweepState) == dut.signalDecoder.controlFsm.stateReg.toEnum)
      dut.io.detectorPin #= true
      dut.clockDomain.waitSampling(500)
      dut.io.detectorPin #= false
      dut.clockDomain.waitSampling(10)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.timeSweepState) == dut.signalDecoder.controlFsm.stateReg.toEnum)

      // Wait for data ready
      dut.clockDomain.waitSamplingWhere(!dut.io.cs.toBoolean)
      dut.clockDomain.waitSampling(10)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.waitNextSyncState) == dut.signalDecoder.controlFsm.stateReg.toEnum)
      dut.clockDomain.waitSampling(100)

      // Next sync
      waitTime(0.5 ms)
      dut.io.detectorPin #= true
      dut.clockDomain.waitSampling(10)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.firstSyncActiveState) == dut.signalDecoder.controlFsm.stateReg.toEnum)
      waitTime(80 us)
      dut.io.detectorPin #= false
      dut.clockDomain.waitSampling(10)
      assert(dut.signalDecoder.controlFsm.stateToEnumElement(dut.signalDecoder.controlFsm.waitSecondSyncState) == dut.signalDecoder.controlFsm.stateReg.toEnum)


    }
  }
}

