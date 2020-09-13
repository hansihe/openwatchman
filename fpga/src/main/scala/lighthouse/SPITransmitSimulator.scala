package lighthouse

import spinal.core._
import spinal.sim._
import spinal.core.sim._

import scala.util.Random

//noinspection FieldFromDelayedInit
object SPITransmitSimulator {
  def main(args: Array[String]) {
    SimConfig.withWave.doSim(new SPITransmit){dut =>
      //Fork a process to generate the reset and the clock on the dut
      dut.clockDomain.forkStimulus(period = 10)

      var idx = 0
      while(idx < 4){
        dut.io.in_data.valid #= true

        waitUntil(dut.io.in_data.ready.toBoolean)
        dut.io.in_data.payload #= 0x5a
        dut.clockDomain.waitRisingEdge()

        idx += 1
      }
      dut.io.in_data.valid #= false

      dut.clockDomain.waitRisingEdge(20)

    }
  }
}

