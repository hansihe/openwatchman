package lighthouse

import spinal.core._

class LighthouseWrapperICE40 extends Component {
  val io = new Bundle {
    val CLK = in Bool
    val LED = out Bool
    val PIN_18 = in Bool

    val PIN_9 = out Bool // Debug

    val PIN_20 = out Bool // SCK
    val PIN_21 = out Bool // SDO
    val PIN_22 = out Bool // CS
  }
  noIoPrefix()

  val power_on_reset = new ClockingArea(ClockDomain(io.CLK, config = ClockDomainConfig(resetKind = BOOT))) {
    val cnt = Reg(UInt(5 bits)) init 0 // reg auto initied to 0 at boot
    val rst = cnt.andR
    cnt := cnt + ~rst.asUInt
  }

  val mainClockDomain = ClockDomain(
    clock = io.CLK,
    reset = power_on_reset.rst,
    config = ClockDomainConfig(
      clockEdge = RISING,
      resetKind = ASYNC,
      resetActiveLevel = LOW
    ),
    frequency = FixedFrequency(16 MHz) // 16MHz oscillator on board
  )

  val topClockArea = new ClockingArea(mainClockDomain) {
    val toplevel = new LighthouseToplevel
    toplevel.io.detectorPin <> io.PIN_18
    toplevel.io.led <> io.LED
    toplevel.io.sck <> io.PIN_20
    toplevel.io.sdo <> io.PIN_21
    toplevel.io.cs <> io.PIN_22
    toplevel.io.debug <> io.PIN_9
  }

}

object LighthouseWrapperICE40Verilog {
  def main(args: Array[String]) {
    SpinalVerilog(new LighthouseWrapperICE40)
  }
}
