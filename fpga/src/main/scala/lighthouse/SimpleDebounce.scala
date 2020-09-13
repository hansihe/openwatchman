package lighthouse

import spinal.core._
import spinal.lib.MajorityVote

class SimpleDebounce(cycles: Int) extends Component {
  val io = new Bundle {
    val input = in Bool
    val output = out Bool
  }

  val shiftReg = Reg(Bits(cycles bits)) init 0
  shiftReg := (shiftReg << 1).resize(cycles)
  shiftReg(0) := io.input

  val majority = MajorityVote(shiftReg)
  io.output <> majority

}
