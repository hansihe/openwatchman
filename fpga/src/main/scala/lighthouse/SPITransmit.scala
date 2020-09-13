package lighthouse

import spinal.core._
import spinal.lib.fsm._
import spinal.lib.slave

//noinspection ForwardReference
class SPITransmit extends Component {
  val io = new Bundle {
    val in_data = slave Stream Bits(8 bits)
    val active = out(Bool)

    // Out interface
    val sck = out(Bool)
    val sdo = out(Bool)
  }

  val out_reg = Reg(Bits(8 bits))
  io.sdo <> out_reg(7)

  val clkDiv = Reg(UInt(1 bits)) init 0
  clkDiv := clkDiv + 1

  val control_fsm = new StateMachine {
    val bit_counter = Reg(UInt(3 bits)) init 0

    io.in_data.ready := False
    io.sck := False
    io.active := False

    val stateReady : State = new State with EntryPoint {
      onEntry {
        io.active := True
      }
      whenIsActive {
        io.in_data.ready := True
        when(io.in_data.valid) {
          goto(stateTransaction)
        }
      }
      onExit {
        io.active := True
        out_reg := io.in_data.payload
      }
    }

    val stateTransaction : State = new State {
      whenIsActive {
        io.active := True
        io.sck := clkDiv(clkDiv.getBitsWidth-1)
        when(clkDiv === clkDiv.maxValue) {
          bit_counter := bit_counter + 1
          out_reg := (out_reg << 1) (1 to 7) ## B(0, 1 bits)
          when(bit_counter === 7) {
            io.in_data.ready := True
            when(io.in_data.valid) {
              out_reg := io.in_data.payload
              goto(stateTransaction)
            } otherwise {
              goto(stateReady)
            }
          }
        }
      }
      onExit {
        bit_counter := 0
      }
    }

  }

}
