package lighthouse

import java.util

import spinal.core._
import spinal.lib.Timeout
import spinal.lib.fsm._

import scala.collection.mutable.{ArrayBuffer, ListBuffer}

class LighthouseToplevel extends Component {
  val io = new Bundle {
    val detectorPin = in Bool
    val led = out Bool

    val debug = out Bool

    val sdo = out Bool
    val sck = out Bool
    val cs = out Bool
  }

  val activeDetectionTimeout = Timeout(2 ms)
  when(io.detectorPin) {
    activeDetectionTimeout.clear()
  }
  io.led <> !activeDetectionTimeout
  //val activeDetection = new Area {
  //  val timeout = Reg(UInt(20 bits)) init 0

  //  val active = !timeout.asBits.andR
  //  when(active) {
  //    timeout := timeout + 1
  //  }

  //  when(io.detectorPin) {
  //    timeout := 0
  //  }

  //  io.led <> active
  //}

  val debouncer = new SimpleDebounce(5)
  debouncer.io.input <> io.detectorPin

  val signalDecoder = new SignalDecoder(1)
  //signalDecoder.io.envelope(0) <> io.detectorPin
  signalDecoder.io.envelope(0) <> debouncer.io.output
  signalDecoder.io.debug <> io.debug

  //val decodedSignalBuf = Reg(Bits(signalDecoder.io.sweepResult.payload.getBitsWidth bits)) init 0
  //val emitting = Reg(Bool) init False
  //val msg = Reg(Bool) init False
  //when(signalDecoder.io.sweepResult.valid && !emitting) {
  //  decodedSignalBuf := signalDecoder.io.sweepResult.payload.asBits
  //  msg := False
  //  emitting := True
  //}

  //when(signalDecoder.io.timeout && !emitting) {
  //  decodedSignalBuf := 0
  //  msg := True
  //  emitting := True
  //}

  //val outBytes = math.max(8, (decodedSignalBuf.getWidth+8 + 7) / 8)
  //val outBits = outBytes*8
  //val outSlices = Vec(decodedSignalBuf.resize(outBits-8).resizeLeft(outBits).subdivideIn(8 bits).reverse)

  val spiTransmit = new SPITransmit
  io.sdo <> spiTransmit.io.sdo
  io.sck <> spiTransmit.io.sck

  // TODO: Hacky
  //val csTimeout = Timeout(20)
  //when(emitting) {
  //  csTimeout.clear()
  //}
  //io.cs <> csTimeout

  //val delayHack = Reg(UInt(2 bits)) init 0
  //val outCounter = Reg(UInt(log2Up(outBytes + 1) bits)) init 0

  //spiTransmit.io.in_data.valid := False
  //spiTransmit.io.in_data.payload := 0
  //when(emitting) {
  //  when(delayHack === 3) {
  //    when(outCounter === outBytes) {
  //      emitting := False
  //      outCounter := 0
  //      delayHack := 0
  //    } otherwise {
  //      spiTransmit.io.in_data.valid := True
  //      //spiTransmit.io.in_data.payload := 0x01
  //      //when(outCounter === 7) {
  //      //  spiTransmit.io.in_data.payload := 0x0f
  //      //} otherwise {
  //      //  spiTransmit.io.in_data.payload := 0x00
  //      //}
  //      when(outCounter === outBytes - 1) {
  //        when(msg) {
  //          spiTransmit.io.in_data.payload := 0x0f
  //        } otherwise {
  //          spiTransmit.io.in_data.payload := 0xf0
  //        }
  //      } otherwise {
  //        spiTransmit.io.in_data.payload := outSlices(outCounter.resize(log2Up(outSlices.length)))
  //      }
  //      when(spiTransmit.io.in_data.ready) {
  //        outCounter := outCounter + 1
  //      }
  //    }
  //  } otherwise {
  //    delayHack := delayHack + 1
  //  }
  //}

  val spiOutFsm = new StateMachine {

    spiTransmit.io.in_data.valid := False
    io.cs := True
    spiTransmit.io.in_data.payload := 0

    val sweepData = Reg(new signalDecoder.OutData)
    val sweepContData = Reg(signalDecoder.io.sweepCont.payloadType)
    val dataCounter = Reg(UInt(6 bits))

    val readyState: State = new State with EntryPoint {
      whenIsActive {
        when(signalDecoder.io.sweepResult.valid) {
          sweepData := signalDecoder.io.sweepResult.payload
          goto(transmitSweepResultState)
        }
        when(signalDecoder.io.timeout) {
          goto(transmitTimeoutState)
        }
        when(signalDecoder.io.sweepCont.valid) {
          sweepContData := signalDecoder.io.sweepCont.payload
          goto(transmitSweepContState)
        }
      }
    }

    val sweepTimerWidthRounded = ((sweepData.secondSyncStart.getBitsWidth + 7) / 8) * 8
    val sweepBytes = ArrayBuffer.empty[BitVector]
    // Message ID
    sweepBytes += B(0, 8 bits)
    // First and second sync ticks
    sweepBytes += B(8 bits, (7 downto 4) -> sweepData.secondSyncTicks.asBits, (3 downto 0) -> sweepData.firstSyncTicks.asBits)
    // Second sync start
    sweepBytes ++= sweepData.secondSyncStart.resize(sweepTimerWidthRounded bits).subdivideIn(8 bits)
    // Channel timings
    sweepBytes ++= sweepData.sweepCycles.flatMap(_.resize(sweepTimerWidthRounded bits).subdivideIn(8 bits))
    val sweepBytesV = Vec[BitVector](sweepBytes, Bits(8 bits))
    println("TransmitSweepResult width: ", sweepBytes.length)

    val transmitSweepResultState = new State {
      onEntry {
        dataCounter := 0
      }
      whenIsActive {
        io.cs := False
        spiTransmit.io.in_data.payload := sweepBytesV(dataCounter.resize(3 bits)).asBits
        when(dataCounter === sweepBytes.length) {
          when(!spiTransmit.io.active) {
            goto(readyState)
          }
        } otherwise {
          spiTransmit.io.in_data.valid := True
          when(spiTransmit.io.in_data.ready) {
            dataCounter := dataCounter + 1
          }
        }
      }
    }

    val sweepContBytes = ArrayBuffer.empty[BitVector]
    // Message ID
    sweepContBytes += B(2, 8 bits)
    // First and second sync ticks
    sweepContBytes ++= sweepContData.resize(sweepTimerWidthRounded bits).subdivideIn(8 bits)
    // Again, hack because of ESP32...
    while (sweepContBytes.length < 8) { sweepContBytes += B(0, 8 bits) }
    val sweepContBytesV = Vec[BitVector](sweepContBytes, Bits(8 bits))
    println("SweepCont width: ", sweepContBytes.length)

    val transmitTimeoutState = new State {
      onEntry {
        dataCounter := 0
      }
      whenIsActive {
        io.cs := False
        spiTransmit.io.in_data.payload := 0x01
        // ESP32 SPI Slave hack. It's buggy and discards segments of data not multiples of 4? Wtf. Should be 1 normally
        when(dataCounter === 4) {
          when(!spiTransmit.io.active) {
            goto(readyState)
          }
        } otherwise {
          spiTransmit.io.in_data.valid := True
          when(spiTransmit.io.in_data.ready) {
            dataCounter := dataCounter + 1
          }
        }
      }
    }

    val transmitSweepContState = new State {
      onEntry {
        dataCounter := 0
      }
      whenIsActive {
        io.cs := False
        spiTransmit.io.in_data.payload := sweepContBytesV(dataCounter.resize(3 bits)).asBits
        when(dataCounter === sweepContBytes.length) {
          when(!spiTransmit.io.active) {
            goto(readyState)
          }
        } otherwise {
          spiTransmit.io.in_data.valid := True
          when(spiTransmit.io.in_data.ready) {
            dataCounter := dataCounter + 1
          }
        }
      }
    }

  }

}
