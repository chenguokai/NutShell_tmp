package nutcore

import Chisel.{Decoupled, Log2}
import chisel3._

trait HasVRegFileParameter {
    val NVReg = 32
    val NBank = 8
}

class VRegFileIO extends NutCoreBundle with HasVRegFileParameter  {
    val raddr = Output(Vec(NBank, UInt(5.W)))
    val rdata = Input(Vec(NBank, UInt(XLEN.W)))
    val waddr = Output(UInt(5.W))
    val wdata = Output(UInt(XLEN.W))
    val wen = Output(UInt(1.W))
}

class VRegFileSingleBankIO extends NutCoreBundle with HasVRegFileParameter {
    val raddr = Output(UInt(5.W))
    val rdata = Input(UInt(XLEN.W))
    val waddr = Output(UInt(5.W))
    val wdata = Output(UInt(XLEN.W))
    val wen = Output(UInt(1.W))
}

class VRegFileSingleBank extends NutCoreModule with HasVRegFileParameter {
    val io = IO(Flipped(new VRegFileSingleBankIO))
    // bank cannot be wrapped inside
    val rf = Reg(Vec(NVReg, UInt(XLEN.W)))
    // leave bank arbeiter logic outside reg file
    io.rdata := rf(io.raddr)
    when (io.wen.asBool()) {
        rf(io.waddr) := io.wdata
    }
}

class VRegFile extends NutCoreModule with HasVRegFileParameter {
    val io = IO(Flipped(new VRegFileIO))
    
    // a vector of single bank register file
    // val rf_vec = Vec(NBank, Module(new VRegFileSingleBank).io)
    val rf_vec = for (i <- 0 until NBank) yield
        {
            val rf_unit = Module(new VRegFileSingleBank)
            // any wiring or other logic can go here
            rf_unit
        }
    // todo: review the register file design
    // only design a single write port
    for (i <- 0 until NBank) {
        rf_vec(i).io.waddr := io.waddr
        rf_vec(i).io.wdata := io.wdata
        rf_vec(i).io.wen := io.wen
        rf_vec(i).io.raddr := io.raddr(i)
        io.rdata(i) := rf_vec(i).io.rdata
    }
}

// Debug purpose only
object VRegFileGen extends App {
    chisel3.Driver.execute(args, () => new VRegFile)
}