package nutcore

import Chisel.Cat
import utils.LookupTree
import chisel3._

class VSeqIO extends NutCoreBundle {
    val in = new VPUIO
    // vmu vmdu vxu
    val out = Flipped(Vec(3, new VPUIO))
    // todo: vsldu
}

class VSeq extends NutCoreModule with HasVectorParameter {
    val io = IO(new VSeqIO)
    
    // note that currently we do not have any io.out from VPU, thus we do not need to care about output arbiter
    
    val valid_hold = RegInit(0.U(3.W))
    
    val in_ready = RegInit(1.U)
    
    // shoud we really care about in_ready ?
    io.in.in.ready := in_ready
    
    val (valid, src1, src2, func, fuType) = (io.in.in.valid, io.in.in.bits.src1, io.in.in.bits.src2, io.in.in.bits.func, io.in.fuType)
    
    def access(valid: Bool, src1: UInt, src2: UInt, func: UInt, fuType: UInt): UInt = {
        this.valid := valid
        this.src1 := src1
        this.src2 := src2
        this.func := func
        this.fuType := fuType
        io.in.out.bits
    }
    
    val next_vs1 = Wire(UInt(5.W))
    val next_vs2 = Wire(UInt(5.W))
    val next_vs1_valid = Wire(Bool())
    val next_vs3_valid = Wire(Bool())
    val next_vd = Wire(UInt(5.W))
    val next_vm = Wire(UInt(1.W))
    val next_v0_read = Wire(Bool())
    
    val next_vsew = Wire(UInt(2.W))
    val next_vlmul = Wire(UInt(2.W))
    val next_vlen = Wire(UInt(XLEN.W))
    
    next_vsew := io.in.cfg.vsew
    next_vlen := io.in.cfg.vlen
    next_vlmul := io.in.cfg.vlmul
    
    next_vm := io.in.instr(25)
    next_vs1 := io.in.instr(19, 15)
    next_vs2 := io.in.instr(24, 20)
    next_vd := io.in.instr(11, 7)
    next_vs3_valid := fuType === FuType.vmu | fuType === FuType.vmdu
    next_vs1_valid := fuType === FuType.vxu | fuType === FuType.vmdu
    next_v0_read := next_vm =/= 0.U
    
    
    
    
    // for three Function Units
    val current_fu_valid = Vec(nVFU, RegInit(0.B))
    val current_vs3_valid = Vec(nVFU, RegInit(0.B))
    val current_vs1_valid = Vec(nVFU, Reg(Bool()))
    val current_vs2_valid = Vec(nVFU, Reg(Bool()))
    val current_vd_valid = Vec(nVFU, Reg(Bool()))
    val current_vs1 = Vec(nVFU, Reg(UInt(5.W)))
    val current_vs2 = Vec(nVFU, Reg(UInt(5.W)))
    val current_vd = Vec(nVFU, Reg(UInt(5.W)))
    val current_vsew = Vec(nVFU, Reg(UInt(2.W)))
    val current_vlmul = Vec(nVFU, Reg(UInt(2.W)))
    val current_vlen = Vec(nVFU, Reg(UInt(XLEN.W)))
    
    
    
    def taken(vreg: UInt, vlmul: UInt) = {
        val ret = Wire(UInt(NUMVREG.W))
        
        val ret_1 = "b1".U << vreg
        val ret_2 = "b11".U << vreg
        val ret_3 = "b111".U << vreg
        val ret_4 = "b1111".U << vreg
        
        ret := LookupTree(vlmul, List(
            "b00".U -> ret_1,
            "b01".U -> ret_2,
            "b10".U -> ret_3,
            "b11".U -> ret_4
        ))
        
        ret
    }
    
    val vs1_taken = Vec(nVFU, Wire(UInt(NUMVREG.W)))
    val vs2_taken = Vec(nVFU, Wire(UInt(NUMVREG.W)))
    val vs3_taken = Vec(nVFU, Wire(UInt(NUMVREG.W)))
    val vd_taken = Vec(nVFU, Wire(UInt(NUMVREG.W)))
    
    val next_vs1_taken = Wire(UInt(NUMVREG.W))
    val next_vs2_taken = Wire(UInt(NUMVREG.W))
    val next_vs3_taken = Wire(UInt(NUMVREG.W))
    val next_vd_taken = Wire(UInt(NUMVREG.W))
    
    next_vs1_taken := taken(next_vs1, next_vsew)
    next_vs2_taken := taken(next_vs2, next_vsew)
    next_vs3_taken := taken(next_vd, next_vsew)
    next_vd_taken := taken(next_vd, next_vsew)
    val next_v0_taken = 1.U
    
    for (i <- 0 until nVFU) {
        // todo: optimize: update from next_
        vs1_taken(i) := taken(current_vs1(i), current_vsew(i))
        vs2_taken(i) := taken(current_vs2(i), current_vsew(i))
        vd_taken(i) := taken(current_vs2(i), current_vsew(i))
        vd_taken(i) := taken(current_vd(i), current_vsew(i))
    }
    
    // RAW conflicts
    
    // next vs1 and all vd s
    val raw_vs1_vd = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        raw_vs1_vd(i) := current_fu_valid(i) && (next_vs1_taken & vd_taken(i) ) =/= 0.U
    }
    val raw_vs1 = raw_vs1_vd.reduce(Cat(_, _)).orR() & next_vs1_valid
    // next vs2 and all vd s
    val raw_vs2_vd = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        raw_vs2_vd(i) := current_fu_valid(i) && (next_vs2_taken & vd_taken(i) ) =/= 0.U
    }
    val raw_vs2 = raw_vs2_vd.reduce(Cat(_, _)).orR()
    // next vs3 and all vd s
    val raw_vs3_vd = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        raw_vs3_vd(i) := current_fu_valid(i) && (next_vs3_taken & vd_taken(i) ) =/= 0.U
    }
    val raw_vs3 = raw_vs3_vd.reduce(Cat(_, _)).orR() & next_vs3_valid
    // next v0 and all vd s
    val raw_v0_vd = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        raw_v0_vd(i) := current_fu_valid(i) && (next_v0_taken & vd_taken(i) ) =/= 0.U
    }
    val raw_v0 = raw_v0_vd.reduce(Cat(_, _)).orR() & next_v0_read
    
    val raw_conflict = raw_vs1 | raw_vs2 | raw_vs3 | raw_v0
    // WAR conflicts
    
    // next vd and all vs1 s
    val war_vd_vs1 = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        war_vd_vs1 := current_fu_valid(i) && current_vs1_valid(i) && (next_vd_taken & vs1_taken(i)) =/= 0.U
    }
    val war_vs1 = war_vd_vs1.reduce(Cat(_, _)).orR()
    // next vd and all vs2 s
    val war_vd_vs2 = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        war_vd_vs2 := current_fu_valid(i) && (next_vd_taken & vs2_taken(i)) =/= 0.U
    }
    val war_vs2 = war_vd_vs2.reduce(Cat(_, _)).orR()
    // next vs3 and all vs3 s
    val war_vd_vs3 = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        war_vd_vs3 := current_fu_valid(i) && current_vs3_valid(i) && (next_vd_taken & vs3_taken(i)) =/= 0.U
    }
    val war_vs3 = war_vd_vs3.reduce(Cat(_, _)).orR()
    val war_conflict = war_vs1 | war_vs2 | war_vs3
    // WAW conflicts
    
    // next vd and all vd s
    val waw_vd_vd = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        waw_vd_vd(i) := current_fu_valid(i) && (next_vd_taken & vd_taken(i)) =/= 0.U
    }
    val waw_conflict = waw_vd_vd.reduce(Cat(_, _)).orR()
    
    val conflict = war_conflict | raw_conflict | waw_conflict
    
    // a valid where fire() has been triggered
    val fired = Vec(nVFU, RegInit(1.B))
    
    when (valid && !conflict && fuType === FuType.vmu) {
    
    } .elsewhen () {
        // this function unit is emptied
    }
    when (valid && !conflict && fuType === FuType.vmdu) {
    
    } .elsewhen ()  {
    
    }
    when (valid && !conflict && fuType === FuType.vxu) {
    
    } .elsewhen () {
    
    }
    
    
    for (i <- 0 until nVFU) {
        when (io.out(i).out.fire()) {
            fired(i) := 1.B
        } .elsewhen (fired(i) && io.out(i).out.valid) {
            fired(i) :=  0.B
        }
    }
    
    
    val out_valid_0 = Wire(Bool())
    val out_valid_1 = Wire(Bool())
    val out_valid_2 = Wire(Bool())
    val out_valid_3 = Wire(Bool())
    
    val out_valid = Vec(nVFU, Wire(Bool()))
    for (i <- 0 until nVFU) {
        out_valid(i) := fired(i) && io.out(i).out.valid
    }
    out_valid_0 := (!out_valid(0)) & (!out_valid(1)) & (!out_valid(2))
    out_valid_1 := ((!out_valid(1)) & (!out_valid(2)) & (out_valid(3))) | ((!out_valid(1)) & (out_valid(2)) & (!out_valid(3))) | ((out_valid(1)) & (!out_valid(2)) & (!out_valid(3)))
    out_valid_2 := ((!out_valid(1)) & (out_valid(2)) & (out_valid(3))) | ((out_valid(1)) & (!out_valid(2)) & (out_valid(3))) | ((out_valid(1)) & (out_valid(2)) & (!out_valid(3)))
    out_valid_3 := out_valid(1) & out_valid(2) & out_valid(3)
    
    io.in.out.valid := out_valid(0) | out_valid(1) | out_valid(2) | valid_hold =/= 0.U
    
    when (out_valid_1) {
        valid_hold := valid_hold
    } .elsewhen (out_valid_2) {
        valid_hold := valid_hold + 1.U
    } .elsewhen (out_valid_3) {
        valid_hold := valid_hold + 2.U
    } .otherwise {
        valid_hold := valid_hold - 1.U
    }
    
}
