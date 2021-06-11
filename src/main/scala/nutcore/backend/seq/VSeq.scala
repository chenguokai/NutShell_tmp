package nutcore

import Chisel.Cat
import utils.LookupTree
import chisel3._
import chisel3.util.experimental.BoringUtils

class VSeqOut extends NutCoreBundle with HasVectorParameter {
    val vs1 = UInt(5.W)
    val vs2 = UInt(5.W)
    val vm = UInt(1.W)
    val vd = UInt(5.W)
    val vsew = UInt(2.W)
    val vlmul = UInt(2.W)
    val vlen = UInt(XLEN.W)
    val func = FuOpType()
    val fuType = FuType()
    val src1 = UInt(XLEN.W)
    val src2 = UInt(XLEN.W)
    val valid = Bool()
}

class VSeqIO extends NutCoreBundle with HasVectorParameter {
    val in = new VPUIO
    // vmu vmdu vxu
    val out = Output(Vec(nVFU, new VSeqOut))
    val dmem = Flipped(new VMEMIO(userBits = DVMemUserBits))
    val out_valid = Input(Vec(nVFU, Bool()))
    // todo: vsldu
}

class VSeq extends NutCoreModule with HasVectorParameter {
    val io = IO(new VSeqIO)

    io.dmem <> io.in.dmem
    // note that currently we do not have any io.out from VPU, thus we do not need to care about output arbiter
    
    val valid_hold = RegInit(0.U(3.W))
    
    val in_ready = RegInit(1.U)


    // shoud we really care about in_ready ?
    io.in.in.ready := in_ready
    
    val (valid, src1, src2, func, fuType, vm) = (io.in.in.valid, io.in.in.bits.src1, io.in.in.bits.src2, io.in.in.bits.func, io.in.fuType, io.in.instr(25))
    
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
    val current_fu_valid = RegInit(VecInit(Seq.fill(nVFU)(0.B)))
    val current_vs3_valid = RegInit(VecInit(Seq.fill(nVFU)(0.B)))
    val current_vs1_valid = RegInit(VecInit(Seq.fill(nVFU)(0.B)))
    val current_vs2_valid = RegInit(VecInit(Seq.fill(nVFU)(0.B)))
    val current_vd_valid = RegInit(VecInit(Seq.fill(nVFU)(0.B)))
    val current_vs1 = Reg(Vec(nVFU, UInt(5.W)))
    val current_vs2 = Reg(Vec(nVFU, UInt(5.W)))
    val current_vd = Reg(Vec(nVFU, UInt(5.W)))
    val current_vsew = Reg(Vec(nVFU, UInt(2.W)))
    val current_fuType = Reg(Vec(nVFU, FuType()))
    val current_func = Reg(Vec(nVFU, FuOpType()))
    val current_src1 = Reg(Vec(nVFU, UInt(XLEN.W)))
    val current_src2 = Reg(Vec(nVFU, UInt(XLEN.W)))
    val current_vm = Reg(Vec(nVFU, UInt(1.W)))
    val current_vlmul = Reg(Vec(nVFU, UInt(2.W)))
    val current_vlen = Reg(Vec(nVFU, UInt(XLEN.W)))
    
    
    
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
    
    val vs1_taken = Wire(Vec(nVFU, UInt(NUMVREG.W)))
    val vs2_taken = Wire(Vec(nVFU, UInt(NUMVREG.W)))
    val vs3_taken = Wire(Vec(nVFU, UInt(NUMVREG.W)))
    val vd_taken = Wire(Vec(nVFU, UInt(NUMVREG.W)))
    
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
        vs3_taken(i) := taken(current_vd(i), current_vsew(i))
        vd_taken(i) := taken(current_vd(i), current_vsew(i))
    }

    // RAW conflicts
    
    // next vs1 and all vd s
    val raw_vs1_vd = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        raw_vs1_vd(i) := current_fu_valid(i) && next_vs1_valid && (next_vs1_taken & vd_taken(i) ) =/= 0.U
    }
    val raw_vs1 = raw_vs1_vd.reduce(Cat(_, _)).orR() & next_vs1_valid
    // next vs2 and all vd s
    val raw_vs2_vd = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        raw_vs2_vd(i) := current_fu_valid(i) && (next_vs2_taken & vd_taken(i) ) =/= 0.U
    }
    val raw_vs2 = raw_vs2_vd.reduce(Cat(_, _)).orR()
    // next vs3 and all vd s
    val raw_vs3_vd = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        raw_vs3_vd(i) := current_fu_valid(i) && next_vs3_valid && (next_vs3_taken & vd_taken(i) ) =/= 0.U
    }
    val raw_vs3 = raw_vs3_vd.reduce(Cat(_, _)).orR() & next_vs3_valid
    // next v0 and all vd s
    val raw_v0_vd = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        raw_v0_vd(i) := current_fu_valid(i) && current_vm(i).asBool() && (next_v0_taken & vd_taken(i) ) =/= 0.U
    }
    val raw_v0 = raw_v0_vd.reduce(Cat(_, _)).orR() & next_v0_read
    
    val raw_conflict = raw_vs1 | raw_vs2 | raw_vs3 | raw_v0
    // WAR conflicts
    
    // next vd and all vs1 s
    val war_vd_vs1 = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        war_vd_vs1(i) := (current_fu_valid(i) && current_vs1_valid(i) && (next_vd_taken & vs1_taken(i)) =/= 0.U).asUInt()
    }
    val war_vs1 = war_vd_vs1.reduce(Cat(_, _)).orR()
    // next vd and all vs2 s
    val war_vd_vs2 = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        war_vd_vs2(i) := (current_fu_valid(i) && current_vs2_valid(i) && (next_vd_taken & vs2_taken(i)) =/= 0.U).asUInt()
    }
    val war_vs2 = war_vd_vs2.reduce(Cat(_, _)).orR()
    // next vs3 and all vs3 s
    val war_vd_vs3 = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        war_vd_vs3(i) := (current_fu_valid(i) && current_vs3_valid(i) && (next_vd_taken & vs3_taken(i)) =/= 0.U).asUInt()
    }
    val war_vs3 = war_vd_vs3.reduce(Cat(_, _)).orR()

    // next vd and all v0 s
    val war_vd_v0 = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        war_vd_v0(i) := (current_fu_valid(i) && current_vm(i).asBool() && ((next_vd_taken & next_v0_taken) =/= 0.U)).asUInt()
    }
    val war_v0 = war_vd_v0.reduce(Cat(_, _)).orR()

    val war_conflict = war_vs1 | war_vs2 | war_vs3 | war_v0
    // WAW conflicts
    
    // next vd and all vd s
    val waw_vd_vd = Wire(Vec(nVFU, UInt(1.W)))
    for (i <- 0 until nVFU) {
        waw_vd_vd(i) := (current_fu_valid(i) && (next_vd_taken & vd_taken(i)) =/= 0.U).asUInt()
    }
    val waw_conflict = waw_vd_vd.reduce(Cat(_, _)).orR()
    
    val conflict = war_conflict | raw_conflict | waw_conflict
    
    // a valid where fire() has been triggered
    // val fired = Vec(nVFU, RegInit(1.B))

    when (valid && !conflict && fuType === FuType.vmu && (!current_fu_valid(fu_vmu) || (current_fu_valid(fu_vmu) && io.out_valid(fu_vmu)))) {
        in_ready := 1.U
    } .elsewhen (valid && !conflict && fuType === FuType.vxu && (!current_fu_valid(fu_vxu) || (current_fu_valid(fu_vxu) && io.out_valid(fu_vxu)))) {
        in_ready := 1.U
    } .elsewhen (valid && !conflict && fuType === FuType.vmdu && (!current_fu_valid(fu_vmdu) || (current_fu_valid(fu_vmdu) && io.out_valid(fu_vmdu)))) {
        in_ready := 1.U
    } .otherwise {
        in_ready := 0.U
    }

    when (valid && !conflict && fuType === FuType.vmu && (!current_fu_valid(fu_vmu) || (current_fu_valid(fu_vmu) && io.out_valid(fu_vmu))) ) {
        current_fu_valid(fu_vmu) := 1.B
        current_vs3_valid(fu_vmu) := 1.B
        current_vs2_valid(fu_vmu) := 1.B
        current_vs1_valid(fu_vmu) := 0.B
        current_vd_valid(fu_vmu) := 1.B
        current_vs1(fu_vmu) := next_vs1
        current_vs2(fu_vmu) := next_vs2
        current_vd(fu_vmu) := next_vd
        current_vsew(fu_vmu) := next_vsew
        current_vlmul(fu_vmu) := next_vlmul
        current_vlen(fu_vmu) := next_vlen
        current_src1(fu_vmu) := src1
        current_src2(fu_vmu) := src2
        current_func(fu_vmu) := func
        current_fuType(fu_vmu) := io.in.fuType
        current_vm(fu_vmu) := vm

    } .elsewhen (io.out_valid(fu_vmu)) {
        // this function unit is being emptied
        current_fu_valid(fu_vmu) := 0.B
    }
    when (valid && !conflict && fuType === FuType.vxu && (!current_fu_valid(fu_vxu) || (current_fu_valid(fu_vxu) && io.out_valid(fu_vxu)))) {
        current_fu_valid(fu_vxu) := 1.B
        current_vs3_valid(fu_vxu) := 0.B
        current_vs2_valid(fu_vxu) := 1.B
        current_vs1_valid(fu_vxu) := 1.B
        current_vd_valid(fu_vxu) := 1.B
        current_vs1(fu_vxu) := next_vs1
        current_vs2(fu_vxu) := next_vs2
        current_vd(fu_vxu) := next_vd
        current_vsew(fu_vxu) := next_vsew
        current_vlmul(fu_vxu) := next_vlmul
        current_vlen(fu_vxu) := next_vlen
        current_src1(fu_vxu) := src1
        current_src2(fu_vxu) := src2
        current_func(fu_vxu) := func
        current_fuType(fu_vxu) := io.in.fuType
        current_vm(fu_vxu) := vm
    } .elsewhen (io.out_valid(fu_vxu)) {
        // this function unit is being emptied
        current_fu_valid(fu_vxu) := 0.B
    }
    when (valid && !conflict && fuType === FuType.vmdu && (!current_fu_valid(fu_vmdu) || (current_fu_valid(fu_vmdu) && io.out_valid(fu_vmdu)))) {
        current_fu_valid(fu_vmdu) := 1.B
        current_vs3_valid(fu_vmdu) := 1.B
        current_vs2_valid(fu_vmdu) := 1.B
        current_vs1_valid(fu_vmdu) := 1.B
        current_vd_valid(fu_vmdu) := 1.B
        current_vs1(fu_vmdu) := next_vs1
        current_vs2(fu_vmdu) := next_vs2
        current_vd(fu_vmdu) := next_vd
        current_vsew(fu_vmdu) := next_vsew
        current_vlmul(fu_vmdu) := next_vlmul
        current_vlen(fu_vmdu) := next_vlen
        current_src1(fu_vmdu) := src1
        current_src2(fu_vmdu) := src2
        current_func(fu_vmdu) := func
        current_fuType(fu_vmdu) := io.in.fuType
        current_vm(fu_vmdu) := vm
    } .elsewhen (io.out_valid(fu_vmdu)) {
        // this functiob unit is being emptied
        current_fu_valid(fu_vmdu) := 0.B
    }

    
    
    val out_valid_0 = Wire(Bool())
    val out_valid_1 = Wire(Bool())
    val out_valid_2 = Wire(Bool())
    val out_valid_3 = Wire(Bool())
    
    val out_valid = Wire(Vec(nVFU, Bool()))
    for (i <- 0 until nVFU) {
        out_valid(i) := io.out_valid(i)
    }
    out_valid_0 := (!out_valid(0)) & (!out_valid(1)) & (!out_valid(2))
    out_valid_1 := ((!out_valid(0)) & (!out_valid(1)) & (out_valid(2))) | ((!out_valid(0)) & (out_valid(1)) & (!out_valid(2))) | ((out_valid(0)) & (!out_valid(1)) & (!out_valid(2)))
    out_valid_2 := ((!out_valid(0)) & (out_valid(1)) & (out_valid(2))) | ((out_valid(0)) & (!out_valid(1)) & (out_valid(2))) | ((out_valid(0)) & (out_valid(1)) & (!out_valid(2)))
    out_valid_3 := out_valid(0) & out_valid(1) & out_valid(2)

    // here we assume that write back ready is always high
    io.in.out.valid := in_ready // out_valid(0) | out_valid(1) | out_valid(2) | valid_hold =/= 0.U
    
    when (out_valid_1) {
        valid_hold := valid_hold
    } .elsewhen (out_valid_2) {
        valid_hold := valid_hold + 1.U
    } .elsewhen (out_valid_3) {
        valid_hold := valid_hold + 2.U
    } .elsewhen (valid_hold =/= 0.U) {
        valid_hold := valid_hold - 1.U
    }

    for (i <- 0 until nVFU) {
        io.out(i).valid := current_fu_valid(i)
        io.out(i).vs1 := current_vs1(i)
        io.out(i).vs2 := current_vs2(i)
        io.out(i).vm := current_vm(i)
        io.out(i).vd := current_vd(i)
        io.out(i).vsew := current_vsew(i)
        io.out(i).vlmul := current_vlmul(i)
        io.out(i).vlen := current_vlen(i)
        io.out(i).func := current_func(i)
        io.out(i).fuType := current_fuType(i)
        io.out(i).src1 := current_src1(i)
        io.out(i).src2 := current_src2(i)
    }
    io.in.out.bits := DontCare


    BoringUtils.addSource(current_fu_valid(fu_vxu) | current_fu_valid(fu_vmu) | current_fu_valid(fu_vmdu), "perfCntMvCycle")
    BoringUtils.addSource(current_fu_valid(fu_vmu), "perfCntMvMemCycle")

}
