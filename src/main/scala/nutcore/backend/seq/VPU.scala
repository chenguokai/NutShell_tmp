package nutcore

import chisel3._
import chisel3.util._
import chisel3.util.experimental.BoringUtils
import utils.{LookupTree, LookupTreeDefault, SignExt, ZeroExt}

class VPUIO extends FunctionUnitIO {
    val instr = Input(UInt(32.W)) // vs1/vs2/vd
    val fuType = Input(FuType()) //
    val dmem = new VMEMIO(userBits = DVMemUserBits)
    val cfg = Flipped(new VCFGIO)
    // vmu may not exec mmio,
}


// TODO: how to deal with VPU-exception, for VPU is also at EXU-stage
class VPU(implicit val p: NutCoreConfig) extends Module with HasVectorParameter with HasVRegFileParameter {
    val io = IO(new VPUIO)
    
    val (valid, src1, src2, func, fuType) = (io.in.valid, io.in.bits.src1, io.in.bits.src2, io.in.bits.func, io.fuType)
    
    def access(valid: Bool, src1: UInt, src2: UInt, func: UInt, fuType: UInt): UInt = {
        this.valid := valid
        this.src1 := src1
        this.src2 := src2
        this.func := func
        this.fuType := fuType
        io.out.bits // move vsetvl(i) from alu to vpu later
    }
    
    val instr = io.instr
    val (vm, vs1, vs2, vd, vs3) = (instr(25), instr(19, 15), instr(24, 20), instr(11, 7), instr(11, 7))

    val vseq = Module(new VSeq)
    vseq.access(valid, src1, src2, func, fuType)
    vseq.io.in <> io

    val vmu = Module(new VMU)
    //val vxu = Module(new VXU)
    // val vrf = new VectorRegFile
    val vexu = for (i <- 0 until NLane) yield {
        val ret = Module(new VEXU(i))
        ret
    }
    
    val vmdu = for (i <- 0 until NLane) yield {
        val ret = Module(new VMDU_Embedded(i))
        ret
    }
    
    val vexu_in_ready_lane = Wire(Vec(NLane, UInt(1.W)))
    val vexu_out_valid_lane = Wire(Vec(NLane, UInt(1.W)))
    for (i <- 0 until NLane) {
        vexu_in_ready_lane(i) := vexu(i).io.in.ready
        vexu_out_valid_lane(i) := vexu(i).io.out.valid
    }
    val vexu_in_ready = vexu_in_ready_lane.reduce(Cat(_, _)).andR()
    val vexu_out_valid = vexu_out_valid_lane.reduce(Cat(_, _)).andR()
    
    val vmdu_in_ready_lane = Wire(Vec(NLane, UInt(1.W)))
    val vmdu_out_valid_lane = Wire(Vec(NLane, UInt(1.W)))
    for (i <- 0 until NLane) {
        vmdu_in_ready_lane(i) := vmdu(i).io.in.ready
        vmdu_out_valid_lane(i) := vmdu(i).io.out.valid
    }
    val vmdu_in_ready = vmdu_in_ready_lane.reduce(Cat(_, _)).andR()
    val vmdu_out_valid = vmdu_out_valid_lane.reduce(Cat(_, _)).andR()
    
    val vrf = for (i <- 0 until NLane) yield {
        val rf_lane = Module(new VRegArbiter())
        
        // Bank 0: VMU v0
        rf_lane.io.read.raddr(0) := 0.U
        // Bank 1: VMU src2
        rf_lane.io.read.raddr(1) := vmu.io.vreg.vs2
        // Bank 2: VMU src3
        rf_lane.io.read.raddr(2) := vmu.io.vreg.vs3
        // Bank 3: VEXU v0
        rf_lane.io.read.raddr(3) := 0.U
        // Bank 4: VEXU src1
        rf_lane.io.read.raddr(4) := vexu(i).io.vreg.vs1
        // Bank 5: VEXU src2
        rf_lane.io.read.raddr(5) := vexu(i).io.vreg.vs2
        // Bank 6: VMDU src1
        rf_lane.io.read.raddr(6) := vmdu(i).io.vreg.vs1
        // Bank 7: VMDU src2
        rf_lane.io.read.raddr(7) := vmdu(i).io.vreg.vs2
        // Bank 8: VMDU src3
        rf_lane.io.read.raddr(8) := vmdu(i).io.vreg.vd
        rf_lane.io.read.raddr(9) := 0.U
        
        // Write port 0: VMU
        rf_lane.io.write.waddr(0) := vmu.io.vreg.vd
        rf_lane.io.write.wdata(0) := vmu.io.vreg.wdata(i * 64 + 63, i * 64)
        rf_lane.io.write.wmask(0) := vmu.io.vreg.wmask(i * 8 + 7, i * 8)
        rf_lane.io.write.wen(0) <> vmu.io.vreg.wen
        // Write port 1: VEXU
        rf_lane.io.write.waddr(1) := vexu(i).io.vreg.vd
        rf_lane.io.write.wdata(1) := vexu(i).io.vreg.wdata
        rf_lane.io.write.wmask(1) := vexu(i).io.vreg.wmask
        rf_lane.io.write.wen(1) <> vexu(i).io.vreg.wen
        // Write port 2: VMDU
        rf_lane.io.write.waddr(2) := vmdu(i).io.vreg.vd
        rf_lane.io.write.wdata(2) := vmdu(i).io.vreg.wdata
        rf_lane.io.write.wmask(2) := vmdu(i).io.vreg.wmask
        rf_lane.io.write.wen(2) <> vmdu(i).io.vreg.wen
        
        rf_lane
    }
    if (!p.FPGAPlatform) {
        val vrf_ref = Wire(Vec(128, UInt(64.W)))
        for (i <- 0 until NVReg) {
            vrf_ref(i * 4) := vrf(0).io.debug(i)
            vrf_ref(i * 4 + 1) := vrf(1).io.debug(i)
            vrf_ref(i * 4 + 2) := vrf(2).io.debug(i)
            vrf_ref(i * 4 + 3) := vrf(3).io.debug(i)
        }
        BoringUtils.addSource(vrf_ref, "difftestVectorRegs")
    }
    
    // Bank 0: VMU v0
    val vmu_v0 = Cat(vrf(3).io.read.rdata(0), vrf(2).io.read.rdata(0), vrf(1).io.read.rdata(0), vrf(0).io.read.rdata(0)) // mask
    
    // VMU src1: DontCare
    // Bank 1: VMU src2
    val vmu_src2 = Cat(vrf(3).io.read.rdata(1), vrf(2).io.read.rdata(1), vrf(1).io.read.rdata(1),  vrf(0).io.read.rdata(1))

    // Bank 2: VMU src3
    val vmu_src3 = Cat(vrf(3).io.read.rdata(2), vrf(2).io.read.rdata(2), vrf(1).io.read.rdata(2), vrf(0).io.read.rdata(2))

    // Bank 3: VXU v0
    val vexu_v0 = Cat(vrf(3).io.read.rdata(3), vrf(2).io.read.rdata(3), vrf(1).io.read.rdata(3), vrf(0).io.read.rdata(3)) // mask
    // Bank 4: VXU src1
    // Bank 5: VXU src2
    for (i <- 0 until NLane) {
        vexu(i).io.vreg.vsrc1 := vrf(i).io.read.rdata(4)
        vexu(i).io.vreg.vsrc2 := vrf(i).io.read.rdata(5)
        vexu(i).io.vreg.vsrc3 := DontCare
    
        // Bank 6: VMDU src1
        // Bank 7: VMDU src2
        // Bank 8: VMDU src3
        vmdu(i).io.vreg.vsrc1 := vrf(i).io.read.rdata(6)
        vmdu(i).io.vreg.vsrc2 := vrf(i).io.read.rdata(7)
        vmdu(i).io.vreg.vsrc3 := vrf(i).io.read.rdata(8)
    }
   
    // Bank 9: VMDU v0
    val vmdu_v0 = Cat(vrf(3).io.read.rdata(9), vrf(2).io.read.rdata(9), vrf(1).io.read.rdata(9), vrf(0).io.read.rdata(9))
    // Bank 9: VSLDU src
    
    vmu.access(FuType.vmu === vseq.io.out(fu_vmu).fuType && vseq.io.out(fu_vmu).valid, vseq.io.out(fu_vmu).src1, vseq.io.out(fu_vmu).src2, vseq.io.out(fu_vmu).func(5, 0), vseq.io.out(fu_vmu).vs2, vseq.io.out(fu_vmu).vd)
    vmu.io.dmem <> vseq.io.dmem
    vmu.io.cfg.vlen := vseq.io.out(fu_vmu).vlen
    vmu.io.cfg.vsew := vseq.io.out(fu_vmu).vsew
    vmu.io.cfg.vlmul := vseq.io.out(fu_vmu).vlmul
    vmu.io.vm := vseq.io.out(fu_vmu).vm
    vmu.io.out.ready := 1.B // always ready to commit

    for (i <- 0 until NLane) {
        vexu(i).access(FuType.vxu === vseq.io.out(fu_vxu).fuType && vseq.io.out(fu_vxu).valid, vseq.io.out(fu_vxu).src1, vseq.io.out(fu_vxu).src2, vseq.io.out(fu_vxu).func, vseq.io.out(fu_vxu).vs1, vseq.io.out(fu_vxu).vs2, vseq.io.out(fu_vxu).vd)
        // vexu(i).io.cfg <> io.cfg
        vexu(i).io.cfg.vsew := vseq.io.out(fu_vxu).vsew
        vexu(i).io.cfg.vlen := vseq.io.out(fu_vxu).vlen
        vexu(i).io.cfg.vlmul := vseq.io.out(fu_vxu).vlmul

        vexu(i).io.vm := vseq.io.out(fu_vxu).vm
        vexu(i).io.out.ready := 1.B // always ready to commit
        vexu(i).io.vreg.v0 := vexu_v0
        
        vmdu(i).access(FuType.vmdu === vseq.io.out(fu_vmdu).fuType && vseq.io.out(fu_vmdu).valid, vseq.io.out(fu_vmdu).src1, vseq.io.out(fu_vmdu).src2, vseq.io.out(fu_vmdu).func, vseq.io.out(fu_vmdu).vs1, vseq.io.out(fu_vmdu).vs2, vseq.io.out(fu_vmdu).vd)
        // vmdu(i).io.cfg <> io.cfg
        vmdu(i).io.cfg.vsew := vseq.io.out(fu_vmdu).vsew
        vmdu(i).io.cfg.vlen := vseq.io.out(fu_vmdu).vlen
        vmdu(i).io.cfg.vlmul := vseq.io.out(fu_vmdu).vlmul
        vmdu(i).io.vm := vseq.io.out(fu_vmdu).vm
        vmdu(i).io.out.ready := 1.B // always ready to commit
        vmdu(i).io.vreg.v0 := vmdu_v0
    }
    
    /*
    val vrfSrc1 = Mux1H(List(
        (vmu.io.in.valid || vmu.io.out.bits) -> 0.U,
        (vxu.io.in.valid || vxu.io.out.bits.busy) -> vxu.io.vreg.vs1
    ))
    val vrfSrc2 = Mux1H(List(
        (vmu.io.in.valid || vmu.io.out.bits) -> vmu.io.vreg.vs2,
        (vxu.io.in.valid || vxu.io.out.bits.busy) -> vxu.io.vreg.vs2
    
    ))
    val vrfSrc3 = Mux1H(List(
        (vmu.io.in.valid || vmu.io.out.bits) -> vmu.io.vreg.vs3,
        (vxu.io.in.valid || vxu.io.out.bits.busy) -> vxu.io.vreg.vs3
    ))
    val vrfDest = Mux1H(List(
        vmu.io.out.bits -> vmu.io.vreg.vd,
        vxu.io.out.bits.busy -> vxu.io.vreg.vd
    ))
    val vWdata = Mux1H(List(
        vmu.io.out.bits -> vmu.io.vreg.wdata,
        vxu.io.out.bits.busy -> vxu.io.vreg.wdata
    ))
    val vWmask = Mux1H(List(
        vmu.io.out.bits -> vmu.io.vreg.wmask,
        vxu.io.out.bits.busy -> vxu.io.vreg.wmask
    ))
    val vWen = vmu.io.vreg.wen || vxu.io.vreg.wen
    assert(!vmu.io.vreg.wen || !vxu.io.vreg.wen)
     */
    
    
    vmu.io.vreg.v0 := vmu_v0
    vmu.io.vreg.vsrc2 := vmu_src2
    vmu.io.vreg.vsrc3 := vmu_src3
    vmu.io.vreg.vsrc1 := DontCare
    
    
    
    /*
    vxu.io.vreg.vsrc1 := vsrc1
    vxu.io.vreg.vsrc2 := vsrc2
    vxu.io.vreg.vsrc3 := vsrc3
     */
    
    // assert(!vmu.io.out.valid || !vxu.io.out.valid)
    vseq.io.out_valid(fu_vmu) := vmu.io.out.valid
    vseq.io.out_valid(fu_vxu) := vexu_out_valid
    vseq.io.out_valid(fu_vmdu) := vmdu_out_valid

    io.out.valid := vseq.io.in.out.valid
    io.out.bits := DontCare //vxu.io.out.bits.scala
    io.in.ready := vseq.io.in.in.ready
    
    /*
    BoringUtils.addSource(io.out.fire(), "perfCntCondMvInstr")
    BoringUtils.addSource(io.out.fire() && vxu.io.out.valid, "perfCntCondMvxInstr")
    BoringUtils.addSource(io.out.fire() && vmu.io.out.valid, "perfCntCondMvmInstr")
    BoringUtils.addSource(!io.in.ready, "perfCntCondMvCycle")
    BoringUtils.addSource(!vxu.io.in.ready, "perfCntCondMvxCycle")
    BoringUtils.addSource(!vmu.io.in.ready, "perfCntCondMvmCycle")
    BoringUtils.addSource(io.dmem.mem.req.fire(), "perfCntCondMvmReq")
    BoringUtils.addSource(io.dmem.mem.req.fire() && io.dmem.mem.isRead(), "perfCntCondMvmReqLd")
    BoringUtils.addSource(io.dmem.mem.req.fire() && io.dmem.mem.isWrite(), "perfCntCondMvmReqSt")
    BoringUtils.addSource(!vmu.io.in.ready && VMUOpType.isLoad(func), "perfCntCondMvmLdStall")
    BoringUtils.addSource(!vmu.io.in.ready && VMUOpType.isStore(func), "perfCntCondMvmStStall")
    BoringUtils.addSource(io.in.fire() && vmu.io.in.valid && VMUOpType.isLoad(func), "perfCntCondMvmLdInstr")
    BoringUtils.addSource(io.in.fire() && vmu.io.in.valid && VMUOpType.isStore(func), "perfCntCondMvmStInstr")
     */
}