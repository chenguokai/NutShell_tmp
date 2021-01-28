package nutcore

import chisel3._
import chisel3.util._
import utils.{LookupTree, LookupTreeDefault, SignExt, ZeroExt}

class SingleIO(val Width: Int = 64) extends NutCoreBundle {
    val in = Flipped(Decoupled(new Bundle {
        val src1 = Output(UInt(Width.W))
        val src2 = Output(UInt(Width.W))
        val src3 = Output(UInt(Width.W))
        val func = Output(FuOpType())
        val vsew = Output(UInt(2.W))
        // val shamt = Output(UInt(log2Ceil(Width).W))
        val mask = Output(Bool())
    }))
    val out = Decoupled(Output(UInt(Width.W)))
}


class Single(val Width: Int = 64) extends NutCoreModule {
    val io = IO(new SingleIO(Width = Width))
    
    val (valid, src1, src2, src3, func, vsew, out, mask) = (io.in.valid, io.in.bits.src1, io.in.bits.src2, io.in.bits.src3, io.in.bits.func, io.in.bits.vsew, io.out.bits, io.in.bits.mask)
    def access(valid: UInt,src1: UInt, src2: UInt, src3: UInt, func: UInt, vsew: UInt, shamt: UInt, ready: Bool, mask: Bool): UInt = {
        this.valid := valid
        this.src1 := src1
        this.src2 := src2
        this.src3 := src3
        this.func := func
        this.vsew := vsew
        this.mask := mask
        io.out.ready := ready
        out
    }
    val vmdu = Module(new VMDU(Width))
    val mduOut = vmdu.access(valid = valid, src1, src2, src3, func, vsew)
    vmdu.io.out.ready := io.out.ready
        
    io.in.ready := vmdu.io.in.ready
    io.out.bits := mduOut // res
    io.out.valid := vmdu.io.out.valid
}

class ClusterIO extends NutCoreBundle {
    val in = Flipped(Decoupled(new Bundle {
        val src1 = Output(UInt(XLEN.W))
        val src2 = Output(UInt(XLEN.W))
        val src3 = Output(UInt(XLEN.W))
        val func = Output(FuOpType())
        val maskv0 = Output(UInt((XLEN/8).W))
    }))
    val out = Decoupled(Output(UInt(XLEN.W)))
    val vsew = Input(UInt(2.W))
}

class Cluster extends NutCoreModule {
    val io = IO(new ClusterIO)
    
    val (valid, ready, src1, src2, src3, func, vsew, maskv0) = (io.in.valid, io.out.ready, io.in.bits.src1, io.in.bits.src2, io.in.bits.src3, io.in.bits.func, io.vsew, io.in.bits.maskv0)
    def access(valid: Bool, src1: UInt, src2: UInt, src3: UInt, func: UInt, vsew: UInt, ready: Bool, maskv0: UInt) = {
        this.valid := valid
        this.src1 := Mux(VXUOpType.isReverse(func), src2, src1)
        this.src2 := Mux(VXUOpType.isReverse(func), src1, src2)
        this.src3 := src3
        this.func := func
        this.vsew := vsew
        this.ready := ready
        this.maskv0 := maskv0
        (io.in.ready, io.out.bits, io.out.valid)
    }
    
    // adder 8:16:8:32:8:16:8:64
    val sg0 = Module(new Single(8))
    val sg1 = Module(new Single(16))
    val sg2 = Module(new Single(8))
    val sg3 = Module(new Single(32))
    val sg4 = Module(new Single(8))
    val sg5 = Module(new Single(16))
    val sg6 = Module(new Single(8))
    val sg7 = Module(new Single(64))
    
    val isSign1 = VXUOpType.isSigned1(func)
    val isSign2 = VXUOpType.isSigned2(func)
    val shamt = LookupTree(vsew, List(
        "b00".U   ->  "b111".U,
        "b01".U   ->  "b1111".U,
        "b10".U   ->  "b11111".U,
        "b11".U   ->  "b111111".U
    ))
    
    // sg0
    val sg0_src1 = src1(63, 56)
    val sg0_src2 = src2(63, 56)
    val sg0_src3 = src3(63, 56)
    val sg0_res = sg0.access(valid, sg0_src1, sg0_src2, sg0_src3, func, vsew, shamt, ready, maskv0(7))
    // sg1
    val sg1_src1 = Mux(vsew(0), src1(63, 48), Mux(isSign1, SignExt(src1(55, 48), 16), ZeroExt(src1(55, 48), 16)))
    val sg1_src2 = Mux(vsew(0), src2(63, 48), Mux(isSign2, SignExt(src2(55, 48), 16), ZeroExt(src2(55, 48), 16)))
    val sg1_src3 = Mux(vsew(0), src3(63, 48), SignExt(src3(55, 48), 16))
    val sg1_res = sg1.access(valid, sg1_src1, sg1_src2, sg1_src3, func, vsew, shamt, ready, maskv0(6))
    // sg2
    val sg2_src1 = src1(47, 40)
    val sg2_src2 = src2(47, 40)
    val sg2_src3 = src3(47, 40)
    val sg2_res = sg2.access(valid, sg2_src1, sg2_src2, sg2_src3, func, vsew, shamt, ready, maskv0(5))
    // sg3
    val sg3_src1 = LookupTree(vsew, List(
        "b00".U   ->  Mux(isSign1, SignExt(src1(39, 32), 32), ZeroExt(src1(39, 32), 32)),
        "b01".U   ->  Mux(isSign1, SignExt(src1(47, 32), 32), ZeroExt(src1(47, 32), 32)),
        "b10".U   ->  src1(63, 32),
        "b11".U   ->  0.U
    ))
    val sg3_src2 = LookupTree(vsew, List(
        "b00".U   ->  Mux(isSign2, SignExt(src2(39, 32), 32), ZeroExt(src2(39, 32), 32)),
        "b01".U   ->  Mux(isSign2, SignExt(src2(47, 32), 32), ZeroExt(src2(47, 32), 32)),
        "b10".U   ->  src2(63, 32),
        "b11".U   ->  0.U
    ))
    val sg3_src3 = LookupTree(vsew, List(
        "b00".U   ->  SignExt(src3(39, 32), 32),
        "b01".U   ->  SignExt(src3(47, 32), 32),
        "b10".U   ->  src3(63, 32),
        "b11".U   ->  0.U
    ))
    val sg3_res = sg3.access(valid, sg3_src1, sg3_src2, sg3_src3, func, vsew, shamt, ready, maskv0(4))
    // sg4
    val sg4_src1 = src1(31, 24)
    val sg4_src2 = src2(31, 24)
    val sg4_src3 = src3(31, 24)
    val sg4_res = sg4.access(valid, sg4_src1, sg4_src2, sg4_src3, func, vsew, shamt, ready, maskv0(3))
    // sg5
    val sg5_src1 = Mux(vsew(0), src1(31, 16), Mux(isSign1, SignExt(src1(23, 16), 16), ZeroExt(src1(23, 16), 16)))
    val sg5_src2 = Mux(vsew(0), src2(31, 16), Mux(isSign2, SignExt(src2(23, 16), 16), ZeroExt(src2(23, 16), 16)))
    val sg5_src3 = Mux(vsew(0), src3(31, 16), SignExt(src3(23, 16), 16))
    val sg5_res = sg5.access(valid, sg5_src1, sg5_src2, sg5_src3, func, vsew, shamt, ready, maskv0(2))
    // sg6
    val sg6_src1 = src1(15, 8)
    val sg6_src2 = src2(15, 8)
    val sg6_src3 = src3(15, 8)
    val sg6_res = sg6.access(valid, sg6_src1, sg6_src2, sg6_src3, func, vsew, shamt, ready, maskv0(1))
    // sg7
    val sg7_src1 = LookupTree(vsew, List(
        "b00".U   ->  Mux(isSign1, SignExt(src1(7, 0), 64), ZeroExt(src1(7, 0), 64)),
        "b01".U   ->  Mux(isSign1, SignExt(src1(15, 0), 64), ZeroExt(src1(15, 0), 64)),
        "b10".U   ->  Mux(isSign1, SignExt(src1(31, 0), 64), ZeroExt(src1(31, 0), 64)),
        "b11".U   ->  src1
    ))
    val sg7_src2 = LookupTree(vsew, List(
        "b00".U   ->  Mux(isSign2, SignExt(src2( 7, 0), 64), ZeroExt(src2(7, 0), 64)),
        "b01".U   ->  Mux(isSign2, SignExt(src2(15, 0), 64), ZeroExt(src2(15, 0), 64)),
        "b10".U   ->  Mux(isSign2, SignExt(src2(31, 0), 64), ZeroExt(src2(31, 0), 64)),
        "b11".U   ->  src2
    ))
    val sg7_src3 = LookupTree(vsew, List(
        "b00".U   ->  SignExt(src3( 7, 0), 64),
        "b01".U   ->  SignExt(src3(15, 0), 64),
        "b10".U   ->  SignExt(src3(31, 0), 64),
        "b11".U   ->  src3
    ))
    val sg7_res = sg7.access(valid, sg7_src1, sg7_src2, sg7_src3, func, vsew, shamt, ready, maskv0(0))
    
    // res
    val mduRes = LookupTree(vsew, List(
        "b00".U   ->  Cat(sg0_res(7,0), sg1_res(7,0), sg2_res(7,0), sg3_res(7,0), sg4_res(7,0), sg5_res(7,0), sg6_res(7,0), sg7_res(7,0)),
        "b01".U   ->  Cat(sg1_res(15,0), sg3_res(15,0),  sg5_res(15,0), sg7_res(15,0)),
        "b10".U   ->  Cat(sg3_res(31,0), sg7_res(31,0)),
        "b11".U   ->  sg7_res
    ))
    
    val res = mduRes
    
    io.out.bits := res
    io.out.valid := sg7.io.out.valid
    io.in.ready := sg7.io.in.ready
}


// unify IO port between MDU and VALU
class VALU_ClusterIO extends NutCoreModule {
    val io = IO(new ClusterIO)
    val (valid, ready, src1, src2, src3, func, vsew, maskv0) = (io.in.valid, io.out.ready, io.in.bits.src1, io.in.bits.src2, io.in.bits.src3, io.in.bits.func, io.vsew, io.in.bits.maskv0)
    def access(valid: Bool, src1: UInt, src2: UInt, src3: UInt, func: UInt, vsew: UInt, ready: Bool, maskv0: UInt) = {
        this.valid := valid
        this.src1 := Mux(VXUOpType.isReverse(func), src2, src1)
        this.src2 := Mux(VXUOpType.isReverse(func), src1, src2)
        this.src3 := src3
        this.func := func
        this.vsew := vsew
        this.ready := ready
        this.maskv0 := maskv0
        (io.in.ready, io.out.bits, io.out.valid)
    }
    
    val valu = Module(new VALU)
    val valufunc = LookupTreeDefault(Cat(func, vsew), 0.U, List(
        Cat(VXUOpType.add, VMUOpType.byte) -> VALUOpType.add8,
        Cat(VXUOpType.add, VMUOpType.half) -> VALUOpType.add16,
        Cat(VXUOpType.add, VMUOpType.word) -> VALUOpType.add32,
        Cat(VXUOpType.add, VMUOpType.elem) -> VALUOpType.add64,
        Cat(VXUOpType.sub, VMUOpType.byte) -> VALUOpType.sub8,
        Cat(VXUOpType.sub, VMUOpType.half) -> VALUOpType.sub16,
        Cat(VXUOpType.sub, VMUOpType.word) -> VALUOpType.sub32,
        Cat(VXUOpType.sub, VMUOpType.elem) -> VALUOpType.sub64,
        Cat(VXUOpType.sll, VMUOpType.byte) -> VALUOpType.sll8,
        Cat(VXUOpType.sll, VMUOpType.half) -> VALUOpType.sll16,
        Cat(VXUOpType.sll, VMUOpType.word) -> VALUOpType.sll32,
        Cat(VXUOpType.sll, VMUOpType.elem) -> VALUOpType.sll64,
        Cat(VXUOpType.srl, VMUOpType.byte) -> VALUOpType.srl8,
        Cat(VXUOpType.srl, VMUOpType.half) -> VALUOpType.srl16,
        Cat(VXUOpType.srl, VMUOpType.word) -> VALUOpType.srl32,
        Cat(VXUOpType.srl, VMUOpType.elem) -> VALUOpType.srl64,
        Cat(VXUOpType.sra, VMUOpType.byte) -> VALUOpType.sra8,
        Cat(VXUOpType.sra, VMUOpType.half) -> VALUOpType.sra16,
        Cat(VXUOpType.sra, VMUOpType.word) -> VALUOpType.sra32,
        Cat(VXUOpType.sra, VMUOpType.elem) -> VALUOpType.sra64,
        Cat(VXUOpType.and, VMUOpType.byte) -> VALUOpType.and,
        Cat(VXUOpType.and, VMUOpType.half) -> VALUOpType.and,
        Cat(VXUOpType.and, VMUOpType.word) -> VALUOpType.and,
        Cat(VXUOpType.and, VMUOpType.elem) -> VALUOpType.and,
        Cat(VXUOpType.or, VMUOpType.byte) -> VALUOpType.or,
        Cat(VXUOpType.or, VMUOpType.half) -> VALUOpType.or,
        Cat(VXUOpType.or, VMUOpType.word) -> VALUOpType.or,
        Cat(VXUOpType.or, VMUOpType.elem) -> VALUOpType.or,
        Cat(VXUOpType.xor, VMUOpType.byte) -> VALUOpType.xor,
        Cat(VXUOpType.xor, VMUOpType.half) -> VALUOpType.xor,
        Cat(VXUOpType.xor, VMUOpType.word) -> VALUOpType.xor,
        Cat(VXUOpType.xor, VMUOpType.elem) -> VALUOpType.xor
        // Cat(VXUOpType.mslt, VMUOpType.byte) -> VALUOpType.sra8,
        // todo: rsub, compare (slt sltu etc)
    ))
    
    // result of ALU
    val aluRes = valu.access(src1, src2, valufunc)
    
    io.out.bits := aluRes
    io.in.ready := 1.U // always ready for new input
    io.out.valid := valid // always depends on input
}