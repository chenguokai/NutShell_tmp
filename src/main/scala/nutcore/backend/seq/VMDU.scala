package nutcore

import chisel3._
import chisel3.util._
import chisel3.util.experimental.BoringUtils
import utils._

object CSA {
    def apply(src1: UInt, src2: UInt, src3: UInt) : (UInt, UInt) = {
        val t1 = src1 ^ src2
        val sum = t1 ^ src3
        val carry = VecInit((0 until t1.getWidth).map(i => Mux(t1(i), src3(i), src1(i)))).asUInt // Mux(t1, src3, src1)
        (sum, carry)
    }
}

object CP53 {
    def apply(src1: Bool, src2: Bool, src3: Bool, src4: Bool, cin: Bool) = {
        val t1 = src1 ^ src2
        val t2 = src3 ^ src4
        val cout = Mux(t1, src3, src1)
        val t3 = t1 ^ t2
        val sum = t3 ^ cin
        val carry = Mux(t3, cin, src4)
        Cat(sum, carry, cout)
    }
}

object CP42 {
    def apply(src1: UInt, src2: UInt, src3: UInt, src4: UInt) = {
        val Width = src1.getWidth
        val all =Wire(Vec(Width, UInt(3.W)))
        all(0) := CP53(src1(0),src2(0),src3(0),src4(0), false.B)
        (1 until Width).map(i => all(i) := CP53(src1(i), src2(i), src3(i), src4(i), all(i-1)(0)))
        
        (Cat(all(Width-1)(0), VecInit((0 until Width).map(all(_)(2))).asUInt),
          Cat(VecInit((0 until Width).map(all(_)(1))).asUInt, 0.U(1.W)))
    }
}

object EXT {
    def apply(src: UInt, p: Int, s: Int) = {
        if(p==0)
            if(s==0) src
            else     Cat(src, 0.U(s.W))
        else
            if(s==0) Cat(0.U(p.W), src)
            else     Cat(0.U(p.W), src, 0.U(s.W))
    }
}

class MulAddIO(val len: Int = 65) extends NutCoreBundle {
    val in = Flipped(DecoupledIO(Vec(3, Output(UInt(len.W)))))
    val out = Decoupled(Output(UInt((2*len).W)))
    val sub = Input(Bool()) //mul-sub
}

class MulAdd65(len: Int = 65) extends NutCoreModule {
    val io = IO(new MulAddIO(len)) // TODO: len only supports 65
    /* assume signed-unsigned is hangled by MDU*/
    // val X = io.in.bits.src1
    // val Y = io.in.bits.src2
    // val Z = io.in.bits.src3
    val valid = io.in.valid
    val X = io.in.bits(0)
    val Y = io.in.bits(1)
    val Z = io.in.bits(2)
    // X * Y + Z. X,Y,Z is both 65-bits, pre-processed by out-module
    // -(X * Y) + Z
    
    // 1st stage: generate partial
    // 2nd stage: compress
    // 3rd stage: add
    // 4th stage: neg
    val s1Fire = Wire(Bool())
    val s2Fire = Wire(Bool())
    val s3Fire = Wire(Bool())
    val s4Fire = Wire(Bool())
    val s1_valid = io.in.valid
    val s2_valid = RegEnable(false.B, init = false.B, s2Fire)
    val s3_valid = RegEnable(false.B, init = false.B, s3Fire)
    val s4_valid = RegEnable(false.B, init = false.B, s4Fire)
    val subS3 = Reg(Bool()) // for mul-sub, get into s4Stage
    when (s1Fire) { s2_valid := true.B }
    when (s2Fire) { s3_valid := true.B }
    when (s3Fire && subS3) { s4_valid := true.B }
    val s1_ready = !s1_valid || s1Fire
    val s2_ready = !s2_valid || s2Fire
    val s3_ready = !s3_valid || s3Fire
    val s4_ready = !s4_valid || s4Fire
    s1Fire := s1_valid && s2_ready
    s2Fire := s2_valid && s3_ready
    s3Fire := Mux(s3_valid && !subS3, s3_valid && io.out.ready, s3_valid && s4_ready)
    s4Fire := s4_valid && io.out.ready
    
    // first stage
    val lenOdd = len % 2 == 1
    assert(lenOdd)
    val Len = if(lenOdd) len+1 else len //
    val NBooth:Int = Len / 2 //
    
    val y_extend = if(lenOdd) Cat(Y(len-1), Y, 0.U(1.W)) else Cat(Y, 0.U(1.W))
    val y_part = Wire(Vec(NBooth, UInt(3.W))) // radix-4 booth
    (0 until NBooth).map(i => y_part(i) := y_extend(i*2+2, i*2))
    val booth_x = Wire(Vec(NBooth, UInt(Len.W)))
    (0 until NBooth).map(i => booth_x(i) := LookupTree(y_part(i), List(
        "b000".U  -> 0.U(Len.W),
        "b001".U  -> SignExt(X, Len),
        "b010".U  -> SignExt(X, Len),
        "b011".U  -> SignExt(X << 1.U, Len),
        "b100".U  -> SignExt(~(X << 1.U), Len),
        "b101".U  -> ~SignExt(X, Len),
        "b110".U  -> ~SignExt(X, Len),
        "b111".U  -> 0.U(Len.W)
    ))) // FIXME: handle the case that len is not odd, 2*X is hard to handle.
    
    val isSub = VecInit((0 until NBooth).map(i => y_part(i)(2).asBool && y_part(i) =/= "b111".U)).asUInt
    val isNeg = VecInit((0 until NBooth).map(i => booth_x(i)(Len-1))).asUInt
    
    val boothWire = Wire(Vec(NBooth, UInt((Len+3).W)))
    (1 until NBooth).map(i => boothWire(i) := Cat(1.U(1.W), ~isNeg(i), booth_x(i)(Len-2, 0), 0.U(1.W), isSub(i-1)))
    boothWire(0) := Cat(0.U(2.W), 1.U(1.W), ~isNeg(0), booth_x(0)(Len-2,0))
    
    // second stage
    val booth = RegEnable(boothWire, s1Fire)
    val Z_reg = RegEnable(Z, s1Fire)
    val subS2 = RegEnable(io.sub, s1Fire)
    val Z_add = Mux(subS2, ~Z_reg, Z_reg)
    
    val last_booth = booth(NBooth-1)(Len+1, 0)
    
    // 33+1 => 32 + 2 => 16 + 2    => 8 + 2 =>
    // first layer, use 42-compressor
    val boothZ = Cat(0.U(1.W), ~Z_add(64), Z_add(64,0)) // 2+65 = 67
    val (s0_0, c0_0) = CP42(EXT(boothZ,4,0),    EXT(booth(0),2,0),  EXT(booth(1),2,0),  EXT(booth(2),0,2)) //72
    val (s0_1, c0_1) = CP42(EXT(booth(3),6,0),  EXT(booth(4),4,2),  EXT(booth(5),2,4),  EXT(booth(6),0,6)) //76
    val (s0_2, c0_2) = CP42(EXT(booth(7),6,0),  EXT(booth(8),4,2),  EXT(booth(9),2,4),  EXT(booth(10),0,6))
    val (s0_3, c0_3) = CP42(EXT(booth(11),6,0), EXT(booth(12),4,2), EXT(booth(13),2,4), EXT(booth(14),0,6))
    val (s0_4, c0_4) = CP42(EXT(booth(15),6,0), EXT(booth(16),4,2), EXT(booth(17),2,4), EXT(booth(18),0,6))
    val (s0_5, c0_5) = CP42(EXT(booth(19),6,0), EXT(booth(20),4,2), EXT(booth(21),2,4), EXT(booth(22),0,6))
    val (s0_6, c0_6) = CP42(EXT(booth(23),6,0), EXT(booth(24),4,2), EXT(booth(25),2,4), EXT(booth(26),0,6))
    val (s0_7, c0_7) = CP42(EXT(booth(27),6,0), EXT(booth(28),4,2), EXT(booth(29),2,4), EXT(booth(30),0,6))
    val add1_0 = s0_0(3,0)
    val add2_0 = c0_0(3,0)
    // second layer
    val (s1_0, c1_0) = CP42(EXT(s0_0(71,4),8,0), EXT(c0_0(71,4),8,0), s0_1, c0_1) // 77
    val (s1_1, c1_1) = CP42(EXT(s0_2,8,0), EXT(c0_2,8,0), EXT(s0_3,0,8), EXT(c0_3,0,8)) //85
    val (s1_2, c1_2) = CP42(EXT(s0_4,8,0), EXT(c0_4,8,0), EXT(s0_5,0,8), EXT(c0_5,0,8)) //85
    val (s1_3, c1_3) = CP42(EXT(s0_6,8,0), EXT(c0_6,8,0), EXT(s0_7,0,8), EXT(c0_7,0,8)) //85
    val add1_1 = s1_0(7,0)
    val add2_1 = c1_0(7,0)
    // third layer
    val (s2_0, c2_0) = CP42(EXT(s1_0(76,8),16,0), EXT(c1_0(76,8),16,0), s1_1, c1_1) // 86
    val (s2_1, c2_1) = CP42(EXT(s1_2,16,0), EXT(c1_2,16,0), EXT(s1_3,0,16), EXT(c1_3,0,16))// 102
    val add1_2 = s2_0(15,0)
    val add2_2 = c2_0(15,0)
    // fourth layer
    val (s3_0, c3_0) = CP42(EXT(s2_0(85,16),32,0), EXT(c2_0(85,16),32,0), s2_1, c2_1)//.map(_(101,0))
    val add1_3 = s3_0(31,0)
    val add2_3 = c3_0(31,0)
    // firth layer
    val (s4_0, c4_0) = CP42(s3_0(101,32), c3_0(101,32), EXT(booth(NBooth-2),1,0), EXT(last_booth,0,2))
    val add1_4 = s4_0(69,0)
    val add2_4 = c4_0(69,0)
    
    // third stage
    val add1 = RegEnable(Cat(add1_4, add1_3, add1_2, add1_1, add1_0), s2Fire)
    val add2 = RegEnable(Cat(add2_4, add2_3, add2_2, add2_1, add2_0), s2Fire)
    when(s2Fire) { subS3 := subS2 } // assign subS3 here
    val res = add1 + add2 + subS3
    
    // fourth stage
    val resS4 = RegEnable(res(len-1, 0), s3Fire)
    val resSub = ~resS4 + 1.U
    
    io.in.ready := s1_ready
    io.out.bits := Mux(s3_valid && !subS3, res(2*len-1,0), ZeroExt(resSub, 2*len))
    io.out.valid := Mux(s3_valid && !subS3, s3_valid, s4_valid)
}

class MulAdd33(len: Int = 33) extends NutCoreModule {
    val io = IO(new MulAddIO(len)) // TODO: len only supports 65
    /* assume signed-unsigned is hangled by MDU*/
    // val X = io.in.bits.src1
    // val Y = io.in.bits.src2
    // val Z = io.in.bits.src3
    val valid = io.in.valid
    val X = io.in.bits(0)
    val Y = io.in.bits(1)
    val Z = io.in.bits(2)
    // X * Y + Z. X,Y,Z is both 65-bits, pre-processed by out-module
    // -(X * Y) + Z
    
    // 1st stage: generate partial
    // 2nd stage: compress
    // 3rd stage: add
    // 4th stage: neg
    val s1Fire = Wire(Bool())
    val s2Fire = Wire(Bool())
    val s3Fire = Wire(Bool())
    val s4Fire = Wire(Bool())
    val s1_valid = io.in.valid
    val s2_valid = RegEnable(false.B, init = false.B, s2Fire)
    val s3_valid = RegEnable(false.B, init = false.B, s3Fire)
    val s4_valid = RegEnable(false.B, init = false.B, s4Fire)
    val subS3 = Reg(Bool()) // for mul-sub, get into s4Stage
    when (s1Fire) { s2_valid := true.B }
    when (s2Fire) { s3_valid := true.B }
    when (s3Fire && subS3) { s4_valid := true.B }
    val s1_ready = !s1_valid || s1Fire
    val s2_ready = !s2_valid || s2Fire
    val s3_ready = !s3_valid || s3Fire
    val s4_ready = !s4_valid || s4Fire
    s1Fire := s1_valid && s2_ready
    s2Fire := s2_valid && s3_ready
    s3Fire := Mux(s3_valid && !subS3, s3_valid && io.out.ready, s3_valid && s4_ready)
    s4Fire := s4_valid && io.out.ready
    
    // First Stage
    val lenOdd = len % 2 == 1
    assert(lenOdd)
    val Len = if(lenOdd) len+1 else len //
    val NBooth:Int = Len / 2 //
    
    val y_extend = if(lenOdd) Cat(Y(len-1), Y, 0.U(1.W)) else Cat(Y, 0.U(1.W))
    val y_part = Wire(Vec(NBooth, UInt(3.W))) // radix-4 booth
    (0 until NBooth).map(i => y_part(i) := y_extend(i*2+2, i*2))
    val booth_x = Wire(Vec(NBooth, UInt(Len.W)))
    (0 until NBooth).map(i => booth_x(i) := LookupTree(y_part(i), List(
        "b000".U  -> 0.U(Len.W),
        "b001".U  -> SignExt(X, Len),
        "b010".U  -> SignExt(X, Len),
        "b011".U  -> SignExt(X << 1.U, Len),
        "b100".U  -> SignExt(~(X << 1.U), Len),
        "b101".U  -> ~SignExt(X, Len),
        "b110".U  -> ~SignExt(X, Len),
        "b111".U  -> 0.U(Len.W)
    ))) // FIXME: handle the case that len is not odd, 2*X is hard to handle.
    
    val isSub = VecInit((0 until NBooth).map(i => y_part(i)(2).asBool && y_part(i) =/= "b111".U)).asUInt
    val isNeg = VecInit((0 until NBooth).map(i => booth_x(i)(Len-1))).asUInt
    
    val boothWire = Wire(Vec(NBooth, UInt((Len+3).W)))
    (1 until NBooth).map(i => boothWire(i) := Cat(1.U(1.W), ~isNeg(i), booth_x(i)(Len-2, 0), 0.U(1.W), isSub(i-1)))
    boothWire(0) := Cat(0.U(2.W), 1.U(1.W), ~isNeg(0), booth_x(0)(Len-2,0))
    
    // second stage
    val booth = RegEnable(boothWire, s1Fire)
    val Z_reg = RegEnable(Z, s1Fire)
    val subS2 = RegEnable(io.sub, s1Fire)
    val Z_add = Mux(subS2, ~Z_reg, Z_reg)
    
    val last_booth = booth(NBooth-1)(Len+1, 0)
    
    // 17+1
    // => 4 + 4 + 4 + 4 + 2
    // => 4 + 4 + 2
    // => 4 + 2 (or 3 + 3)
    // => 4
    // => 2
    val boothZ = Cat(0.U(1.W), ~Z_add(32), Z_add(32,0)) // 2+len=35
    val (s0_0, c0_0) = CP42(EXT(boothZ,4,0),    EXT(booth(0),2,0),  EXT(booth(1),2,0),  EXT(booth(2),0,2)) //40
    val (s0_1, c0_1) = CP42(EXT(booth(3),6,0),  EXT(booth(4),4,2),  EXT(booth(5),2,4),  EXT(booth(6),0,6)) //44
    val (s0_2, c0_2) = CP42(EXT(booth(7),6,0),  EXT(booth(8),4,2),  EXT(booth(9),2,4),  EXT(booth(10),0,6))
    val (s0_3, c0_3) = CP42(EXT(booth(11),6,0), EXT(booth(12),4,2), EXT(booth(13),2,4), EXT(booth(14),0,6)) //44
    val add1_0 = s0_0(3,0)
    val add2_0 = c0_0(3,0)
    // seconde layer
    val (s1_0, c1_0) = CP42(EXT(s0_0(39,4),8,0), EXT(c0_0(39,4),8,0), s0_1, c0_1) // 45
    val (s1_1, c1_1) = CP42(EXT(s0_2,8,0), EXT(c0_2,8,0), EXT(s0_3,0,8), EXT(c0_3,0,8)) //53
    val add1_1 = s1_0(7,0)
    val add2_1 = c1_0(7,0)
    // third layer
    val (s2_0, c2_0) = CP42(EXT(s1_0(44,8),16,0), EXT(c1_0(44,8),16,0), s1_1, c1_1) // 54
    val add1_2 = s2_0(15,0)
    val add2_2 = c2_0(15,0)
    // fouth layer
    // val (s3_0, c3_0) = cp42(s2_0(53,16), c2_0(53,16), s0_4, Cat(c0_4(c0_4.getWidth-2,0),0.U(1.W))) //39
    val (s3_0, c3_0) = CP42(s2_0(53,16), c2_0(53,16), EXT(booth(NBooth-2),1,0), EXT(last_booth,0,2))
    val add1_3 = s3_0(37,0)
    val add2_3 = c3_0(37,0)
    
    // Third stage
    // 4 + 8 + 16 + 38 => 66
    val add1 = RegEnable(Cat(add1_3, add1_2, add1_1, add1_0), s2Fire)
    val add2 = RegEnable(Cat(add2_3, add2_2, add2_1, add2_0), s2Fire)
    when(s2Fire) { subS3 := subS2 } // assign subS3 here
    val res = add1 + add2 + subS3
    
    // fourth stage
    val resS4 = RegEnable(res(len-1, 0), s3Fire)
    val resSub = ~resS4 + 1.U
    
    io.in.ready := s1_ready
    io.out.bits := Mux(s3_valid && !subS3, res(2*len-1,0), ZeroExt(resSub, 2*len))
    io.out.valid := Mux(s3_valid && !subS3, s3_valid, s4_valid)
}

class MulAdd17(len: Int = 17) extends NutCoreModule {
    val io = IO(new MulAddIO(len)) // TODO: len only supports 17
    /* assume signed-unsigned is hangled by MDU*/
    // val X = io.in.bits.src1
    // val Y = io.in.bits.src2
    // val Z = io.in.bits.src3
    val valid = io.in.valid
    val X = io.in.bits(0)
    val Y = io.in.bits(1)
    val Z = io.in.bits(2)
    // X * Y + Z. X,Y,Z is both 65-bits, pre-processed by out-module
    // -(X * Y) + Z
    
    // 1st stage: generate partial
    // 2nd stage: compress
    // 3rd stage: add
    // 4th stage: neg
    val s1Fire = Wire(Bool())
    val s2Fire = Wire(Bool())
    val s3Fire = Wire(Bool())
    val s4Fire = Wire(Bool())
    val s1_valid = io.in.valid
    val s2_valid = RegEnable(false.B, init = false.B, s2Fire)
    val s3_valid = RegEnable(false.B, init = false.B, s3Fire)
    val s4_valid = RegEnable(false.B, init = false.B, s4Fire)
    val subS3 = Reg(Bool()) // for mul-sub, get into s4Stage
    when (s1Fire) { s2_valid := true.B }
    when (s2Fire) { s3_valid := true.B }
    when (s3Fire && subS3) { s4_valid := true.B }
    val s1_ready = !s1_valid || s1Fire
    val s2_ready = !s2_valid || s2Fire
    val s3_ready = !s3_valid || s3Fire
    val s4_ready = !s4_valid || s4Fire
    s1Fire := s1_valid && s2_ready
    s2Fire := s2_valid && s3_ready
    s3Fire := Mux(s3_valid && !subS3, s3_valid && io.out.ready, s3_valid && s4_ready)
    s4Fire := s4_valid && io.out.ready
    
    // First Stage
    val lenOdd = len % 2 == 1
    assert(lenOdd)
    val Len = if(lenOdd) len+1 else len //
    val NBooth:Int = Len / 2 //
    
    val y_extend = if(lenOdd) Cat(Y(len-1), Y, 0.U(1.W)) else Cat(Y, 0.U(1.W))
    val y_part = Wire(Vec(NBooth, UInt(3.W))) // radix-4 booth
    (0 until NBooth).map(i => y_part(i) := y_extend(i*2+2, i*2))
    val booth_x = Wire(Vec(NBooth, UInt(Len.W)))
    (0 until NBooth).map(i => booth_x(i) := LookupTree(y_part(i), List(
        "b000".U  -> 0.U(Len.W),
        "b001".U  -> SignExt(X, Len),
        "b010".U  -> SignExt(X, Len),
        "b011".U  -> SignExt(X << 1.U, Len),
        "b100".U  -> SignExt(~(X << 1.U), Len),
        "b101".U  -> ~SignExt(X, Len),
        "b110".U  -> ~SignExt(X, Len),
        "b111".U  -> 0.U(Len.W)
    ))) // FIXME: handle the case that len is not odd, 2*X is hard to handle.
    
    val isSub = VecInit((0 until NBooth).map(i => y_part(i)(2).asBool && y_part(i) =/= "b111".U)).asUInt
    val isNeg = VecInit((0 until NBooth).map(i => booth_x(i)(Len-1))).asUInt
    
    val boothWire = Wire(Vec(NBooth, UInt((Len+3).W)))
    (1 until NBooth).map(i => boothWire(i) := Cat(1.U(1.W), ~isNeg(i), booth_x(i)(Len-2, 0), 0.U(1.W), isSub(i-1)))
    boothWire(0) := Cat(0.U(2.W), 1.U(1.W), ~isNeg(0), booth_x(0)(Len-2,0))
    
    // Second Stage
    val booth = RegEnable(boothWire, s1Fire)
    val Z_reg = RegEnable(Z, s1Fire)
    val subS2 = RegEnable(io.sub, s1Fire)
    val Z_add = Mux(subS2, ~Z_reg, Z_reg)
    
    val last_booth = booth(NBooth-1)(Len+1, 0)
    
    // 9 + 1 => 4 + 4 + 2 => 4 + 2 => 4 => 2
    // first layer, use 42-compressor
    val boothZ = Cat(0.U(1.W), ~Z_add(16), Z_add(16,0)) // 2+17=19
    val (s0_0, c0_0) = CP42(EXT(boothZ,4,0), EXT(booth(0),2,0), EXT(booth(1),2,0), EXT(booth(2),0,2)) //24
    val (s0_1, c0_1) = CP42(EXT(booth(3),6,0), EXT(booth(4),4,2), EXT(booth(5),2,4), EXT(booth(6),0,6)) //28
    val add1_0 = s0_0(3,0)
    val add2_0 = c0_0(3,0)
    // second layer
    val (s1_0, c1_0) = CP42(EXT(s0_0(23,4),8,0), EXT(c0_0(23,4),8,0), s0_1, c0_1) // 29
    val add1_1 = s1_0(7,0)
    val add2_1 = c1_0(7,0)
    // third layer
    val (s2_0, c2_0) = CP42(EXT(s1_0(28,8),1,0), EXT(c1_0(28,8),1,0), EXT(booth(NBooth-2),1,0), EXT(last_booth,0,2))
    val add1_2 = s2_0(21,0)
    val add2_2 = c2_0(21,0)
    
    // Third Stage
    // 4 + 8 + 22 => 34
    val add1 = RegEnable(Cat(add1_2, add1_1, add1_0), s2Fire)
    val add2 = RegEnable(Cat(add2_2, add2_1, add2_0), s2Fire)
    when(s2Fire) { subS3 := subS2 } // assign subS3 here
    val res = add1 + add2 + subS3
    
    // fourth stage
    val resS4 = RegEnable(res(len-1, 0), s3Fire)
    val resSub = ~resS4 + 1.U
    
    io.in.ready := s1_ready
    io.out.bits := Mux(s3_valid && !subS3, res(2*len-1,0), ZeroExt(resSub, 2*len))
    io.out.valid := Mux(s3_valid && !subS3, s3_valid, s4_valid)
}

class MulAdd9(len: Int = 9) extends NutCoreModule {
    val io = IO(new MulAddIO(len)) // TODO: len only supports 65
    /* assume signed-unsigned is hangled by MDU*/
    // val X = io.in.bits.src1
    // val Y = io.in.bits.src2
    // val Z = io.in.bits.src3
    val valid = io.in.valid
    val X = io.in.bits(0)
    val Y = io.in.bits(1)
    val Z = io.in.bits(2)
    // X * Y + Z. X,Y,Z is both 65-bits, pre-processed by out-module
    // -(X * Y) + Z
    
    // 1st stage: generate partial
    // 2nd stage: compress
    // 3rd stage: add
    // 4th stage: neg
    val s1Fire = Wire(Bool())
    val s2Fire = Wire(Bool())
    val s3Fire = Wire(Bool())
    val s4Fire = Wire(Bool())
    val s1_valid = io.in.valid
    val s2_valid = RegEnable(false.B, init = false.B, s2Fire)
    val s3_valid = RegEnable(false.B, init = false.B, s3Fire)
    val s4_valid = RegEnable(false.B, init = false.B, s4Fire)
    val subS3 = Reg(Bool()) // for mul-sub, get into s4Stage
    when (s1Fire) { s2_valid := true.B }
    when (s2Fire) { s3_valid := true.B }
    when (s3Fire && subS3) { s4_valid := true.B }
    val s1_ready = !s1_valid || s1Fire
    val s2_ready = !s2_valid || s2Fire
    val s3_ready = !s3_valid || s3Fire
    val s4_ready = !s4_valid || s4Fire
    s1Fire := s1_valid && s2_ready
    s2Fire := s2_valid && s3_ready
    s3Fire := Mux(s3_valid && !subS3, s3_valid && io.out.ready, s3_valid && s4_ready)
    s4Fire := s4_valid && io.out.ready
    
    // First Stage
    val lenOdd = len % 2 == 1
    assert(lenOdd)
    val Len = if(lenOdd) len+1 else len //
    val NBooth = Len / 2 //
    
    val y_extend = if(lenOdd) Cat(Y(len-1), Y, 0.U(1.W)) else Cat(Y, 0.U(1.W))
    val y_part = Wire(Vec(NBooth, UInt(3.W))) // radix-4 booth
    (0 until NBooth).map(i => y_part(i) := y_extend(i*2+2, i*2))
    val booth_x = Wire(Vec(NBooth, UInt(Len.W)))
    (0 until NBooth).map(i => booth_x(i) := LookupTree(y_part(i), List(
        "b000".U  -> 0.U(Len.W),
        "b001".U  -> SignExt(X, Len),
        "b010".U  -> SignExt(X, Len),
        "b011".U  -> SignExt(X << 1.U, Len),
        "b100".U  -> SignExt(~(X << 1.U), Len),
        "b101".U  -> ~SignExt(X, Len),
        "b110".U  -> ~SignExt(X, Len),
        "b111".U  -> 0.U(Len.W)
    ))) // FIXME: handle the case that len is not odd, 2*X is hard to handle.
    
    val isSub = VecInit((0 until NBooth).map(i => y_part(i)(2).asBool && y_part(i) =/= "b111".U)).asUInt
    val isNeg = VecInit((0 until NBooth).map(i => booth_x(i)(Len-1))).asUInt
    
    val boothWire = Wire(Vec(NBooth, UInt((Len+3).W)))
    (1 until NBooth).map(i => boothWire(i) := Cat(1.U(1.W), ~isNeg(i), booth_x(i)(Len-2, 0), 0.U(1.W), isSub(i-1)))
    boothWire(0) := Cat(0.U(2.W), 1.U(1.W), ~isNeg(0), booth_x(0)(Len-2,0))
    
    // Second Stage
    val booth = RegEnable(boothWire, s1Fire)
    val Z_reg = RegEnable(Z, s1Fire)
    val subS2 = RegEnable(io.sub, s1Fire)
    val Z_add = Mux(subS2, ~Z_reg, Z_reg)
    
    val last_booth = booth(NBooth-1)(Len+1, 0)
    
    // 5+1 => 4 + 2 => 4 => 2
    // first layer, use 42-compressor
    val boothZ = Cat(0.U(1.W), ~Z_add(8), Z_add(8,0)) // 2+9 = 11
    val (s0_0, c0_0) = CP42(EXT(boothZ,4,0), EXT(booth(0),2,0), EXT(booth(1),2,0), EXT(booth(2),0,2)) //16
    val add1_0 = s0_0(3,0)
    val add2_0 = c0_0(3,0)
    // second layer
    val (s1_0, c1_0) = CP42(EXT(s0_0(15,4),2,0), EXT(c0_0(15,4),2,0), EXT(booth(NBooth-2),1,0), EXT(last_booth,0,2))
    val add1_1 = s1_0(13,0)
    val add2_1 = c1_0(13,0)
    
    // Third Stage
    val add1 = RegEnable(Cat(add1_1, add1_0), s2Fire)
    val add2 = RegEnable(Cat(add2_1, add2_0), s2Fire)
    when(s2Fire) { subS3 := subS2 } // assign subS3 here
    val res = add1 + add2 + subS3
    
    // fourth stage
    val resS4 = RegEnable(res(len-1, 0), s3Fire)
    val resSub = ~resS4 + 1.U
    
    // io
    io.in.ready := s1_ready
    io.out.bits := Mux(s3_valid && !subS3, res(2*len-1,0), ZeroExt(resSub, 2*len))
    io.out.valid := Mux(s3_valid && !subS3, s3_valid, s4_valid)
}

class MulAddTop(len: Int = 65) extends NutCoreModule {
    val io = IO(new MulAddIO(len))
    
    val mul = if(len==65)
        Module(new MulAdd65)
    else if(len==33)
        Module(new MulAdd33)
    else if(len==17)
        Module(new MulAdd17)
    else if(len==9)
        Module(new MulAdd9)
    else {
        assert(false.B)
        Module(new MulAdd65)
    }
    mul.io <> io
    // mul.io.in <> io.in
    // mul.io.out <> io.out
}

class DividerSP(len: Int = 64) extends NutCoreModule {
    val io = IO(new MulDivIO(len))
    
    def abs(a: UInt, sign: Bool): (Bool, UInt) = {
        val s = a(len - 1) && sign
        (s, Mux(s, -a, a))
    }
    
    val s_idle :: s_log2 :: s_shift :: s_compute :: s_finish :: Nil = Enum(5)
    val state = RegInit(s_idle)
    val newReq = (state === s_idle) && io.in.fire()
    
    val (a, b) = (io.in.bits(0), io.in.bits(1))
    val divBy0 = b === 0.U(len.W)
    
    val shiftReg = Reg(UInt((1 + len * 2).W))
    val hi = shiftReg(len * 2, len)
    val lo = shiftReg(len - 1, 0)
    
    val (aSign, aVal) = abs(a, io.sign)
    val (bSign, bVal) = abs(b, io.sign)
    val aSignReg = RegEnable(aSign, newReq)
    val qSignReg = RegEnable((aSign ^ bSign) && !divBy0, newReq)
    val bReg = RegEnable(bVal, newReq)
    val aValx2Reg = RegEnable(Cat(aVal, "b0".U), newReq)
    
    val cnt = Counter(len)
    when (newReq) {
        state := s_log2
    } .elsewhen (state === s_log2) {
        // `canSkipShift` is calculated as following:
        //   bEffectiveBit = Log2(bVal, XLEN) + 1.U
        //   aLeadingZero = 64.U - aEffectiveBit = 64.U - (Log2(aVal, XLEN) + 1.U)
        //   canSkipShift = aLeadingZero + bEffectiveBit
        //     = 64.U - (Log2(aVal, XLEN) + 1.U) + Log2(bVal, XLEN) + 1.U
        //     = 64.U + Log2(bVal, XLEN) - Log2(aVal, XLEN)
        //     = (64.U | Log2(bVal, XLEN)) - Log2(aVal, XLEN)  // since Log2(bVal, XLEN) < 64.U
        val canSkipShift = 0.U//(64.U | Log2(bReg)) - Log2(aValx2Reg)
        // When divide by 0, the quotient should be all 1's.
        // Therefore we can not shift in 0s here.
        // We do not skip any shift to avoid this.
        cnt.value := Mux(divBy0, 0.U, Mux(canSkipShift >= (len-1).U, (len-1).U, canSkipShift))
        state := s_shift
    } .elsewhen (state === s_shift) {
        shiftReg := aValx2Reg << cnt.value
        state := s_compute
    } .elsewhen (state === s_compute) {
        val enough = hi.asUInt >= bReg.asUInt
        shiftReg := Cat(Mux(enough, hi - bReg, hi)(len - 1, 0), lo, enough)
        cnt.inc()
        when (cnt.value === (len-1).U) { state := s_finish }
    } .elsewhen (state === s_finish) {
        state := s_idle
    }
    
    val r = hi(len, 1)
    val resQ = Mux(qSignReg, -lo, lo)
    val resR = Mux(aSignReg, -r, r)
    io.out.bits := Cat(resR, resQ)
    
    io.out.valid := (if (HasDiv) (state === s_finish) else io.in.valid) // FIXME: should deal with ready = 0
    io.in.ready := (state === s_idle)
}

class VMDUIO(val len: Int) extends NutCoreBundle {
    val in = Flipped(Decoupled(new Bundle {
        val src1 = Output(UInt(len.W))
        val src2 = Output(UInt(len.W))
        val src3 = Output(UInt(len.W))
        val func = Output(FuOpType())
        val vsew = Output(UInt(2.W))
    }))
    val out = Decoupled(Output(UInt(len.W)))
}

class VMDU(len: Int, MulAddEnable: Boolean = true) extends NutCoreModule { // copy from MDU, adhoc
    val io = IO(new VMDUIO(len))
    // val MulAddEnable = true
    
    val (valid, src1, src2, src3, func, vsew) = (io.in.valid, io.in.bits.src1, io.in.bits.src2, io.in.bits.src3, io.in.bits.func, io.in.bits.vsew)
    
    def access(valid: Bool, src1: UInt, src2: UInt, src3: UInt, func: UInt, vsew: UInt) = {
        this.valid := valid
        this.src1 := src1
        this.src2 := src2
        this.src3 := src3
        this.func := func
        this.vsew := vsew
        io.out.bits
    }
    
    val vsew_reg = RegEnable(vsew, io.in.fire())
    val func_reg = RegEnable(func, io.in.fire())
    
    val isDiv = VXUOpType.isDiv(func)
    val isDivSign = VXUOpType.isDivSign(func)
    
    val mul = Module(new MulAddTop(len + 1)) // Enable mulAdd here 1
    // val mul = Module(new Multiplier(len + 1))
    
    // val div = Module(new Divider(len))
    val div = Module(new DividerSP(XLEN))
    List(div.io).map { case x =>
        // List(mul.io, div.io).map { case x =>
        x.sign := isDivSign
        x.out.ready := io.out.ready
    }
    mul.io.out.ready := io.out.ready
    
    mul.io.sub := VXUOpType.isMulAddSub(func) // Enable mulAdd here 1
    // mul.io.sign := isDivSign
    
    val signext = SignExt(_: UInt, len + 1)
    val zeroext = ZeroExt(_: UInt, len + 1)
    // val signext = SignExt(_: UInt, XLEN+1)
    // val zeroext = ZeroExt(_: UInt, XLEN+1)
    val mulInputFuncTable = List(
        VXUOpType.mul -> (signext, signext),
        VXUOpType.mulh -> (signext, signext),
        VXUOpType.mulhsu -> (zeroext, signext), // different order from MDU, for that Mul-Add has different order with Multiplier
        VXUOpType.mulhu -> (zeroext, zeroext)
    )
    // SRC1-VS2 : SRC2:VS1 : SRC3:VS3
    val m_src1 = src2
    val m_src2 = Mux(VXUOpType.isMulAddReverse(func), src3, src1)
    val m_src3 = Mux(VXUOpType.isMulAddReverse(func), src1, src3)
    mul.io.in.bits(0) := Mux(VXUOpType.isMulAdd(func), signext(m_src1), LookupTree(func(6, 5), mulInputFuncTable.map(p => (p._1(4, 3), p._2._1(m_src1)))))
    mul.io.in.bits(1) := Mux(VXUOpType.isMulAdd(func), signext(m_src2), LookupTree(func(6, 5), mulInputFuncTable.map(p => (p._1(4, 3), p._2._2(m_src2)))))
    
    mul.io.in.bits(2) := Mux(VXUOpType.isMulAdd(func), signext(m_src3), 0.U) // Enable mulAdd here 1
    
    // val divInputFunc = (x: UInt) => x
    div.io.in.bits(0) := Mux(isDivSign, SignExt(src1, XLEN), ZeroExt(src1, XLEN)) //src1// divInputFunc(src1)
    div.io.in.bits(1) := Mux(isDivSign, SignExt(src2, XLEN), ZeroExt(src2, XLEN)) // src2// divInputFunc(src2)
    
    mul.io.in.valid := io.in.valid && !isDiv
    div.io.in.valid := io.in.valid && isDiv
    
    val mulHighres = WireInit(0.U(len.W))
    if (len == 8) {
        mulHighres := mul.io.out.bits(2 * len - 1, len)
    } else if (len == 16) {
        mulHighres := Mux(vsew(0), mul.io.out.bits(2 * len - 1, len), ZeroExt(mul.io.out.bits(2 * 8 - 1, 8), len))
    } else if (len == 32) {
        mulHighres := LookupTree(vsew(1, 0), List(
            "b00".U -> ZeroExt(mul.io.out.bits(2 * 8 - 1, 8), len),
            "b01".U -> ZeroExt(mul.io.out.bits(2 * 16 - 1, 16), len),
            "b10".U -> mul.io.out.bits(2 * len - 1, len),
            "b11".U -> 0.U
        ))
    } else if (len == 64) {
        mulHighres := LookupTree(vsew(1, 0), List(
            "b00".U -> ZeroExt(mul.io.out.bits(2 * 8 - 1, 8), len),
            "b01".U -> ZeroExt(mul.io.out.bits(2 * 16 - 1, 16), len),
            "b10".U -> ZeroExt(mul.io.out.bits(2 * 32 - 1, 32), len),
            "b11".U -> mul.io.out.bits(2 * len - 1, len)
        ))
    }
    val mulRes = Mux(VXUOpType.isMulL(func_reg), mul.io.out.bits(len - 1, 0), mulHighres)
    val divRes = Mux(~VXUOpType.isDivL(func_reg) /* func(1) rem */ , div.io.out.bits(2 * XLEN - 1, XLEN)(len - 1, 0), div.io.out.bits(XLEN - 1, 0)(len - 1, 0))
    val res = Mux(isDiv, divRes, mulRes)
    io.out.bits := res
    
    val isDivReg = Mux(io.in.fire(), isDiv, RegNext(isDiv))
    io.in.ready := Mux(isDiv, div.io.in.ready, mul.io.in.ready)
    io.out.valid := Mux(isDivReg, div.io.out.valid, mul.io.out.valid)
    
    // BoringUtils.addSource(mul.io.out.fire(), "perfCntCondMmulInstr")
}
