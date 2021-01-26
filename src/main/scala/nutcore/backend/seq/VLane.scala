package nutcore

import chisel3._

trait VLaneParameter extends HasNutCoreParameter {
    val NLane = VLEN / XLEN
}

class VLane {
    
}
