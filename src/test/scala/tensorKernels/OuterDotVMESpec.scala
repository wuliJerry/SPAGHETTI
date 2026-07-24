package tensorKernels

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import config._
import interfaces.CooDataBundle
import node.vecN
import shell._
import dnn.memory.CoreConfig

/** Behavioral AXI read-only memory: answers ar bursts from a beat ROM, with latency. */
class AXIReadMem(beats: Seq[BigInt], latency: Int)(implicit p: Parameters) extends Module {
  val mp = p(ShellKey).memParams
  val io = IO(new Bundle { val mem = Flipped(new AXIMaster(mp)) })
  val sizeConst = log2Ceil(mp.dataBits / 8) // bytes-per-beat shift
  val rom = VecInit((beats ++ Seq.fill(4)(BigInt(0))).map(_.U(mp.dataBits.W)))

  // write channels unused
  io.mem.aw.ready := false.B
  io.mem.w.ready := false.B
  io.mem.b.valid := false.B
  io.mem.b.bits.resp := 0.U
  io.mem.b.bits.id := 0.U
  io.mem.b.bits.user := 0.U

  val sIdle :: sWait :: sData :: Nil = Enum(3)
  val state = RegInit(sIdle)
  val beatIdx = Reg(UInt(16.W))
  val len = Reg(UInt(mp.lenBits.W))
  val timer = RegInit(0.U(16.W))

  io.mem.ar.ready := state === sIdle
  io.mem.r.valid := state === sData
  io.mem.r.bits.data := rom(beatIdx)
  io.mem.r.bits.last := (state === sData) && (len === 0.U)
  io.mem.r.bits.resp := 0.U
  io.mem.r.bits.id := 0.U
  io.mem.r.bits.user := 0.U

  switch(state) {
    is(sIdle) {
      when(io.mem.ar.fire()) {
        beatIdx := io.mem.ar.bits.addr >> sizeConst.U
        len := io.mem.ar.bits.len
        timer := latency.U
        state := Mux(latency.U === 0.U, sData, sWait)
      }
    }
    is(sWait) { timer := timer - 1.U; when(timer === 1.U) { state := sData } }
    is(sData) {
      when(io.mem.r.fire()) {
        beatIdx := beatIdx + 1.U
        when(len === 0.U) { state := sIdle }.otherwise { len := len - 1.U }
      }
    }
  }
}

/** OuterDot wired to the REAL VME (shared RRArbiter) backed by AXIReadMem.
  * 6 streams live at beats 0..5 (byte addrs 0,64,128,192,256,320). */
class OuterDotVMEHarness(indA: Seq[Int], valA: Seq[Int], ptrA: Seq[Int],
                         indB: Seq[Int], valB: Seq[Int], ptrB: Seq[Int], latency: Int)
                        (implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val nnz_A = Input(UInt(32.W)); val nnz_B = Input(UInt(32.W)); val segSize = Input(UInt(32.W))
    val out = Decoupled(new CooDataBundle(UInt(p(XLEN).W)))
    val eop = Output(Bool())
  })
  def pack(words: Seq[Int]): BigInt =
    words.zipWithIndex.map { case (w, i) => (BigInt(w) & BigInt("FFFFFFFF", 16)) << (32 * i) }
      .foldLeft(BigInt(0))(_ | _)

  val dut = Module(new OuterDot(memTensorType = "inp", maxRowLen = 16)(new vecN(1, 0, false)))
  val vme = Module(new VME())
  // beats: 0 ptrA, 1 indA, 2 valA, 3 ptrB, 4 indB, 5 valB
  val mem = Module(new AXIReadMem(Seq(pack(ptrA), pack(indA), pack(valA),
                                      pack(ptrB), pack(indB), pack(valB)), latency))
  mem.io.mem.ar <> vme.io.mem.ar
  vme.io.mem.r <> mem.io.mem.r
  mem.io.mem.aw <> vme.io.mem.aw
  mem.io.mem.w <> vme.io.mem.w
  vme.io.mem.b <> mem.io.mem.b

  // read clients 0..5
  vme.io.vme.rd(0) <> dut.io.vme_rd_ptr(0)
  vme.io.vme.rd(1) <> dut.io.vme_rd_ind(0)
  vme.io.vme.rd(2) <> dut.io.vme_rd_val(0)
  vme.io.vme.rd(3) <> dut.io.vme_rd_ptr(1)
  vme.io.vme.rd(4) <> dut.io.vme_rd_ind(1)
  vme.io.vme.rd(5) <> dut.io.vme_rd_val(1)
  // write clients unused
  for (i <- 0 until vme.io.vme.wr.length) {
    vme.io.vme.wr(i).cmd.valid := false.B
    vme.io.vme.wr(i).cmd.bits.addr := 0.U
    vme.io.vme.wr(i).cmd.bits.len := 0.U
    vme.io.vme.wr(i).data.valid := false.B
    vme.io.vme.wr(i).data.bits := 0.U
  }

  dut.io.start := io.start; dut.io.nnz_A := io.nnz_A; dut.io.nnz_B := io.nnz_B; dut.io.segSize := io.segSize
  dut.io.ptr_A_BaseAddr := (0 * 64).U
  dut.io.ind_A_BaseAddr := (1 * 64).U
  dut.io.val_A_BaseAddr := (2 * 64).U
  dut.io.ptr_B_BaseAddr := (3 * 64).U
  dut.io.ind_B_BaseAddr := (4 * 64).U
  dut.io.val_B_BaseAddr := (5 * 64).U
  io.out <> dut.io.out; io.eop := dut.io.eop
}

class OuterDotVMETester(c: OuterDotVMEHarness, segSize: Int, nnzA: Int, nnzB: Int, bpPeriod: Int, label: String)
  extends PeekPokeTester(c) {
  poke(c.io.segSize, segSize); poke(c.io.nnz_A, nnzA); poke(c.io.nnz_B, nnzB)
  poke(c.io.start, 0); poke(c.io.out.ready, 1)
  step(2); poke(c.io.start, 1); step(1); poke(c.io.start, 0)
  var eopSeen = false; var eopCycle = -1
  val outs = scala.collection.mutable.ArrayBuffer[(BigInt, BigInt, BigInt)]()
  for (t <- 0 until 800) {
    val rdy = if (bpPeriod <= 1) 1 else if (t % bpPeriod == 0) 1 else 0
    poke(c.io.out.ready, rdy)
    if (peek(c.io.out.valid) == 1 && rdy == 1)
      outs += ((peek(c.io.out.bits.row), peek(c.io.out.bits.col), peek(c.io.out.bits.data)))
    if (peek(c.io.eop) == 1 && !eopSeen) { eopSeen = true; eopCycle = t }
    step(1)
  }
  println(s"[VME $label] ${if (!eopSeen) "*** HANG ***" else "done "} eop=$eopSeen cyc=$eopCycle out=${outs.toList}")
}

/** Runs TWO launches back-to-back WITHOUT a chip reset (as the accelerator is
  * driven across a batch of vectors / segments). DRAM holds 3-column data. */
class OuterDotReentrantTester(c: OuterDotVMEHarness, run1: (Int,Int,Int), run2: (Int,Int,Int))
  extends PeekPokeTester(c) {
  poke(c.io.out.ready, 1)
  def launch(seg: Int, nA: Int, nB: Int, tag: String): Unit = {
    poke(c.io.segSize, seg); poke(c.io.nnz_A, nA); poke(c.io.nnz_B, nB)
    poke(c.io.start, 0); step(2); poke(c.io.start, 1); step(1); poke(c.io.start, 0)
    var eopSeen = false; var eopCycle = -1
    val outs = scala.collection.mutable.ArrayBuffer[(BigInt, BigInt, BigInt)]()
    var t = 0
    while (t < 300 && !eopSeen) {
      if (peek(c.io.out.valid) == 1) outs += ((peek(c.io.out.bits.row), peek(c.io.out.bits.col), peek(c.io.out.bits.data)))
      if (peek(c.io.eop) == 1) { eopSeen = true; eopCycle = t }
      step(1); t += 1
    }
    println(s"[REENTRANT $tag seg=$seg] ${if (!eopSeen) "*** HANG ***" else "done "} eop=$eopSeen cyc=$eopCycle out=${outs.toList}")
  }
  launch(run1._1, run1._2, run1._3, "run1")
  launch(run2._1, run2._2, run2._3, "run2")
}

class OuterDotReentrantSpec extends ChiselFlatSpec {
  implicit val p: Parameters = new De10Config(1, 1) ++ new CoreConfig ++ new MiniConfig
  // 3-column DRAM: A cols (row0) vals 2,3,4 ; B rows cols 0,1,2 vals 10,20,30
  def mk() = new OuterDotVMEHarness(
    indA = Seq(0,0,0), valA = Seq(2,3,4), ptrA = Seq(0,1,2,3),
    indB = Seq(0,1,2), valB = Seq(10,20,30), ptrB = Seq(0,1,2,3), latency = 0)

  // second-launch symptom depends on k=0 emptiness of the (re-run) data:
  def mkK0nonempty() = new OuterDotVMEHarness(
    indA = Seq(0,0), valA = Seq(2,3), ptrA = Seq(0,1,2),
    indB = Seq(0,1), valB = Seq(10,20), ptrB = Seq(0,1,2), latency = 0)
  def mkK0empty() = new OuterDotVMEHarness(
    indA = Seq(0), valA = Seq(3), ptrA = Seq(0,0,1),
    indB = Seq(1), valB = Seq(20), ptrB = Seq(0,0,1), latency = 0)

  behavior of "OuterDot reentrancy (2 launches, no reset)"
  it should "SECOND-LAUNCH k0-NONEMPTY seg2 (expect HANG)" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/outerdot_re"),
      () => mkK0nonempty()) { c => new OuterDotReentrantTester(c, (2,2,2), (2,2,2)) }
  }
  it should "SECOND-LAUNCH k0-EMPTY seg2 (expect WRONG DATA, not hang)" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/outerdot_re"),
      () => mkK0empty()) { c => new OuterDotReentrantTester(c, (2,1,1), (2,1,1)) }
  }
  it should "seg3 then seg2 (larger then smaller)" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/outerdot_re"),
      () => mk()) { c => new OuterDotReentrantTester(c, (3,3,3), (2,2,2)) }
  }
  it should "seg2 then seg2 (same)" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/outerdot_re"),
      () => mk()) { c => new OuterDotReentrantTester(c, (2,2,2), (2,2,2)) }
  }
  it should "seg1 then seg2 (smaller then larger)" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/outerdot_re"),
      () => mk()) { c => new OuterDotReentrantTester(c, (1,1,1), (2,2,2)) }
  }
}

class OuterDotVMESpec extends ChiselFlatSpec {
  implicit val p: Parameters = new De10Config(1, 1) ++ new CoreConfig ++ new MiniConfig

  val scenarios = Seq(
    ("k0nonempty", Seq(0,0),Seq(2,3),Seq(0,1,2), Seq(0,1),Seq(10,20),Seq(0,1,2), 2,2,2),
    ("k0empty_Bempty", Seq(0),Seq(3),Seq(0,0,1), Seq(1),Seq(20),Seq(0,0,1), 2,1,1),
    ("k0empty_Bnonempty", Seq(0),Seq(3),Seq(0,0,1), Seq(0,1),Seq(99,20),Seq(0,1,2), 2,1,2),
    ("seg1", Seq(0),Seq(2),Seq(0,1), Seq(0),Seq(10),Seq(0,1), 1,1,1)
  )
  val bpPeriods = Seq(1, 3)
  val lats = Seq(0, 6)

  behavior of "OuterDot via real VME"
  for ((nm, iA,vA,pA, iB,vB,pB, seg,nA,nB) <- scenarios; bp <- bpPeriods; lat <- lats) {
    it should s"$nm bp=$bp lat=$lat" in {
      Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/outerdot_vme"),
        () => new OuterDotVMEHarness(iA,vA,pA, iB,vB,pB, lat)) {
        c => new OuterDotVMETester(c, seg, nA, nB, bp, s"$nm bp=$bp lat=$lat")
      }
    }
  }
}
