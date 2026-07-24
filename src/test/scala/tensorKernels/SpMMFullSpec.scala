package tensorKernels

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import config._
import node.vecN
import shell._
import dnn.memory.CoreConfig

/** Behavioral AXI read+write memory. Reads served from a beat ROM; writes stored
  * into a small RAM (beats 16..) that the tester can read back. Independent
  * read/write FSMs (VME drives ar/r and aw/w/b concurrently). */
class AXIMemRW(readBeats: Seq[BigInt], nWr: Int)(implicit p: Parameters) extends Module {
  val mp = p(ShellKey).memParams
  val io = IO(new Bundle {
    val mem = Flipped(new AXIMaster(mp))
    val wr  = Output(Vec(nWr, UInt(mp.dataBits.W)))
  })
  val szC = log2Ceil(mp.dataBits / 8)
  val WBASE = 16 // write beats start at index 16 (byte 1024)
  val rom = VecInit((readBeats ++ Seq.fill(8)(BigInt(0))).map(_.U(mp.dataBits.W)))
  val ram = Reg(Vec(nWr, UInt(mp.dataBits.W)))
  io.wr := ram

  // ---- read FSM ----
  val rIdle :: rData :: Nil = Enum(2)
  val rstate = RegInit(rIdle)
  val rbeat = Reg(UInt(16.W)); val rlen = Reg(UInt(mp.lenBits.W))
  io.mem.ar.ready := rstate === rIdle
  io.mem.r.valid := rstate === rData
  io.mem.r.bits.data := rom(rbeat)
  io.mem.r.bits.last := (rstate === rData) && (rlen === 0.U)
  io.mem.r.bits.resp := 0.U; io.mem.r.bits.id := 0.U; io.mem.r.bits.user := 0.U
  switch(rstate) {
    is(rIdle) { when(io.mem.ar.fire()) { rbeat := io.mem.ar.bits.addr >> szC.U; rlen := io.mem.ar.bits.len; rstate := rData } }
    is(rData) { when(io.mem.r.fire()) { rbeat := rbeat + 1.U; when(rlen === 0.U) { rstate := rIdle }.otherwise { rlen := rlen - 1.U } } }
  }

  // ---- write FSM ----
  val wIdle :: wData :: wResp :: Nil = Enum(3)
  val wstate = RegInit(wIdle)
  val wbeat = Reg(UInt(16.W))
  io.mem.aw.ready := wstate === wIdle
  io.mem.w.ready := wstate === wData
  io.mem.b.valid := wstate === wResp
  io.mem.b.bits.resp := 0.U; io.mem.b.bits.id := 0.U; io.mem.b.bits.user := 0.U
  val widx = wbeat - WBASE.U
  switch(wstate) {
    is(wIdle) { when(io.mem.aw.fire()) { wbeat := io.mem.aw.bits.addr >> szC.U; wstate := wData } }
    is(wData) { when(io.mem.w.fire()) {
      when(widx < nWr.U) { ram(widx) := io.mem.w.bits.data }
      wbeat := wbeat + 1.U
      when(io.mem.w.bits.last) { wstate := wResp }
    } }
    is(wResp) { when(io.mem.b.fire()) { wstate := wIdle } }
  }
}

class SpMMHarness(indA: Seq[Int], valA: Seq[Int], ptrA: Seq[Int],
                  indB: Seq[Int], valB: Seq[Int], ptrB: Seq[Int], sorterDepth: Int)
                 (implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val nnz_A = Input(UInt(32.W)); val nnz_B = Input(UInt(32.W)); val segSize = Input(UInt(32.W))
    val done = Output(Bool())
    val multDone = Output(Bool())
    val inStrDone = Output(Bool())
    val outLen = Output(UInt(32.W))
    val wr = Output(Vec(3, UInt(p(ShellKey).memParams.dataBits.W)))
  })
  def pack(w: Seq[Int]): BigInt =
    w.zipWithIndex.map { case (x, i) => (BigInt(x) & BigInt("FFFFFFFF", 16)) << (32 * i) }.foldLeft(BigInt(0))(_ | _)

  val spmm = Module(new SpMM(numSegments = 1, numSorter = 1, numVC = 1, VCDepth = 2, sorterDepth = sorterDepth)(new vecN(1, 0, false)))
  val vme = Module(new VME())
  // read beats 0..5: ptrA,indA,valA,ptrB,indB,valB
  val mem = Module(new AXIMemRW(Seq(pack(ptrA), pack(indA), pack(valA), pack(ptrB), pack(indB), pack(valB)), nWr = 3))
  mem.io.mem.ar <> vme.io.mem.ar
  vme.io.mem.r <> mem.io.mem.r
  mem.io.mem.aw <> vme.io.mem.aw
  mem.io.mem.w <> vme.io.mem.w
  vme.io.mem.b <> mem.io.mem.b

  // read clients: 0 ptrA,1 indA,2 valA,3 ptrB,4 indB,5 valB
  vme.io.vme.rd(0) <> spmm.io.vme_rd_ptr(0)
  vme.io.vme.rd(1) <> spmm.io.vme_rd_ind(0)
  vme.io.vme.rd(2) <> spmm.io.vme_rd_val(0)
  vme.io.vme.rd(3) <> spmm.io.vme_rd_ptr(1)
  vme.io.vme.rd(4) <> spmm.io.vme_rd_ind(1)
  vme.io.vme.rd(5) <> spmm.io.vme_rd_val(1)
  // write clients: 0 row,1 col,2 val
  vme.io.vme.wr(0) <> spmm.io.vme_wr_row(0)
  vme.io.vme.wr(1) <> spmm.io.vme_wr_col(0)
  vme.io.vme.wr(2) <> spmm.io.vme_wr_val(0)

  spmm.io.start := io.start
  spmm.io.nnz_A(0) := io.nnz_A; spmm.io.nnz_B(0) := io.nnz_B; spmm.io.segSize(0) := io.segSize
  spmm.io.ptr_A_BaseAddr(0) := (0 * 64).U; spmm.io.ind_A_BaseAddr(0) := (1 * 64).U; spmm.io.val_A_BaseAddr(0) := (2 * 64).U
  spmm.io.ptr_B_BaseAddr(0) := (3 * 64).U; spmm.io.ind_B_BaseAddr(0) := (4 * 64).U; spmm.io.val_B_BaseAddr(0) := (5 * 64).U
  spmm.io.outBaseAddr_row(0) := (16 * 64).U; spmm.io.outBaseAddr_col(0) := (17 * 64).U; spmm.io.outBaseAddr_val(0) := (18 * 64).U

  io.done := spmm.io.done
  io.multDone := spmm.io.multiplicationDone
  io.inStrDone := spmm.io.inStreamingDone
  io.outLen := spmm.io.outDMA_len(0)
  io.wr := mem.io.wr
}

class SpMMFullTester(c: SpMMHarness, segSize: Int, nnzA: Int, nnzB: Int, label: String)
  extends PeekPokeTester(c) {
  poke(c.io.segSize, segSize); poke(c.io.nnz_A, nnzA); poke(c.io.nnz_B, nnzB)
  poke(c.io.start, 0); step(2); poke(c.io.start, 1); step(1); poke(c.io.start, 0)
  var doneSeen = false; var doneCyc = -1
  var multCyc = -1; var inStrCyc = -1
  for (t <- 0 until 2000) {
    if (multCyc < 0 && peek(c.io.multDone) == 1) multCyc = t
    if (inStrCyc < 0 && peek(c.io.inStrDone) == 1) inStrCyc = t
    if (!doneSeen && peek(c.io.done) == 1) { doneSeen = true; doneCyc = t }
    step(1)
  }
  def words(beat: BigInt, n: Int): Seq[BigInt] = (0 until n).map(i => (beat >> (32 * i)) & BigInt("FFFFFFFF", 16))
  val nnz = peek(c.io.outLen)
  val rowB = peek(c.io.wr(0)); val colB = peek(c.io.wr(1)); val valB = peek(c.io.wr(2))
  val n = math.max(nnz.toInt, 0).min(4)
  val coo = (words(rowB, n) zip words(colB, n) zip words(valB, n)).map { case ((r, cc), v) => (r, cc, v) }
  println(s"[SpMM $label] ${if (!doneSeen) "*** HANG (done never) ***" else "DONE"} done=$doneSeen@$doneCyc inStrDone@$inStrCyc multDone@$multCyc outLen=$nnz coo=$coo")
}

class SpMMFullSpec extends ChiselFlatSpec {
  implicit val p: Parameters = new De10Config(1, 1) ++ new CoreConfig ++ new MiniConfig

  behavior of "Full SpMM chain (integer), fresh single launch"

  it should "seg1 (baseline)" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm"),
      () => new SpMMHarness(Seq(0),Seq(1),Seq(0,1), Seq(0),Seq(10),Seq(0,1), 64)) {
      c => new SpMMFullTester(c, 1, 1, 1, "seg1") }
  }
  it should "seg2 both-cols-nonempty distinct rows (expect (0,0,10),(1,1,40))" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm"),
      () => new SpMMHarness(Seq(0,1),Seq(1,2),Seq(0,1,2), Seq(0,1),Seq(10,20),Seq(0,1,2), 64)) {
      c => new SpMMFullTester(c, 2, 2, 2, "seg2_distinct") }
  }
  it should "seg2 multi-output-per-column (3 outputs, sorter/reducer stress)" in {
    // A CSC: col0=(row0,1),(row1,1) ; col1=(row0,1).  B CSR: row0=(col0,10) ; row1=(col1,20)
    // k0 -> (0,0,10),(1,0,10) ; k1 -> (0,1,20)
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm"),
      () => new SpMMHarness(Seq(0,1,0),Seq(1,1,1),Seq(0,2,3), Seq(0,1),Seq(10,20),Seq(0,1,2), 64)) {
      c => new SpMMFullTester(c, 2, 3, 2, "seg2_multi") }
  }
  it should "seg2 accumulation into (0,1)" in {
    // A CSC: col0=(row0,v1), col1=(row0,v3) ; B CSR: row0=(col1,v10), row1=(col1,v30)
    // k0: (0,1, 1*10=10) ; k1: (0,1, 3*30=90) -> accumulate (0,1)=100
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm"),
      () => new SpMMHarness(Seq(0,0),Seq(1,3),Seq(0,1,2), Seq(1,1),Seq(10,30),Seq(0,1,2), 64)) {
      c => new SpMMFullTester(c, 2, 2, 2, "seg2_accum") }
  }
}
