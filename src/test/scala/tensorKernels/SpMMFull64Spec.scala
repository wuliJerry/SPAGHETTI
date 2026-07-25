package tensorKernels

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import config._
import node.vecN
import shell._
import dnn.memory.CoreConfig

/** Full SpMM chain at the ZC706 hardware config (PynqConfig: 64-bit AXI, 2 words
  * per beat, a 16-word tensor row = 8 beats). This is the width the FPGA runs at;
  * the original SpMMFullSpec runs at 512-bit (row = 1 beat) and cannot see
  * multi-beat assembly bugs. Reproduces the on-board seg2 hang pre-fix.
  *
  * Memory layout (matches the 512-bit spec byte-for-byte):
  *   input array j (ptrA,indA,valA,ptrB,indB,valB) at byte j*64 (= 16 words padded)
  *   output arrays row/col/val at bytes 1024/1088/1152
  */
class AXIMemRWG(readWords: Seq[Seq[Int]], nWrBeats: Int)(implicit p: Parameters) extends Module {
  val mp = p(ShellKey).memParams
  val io = IO(new Bundle {
    val mem = Flipped(new AXIMaster(mp))
    val wr  = Output(Vec(nWrBeats, UInt(mp.dataBits.W)))
  })
  val szC = log2Ceil(mp.dataBits / 8)
  val wpb = mp.dataBits / 32                       // words per beat
  val WBASE = 1024 >> szC                          // write region starts at byte 1024
  def toBeats(words: Seq[Int]): Seq[BigInt] = {
    val padded = words ++ Seq.fill((16 - words.length % 16) % 16 max 0)(0)
    val p16 = if (padded.length < 16) padded ++ Seq.fill(16 - padded.length)(0) else padded
    p16.grouped(wpb).map(_.zipWithIndex.map { case (x, i) =>
      (BigInt(x) & BigInt("FFFFFFFF", 16)) << (32 * i) }.foldLeft(BigInt(0))(_ | _)).toSeq
  }
  val romSeq = readWords.flatMap(toBeats)
  val rom = VecInit((romSeq ++ Seq.fill(8)(BigInt(0))).map(_.U(mp.dataBits.W)))
  val ram = Reg(Vec(nWrBeats, UInt(mp.dataBits.W)))
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
      when(widx < nWrBeats.U) { ram(widx) := io.mem.w.bits.data }
      wbeat := wbeat + 1.U
      when(io.mem.w.bits.last) { wstate := wResp }
    } }
    is(wResp) { when(io.mem.b.fire()) { wstate := wIdle } }
  }
}

class SpMM64Harness(indA: Seq[Int], valA: Seq[Int], ptrA: Seq[Int],
                    indB: Seq[Int], valB: Seq[Int], ptrB: Seq[Int], sorterDepth: Int)
                   (implicit p: Parameters) extends Module {
  val mp = p(ShellKey).memParams
  val nWrBeats = (3 * 64) >> log2Ceil(mp.dataBits / 8)   // 3 padded output arrays
  val io = IO(new Bundle {
    val start = Input(Bool())
    val nnz_A = Input(UInt(32.W)); val nnz_B = Input(UInt(32.W)); val segSize = Input(UInt(32.W))
    val done = Output(Bool())
    val multDone = Output(Bool())
    val inStrDone = Output(Bool())
    val outLen = Output(UInt(32.W))
    val wr = Output(Vec(nWrBeats, UInt(mp.dataBits.W)))
  })

  val spmm = Module(new SpMM(numSegments = 1, numSorter = 1, numVC = 1, VCDepth = 2, sorterDepth = sorterDepth)(new vecN(1, 0, false)))
  val vme = Module(new VME())
  val mem = Module(new AXIMemRWG(Seq(ptrA, indA, valA, ptrB, indB, valB), nWrBeats))
  mem.io.mem.ar <> vme.io.mem.ar
  vme.io.mem.r <> mem.io.mem.r
  mem.io.mem.aw <> vme.io.mem.aw
  mem.io.mem.w <> vme.io.mem.w
  vme.io.mem.b <> mem.io.mem.b

  vme.io.vme.rd(0) <> spmm.io.vme_rd_ptr(0)
  vme.io.vme.rd(1) <> spmm.io.vme_rd_ind(0)
  vme.io.vme.rd(2) <> spmm.io.vme_rd_val(0)
  vme.io.vme.rd(3) <> spmm.io.vme_rd_ptr(1)
  vme.io.vme.rd(4) <> spmm.io.vme_rd_ind(1)
  vme.io.vme.rd(5) <> spmm.io.vme_rd_val(1)
  vme.io.vme.wr(0) <> spmm.io.vme_wr_row(0)
  vme.io.vme.wr(1) <> spmm.io.vme_wr_col(0)
  vme.io.vme.wr(2) <> spmm.io.vme_wr_val(0)

  spmm.io.start := io.start
  spmm.io.nnz_A(0) := io.nnz_A; spmm.io.nnz_B(0) := io.nnz_B; spmm.io.segSize(0) := io.segSize
  spmm.io.ptr_A_BaseAddr(0) := (0 * 64).U; spmm.io.ind_A_BaseAddr(0) := (1 * 64).U; spmm.io.val_A_BaseAddr(0) := (2 * 64).U
  spmm.io.ptr_B_BaseAddr(0) := (3 * 64).U; spmm.io.ind_B_BaseAddr(0) := (4 * 64).U; spmm.io.val_B_BaseAddr(0) := (5 * 64).U
  spmm.io.outBaseAddr_row(0) := 1024.U; spmm.io.outBaseAddr_col(0) := (1024 + 64).U; spmm.io.outBaseAddr_val(0) := (1024 + 128).U

  io.done := spmm.io.done
  io.multDone := spmm.io.multiplicationDone
  io.inStrDone := spmm.io.inStreamingDone
  io.outLen := spmm.io.outDMA_len(0)
  io.wr := mem.io.wr
}

/** Runs one launch; checks done + outLen + the exact (row,col,val) set. */
class SpMM64Tester(c: SpMM64Harness, segSize: Int, nnzA: Int, nnzB: Int,
                   expected: Seq[(Int, Int, Int)], label: String)
  extends PeekPokeTester(c) {
  val wpb = 2 // 64-bit bus
  val beatsPerArray = 8
  poke(c.io.segSize, segSize); poke(c.io.nnz_A, nnzA); poke(c.io.nnz_B, nnzB)
  poke(c.io.start, 0); step(2); poke(c.io.start, 1); step(1); poke(c.io.start, 0)
  var doneSeen = false; var doneCyc = -1
  var cyc = 0
  while (cyc < 3000 && !doneSeen) {
    if (peek(c.io.done) == 1) { doneSeen = true; doneCyc = cyc }
    step(1); cyc += 1
  }
  step(20) // let trailing writes land
  val nnz = peek(c.io.outLen).toInt
  def word(arrayIdx: Int, w: Int): BigInt = {
    val beat = peek(c.io.wr(arrayIdx * beatsPerArray + w / wpb))
    (beat >> (32 * (w % wpb))) & BigInt("FFFFFFFF", 16)
  }
  val n = nnz.max(0).min(8)
  val coo = (0 until n).map(i => (word(0, i).toInt, word(1, i).toInt, word(2, i).toInt))
  println(s"[SpMM64 $label] ${if (!doneSeen) "*** HANG (done never) ***" else s"DONE@$doneCyc"} outLen=$nnz coo=$coo expected=$expected")
  expect(doneSeen, s"$label: done must assert")
  if (doneSeen) {
    expect(nnz == expected.length, s"$label: outLen $nnz != ${expected.length}")
    if (nnz == expected.length) expect(coo == expected, s"$label: coo mismatch")
  }
}

class SpMMFull64Spec extends ChiselFlatSpec {
  implicit val p: Parameters = new PynqConfig(1, 1) ++ new CoreConfig ++ new MiniConfig

  behavior of "Full SpMM chain at ZC706 width (64-bit AXI, multi-beat rows)"

  it should "seg1 baseline: (0,0,10)" in {
    assert(Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm64"),
      () => new SpMM64Harness(Seq(0),Seq(1),Seq(0,1), Seq(0),Seq(10),Seq(0,1), 64)) {
      c => new SpMM64Tester(c, 1, 1, 1, Seq((0,0,10)), "seg1") })
  }
  it should "seg1 third-element (the on-board discriminator): 3 outputs" in {
    // A col0 = rows 0,1,2 (ind spans 2 beats); B row0 = (col0,10)
    assert(Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm64"),
      () => new SpMM64Harness(Seq(0,1,2),Seq(1,2,3),Seq(0,3), Seq(0),Seq(10),Seq(0,1), 64)) {
      c => new SpMM64Tester(c, 1, 3, 1, Seq((0,0,10),(1,0,20),(2,0,30)), "seg1_3rd_elem") })
  }
  it should "seg2 both-cols-nonempty (the on-board hang): (0,0,10),(1,1,40)" in {
    assert(Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm64"),
      () => new SpMM64Harness(Seq(0,1),Seq(1,2),Seq(0,1,2), Seq(0,1),Seq(10,20),Seq(0,1,2), 64)) {
      c => new SpMM64Tester(c, 2, 2, 2, Seq((0,0,10),(1,1,40)), "seg2_distinct") })
  }
  it should "seg2 k0-empty (the on-board wrong-data case): (0,1,20)" in {
    // A: col0 empty, col1=(row0,1); B: row0 empty, row1=(col1,20)
    assert(Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm64"),
      () => new SpMM64Harness(Seq(0),Seq(1),Seq(0,0,1), Seq(1),Seq(20),Seq(0,0,1), 64)) {
      c => new SpMM64Tester(c, 2, 1, 1, Seq((0,1,20)), "seg2_k0empty") })
  }
  it should "seg2 accumulation: (0,1,100)" in {
    assert(Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmm64"),
      () => new SpMM64Harness(Seq(0,0),Seq(1,3),Seq(0,1,2), Seq(1,1),Seq(10,30),Seq(0,1,2), 64)) {
      c => new SpMM64Tester(c, 2, 2, 2, Seq((0,1,100)), "seg2_accum") })
  }
}
