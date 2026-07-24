package tensorKernels

import chisel3._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import config._
import node.FPvecN
import FPU.FType
import shell._
import dnn.memory.CoreConfig

/** Full SpMM chain with the exact hardware FP shape FPvecN(1, FType(8,24)),
  * routed through the real VME + behavioral AXI read/write memory. */
class SpMMHarnessFP(indA: Seq[Int], valA: Seq[Int], ptrA: Seq[Int],
                    indB: Seq[Int], valB: Seq[Int], ptrB: Seq[Int], sorterDepth: Int)
                   (implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val nnz_A = Input(UInt(32.W)); val nnz_B = Input(UInt(32.W)); val segSize = Input(UInt(32.W))
    val done = Output(Bool()); val multDone = Output(Bool()); val inStrDone = Output(Bool())
    val outLen = Output(UInt(32.W))
    val wr = Output(Vec(3, UInt(p(ShellKey).memParams.dataBits.W)))
  })
  def pack(w: Seq[Int]): BigInt =
    w.zipWithIndex.map { case (x, i) => (BigInt(x) & BigInt("FFFFFFFF", 16)) << (32 * i) }.foldLeft(BigInt(0))(_ | _)

  val spmm = Module(new SpMM(numSegments = 1, numSorter = 1, numVC = 1, VCDepth = 2, sorterDepth = sorterDepth)(new FPvecN(1, new FType(8, 24), 0)))
  val vme = Module(new VME())
  val mem = Module(new AXIMemRW(Seq(pack(ptrA), pack(indA), pack(valA), pack(ptrB), pack(indB), pack(valB)), nWr = 3))
  mem.io.mem.ar <> vme.io.mem.ar; vme.io.mem.r <> mem.io.mem.r
  mem.io.mem.aw <> vme.io.mem.aw; mem.io.mem.w <> vme.io.mem.w; vme.io.mem.b <> mem.io.mem.b
  vme.io.vme.rd(0) <> spmm.io.vme_rd_ptr(0); vme.io.vme.rd(1) <> spmm.io.vme_rd_ind(0); vme.io.vme.rd(2) <> spmm.io.vme_rd_val(0)
  vme.io.vme.rd(3) <> spmm.io.vme_rd_ptr(1); vme.io.vme.rd(4) <> spmm.io.vme_rd_ind(1); vme.io.vme.rd(5) <> spmm.io.vme_rd_val(1)
  vme.io.vme.wr(0) <> spmm.io.vme_wr_row(0); vme.io.vme.wr(1) <> spmm.io.vme_wr_col(0); vme.io.vme.wr(2) <> spmm.io.vme_wr_val(0)
  spmm.io.start := io.start
  spmm.io.nnz_A(0) := io.nnz_A; spmm.io.nnz_B(0) := io.nnz_B; spmm.io.segSize(0) := io.segSize
  spmm.io.ptr_A_BaseAddr(0) := (0*64).U; spmm.io.ind_A_BaseAddr(0) := (1*64).U; spmm.io.val_A_BaseAddr(0) := (2*64).U
  spmm.io.ptr_B_BaseAddr(0) := (3*64).U; spmm.io.ind_B_BaseAddr(0) := (4*64).U; spmm.io.val_B_BaseAddr(0) := (5*64).U
  spmm.io.outBaseAddr_row(0) := (16*64).U; spmm.io.outBaseAddr_col(0) := (17*64).U; spmm.io.outBaseAddr_val(0) := (18*64).U
  io.done := spmm.io.done; io.multDone := spmm.io.multiplicationDone; io.inStrDone := spmm.io.inStreamingDone
  io.outLen := spmm.io.outDMA_len(0); io.wr := mem.io.wr
}

class SpMMFullFPTester(c: SpMMHarnessFP, segSize: Int, nnzA: Int, nnzB: Int, label: String)
  extends PeekPokeTester(c) {
  poke(c.io.segSize, segSize); poke(c.io.nnz_A, nnzA); poke(c.io.nnz_B, nnzB)
  poke(c.io.start, 0); step(2); poke(c.io.start, 1); step(1); poke(c.io.start, 0)
  var doneSeen = false; var doneCyc = -1; var multCyc = -1; var inStrCyc = -1
  for (t <- 0 until 2000) {
    if (multCyc < 0 && peek(c.io.multDone) == 1) multCyc = t
    if (inStrCyc < 0 && peek(c.io.inStrDone) == 1) inStrCyc = t
    if (!doneSeen && peek(c.io.done) == 1) { doneSeen = true; doneCyc = t }
    step(1)
  }
  def words(beat: BigInt, n: Int): Seq[BigInt] = (0 until n).map(i => (beat >> (32*i)) & BigInt("FFFFFFFF", 16))
  def f(bits: BigInt): Float = java.lang.Float.intBitsToFloat(bits.toInt)
  val nnz = peek(c.io.outLen); val n = math.max(nnz.toInt, 0).min(4)
  val rowB = peek(c.io.wr(0)); val colB = peek(c.io.wr(1)); val valB = peek(c.io.wr(2))
  val coo = (words(rowB, n) zip words(colB, n) zip words(valB, n)).map { case ((r, cc), v) => (r, cc, f(v)) }
  println(s"[SpMM-FP $label] ${if (!doneSeen) "*** HANG (done never) ***" else "DONE"} done=$doneSeen@$doneCyc inStr@$inStrCyc mult@$multCyc outLen=$nnz coo(row,col,valF)=$coo")
}

class SpMMFullFPSpec extends ChiselFlatSpec {
  implicit val p: Parameters = new De10Config(1, 1) ++ new CoreConfig ++ new MiniConfig
  // float32 bit patterns
  val f1 = 0x3F800000; val f2 = 0x40000000; val f3 = 0x40400000
  val f10 = 0x41200000; val f20 = 0x41A00000; val f30 = 0x41F00000

  behavior of "Full SpMM chain (FP), fresh single launch"
  it should "seg1 baseline (expect (0,0,10.0))" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmmfp"),
      () => new SpMMHarnessFP(Seq(0),Seq(f1),Seq(0,1), Seq(0),Seq(f10),Seq(0,1), 64)) {
      c => new SpMMFullFPTester(c, 1, 1, 1, "seg1") }
  }
  it should "seg2 distinct rows (expect (0,0,10.0),(1,1,40.0)) -- FRESH SEG2 HANG HUNT" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmmfp"),
      () => new SpMMHarnessFP(Seq(0,1),Seq(f1,f2),Seq(0,1,2), Seq(0,1),Seq(f10,f20),Seq(0,1,2), 64)) {
      c => new SpMMFullFPTester(c, 2, 2, 2, "seg2_distinct") }
  }
  it should "seg2 accumulation into (0,1)=100.0" in {
    Driver.execute(Array("--backend-name","treadle","--target-dir","test_run_dir/spmmfp"),
      () => new SpMMHarnessFP(Seq(0,0),Seq(f1,f3),Seq(0,1,2), Seq(1,1),Seq(f10,f30),Seq(0,1,2), 64)) {
      c => new SpMMFullFPTester(c, 2, 2, 2, "seg2_accum") }
  }
}
