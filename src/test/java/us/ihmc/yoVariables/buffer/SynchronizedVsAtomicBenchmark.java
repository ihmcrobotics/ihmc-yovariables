package us.ihmc.yoVariables.buffer;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Manual timing comparison between {@link YoBufferVariableEntry}'s {@code synchronized}
 * write/bounds-tracking path and the lock-free {@link AtomicBufferVariableEntryPrototype}.
 * <p>
 * This is deliberately <b>not</b> a JUnit test: timing results are inherently noisy (JIT warmup, GC,
 * whatever else is running on the machine), so this prints a comparison for a human to read rather
 * than asserting a specific speedup ratio, which would make for a flaky test. It is not picked up by
 * {@code gradle test} (no {@code @Test} annotations) - run it directly, e.g.:
 * </p>
 * <pre>
 * gradle :ihmc-yovariables-test:run -PmainClass=us.ihmc.yoVariables.buffer.SynchronizedVsAtomicBenchmark
 * </pre>
 * <p>
 * or, if that project property isn't wired up in this version of the build, extract the resolved
 * classpath from {@code gradle :ihmc-yovariables-test:run --info} (look for the "Command:" line) and
 * run {@code java -cp <that classpath> us.ihmc.yoVariables.buffer.SynchronizedVsAtomicBenchmark}
 * directly.
 * </p>
 * <p>
 * Simulates the one confirmed real usage pattern for this class: one writer thread ticking the
 * buffer (e.g. a simulation/timer thread) concurrently with one reader thread polling bounds/data
 * (e.g. a render thread), matching {@code RDXLoggingDevelopmentUI}'s
 * {@code timer.scheduleAtFixedRate(...) -> yoBuffer.tickAndWriteIntoBuffer()} alongside its
 * {@code render()} method.
 * </p>
 */
public class SynchronizedVsAtomicBenchmark
{
   private static final int BUFFER_SIZE = 10_000;
   private static final int WARMUP_OPS = 2_000_000;
   private static final int MEASURED_OPS = 5_000_000;
   private static final int TRIALS = 5;

   public static void main(String[] args) throws InterruptedException
   {
      System.out.println("Warming up (synchronized)...");
      runTrial(WARMUP_OPS, true);
      System.out.println("Warming up (atomic)...");
      runTrial(WARMUP_OPS, false);

      long[] syncWriterNanos = new long[TRIALS];
      long[] atomicWriterNanos = new long[TRIALS];
      long[] syncReaderOps = new long[TRIALS];
      long[] atomicReaderOps = new long[TRIALS];

      for (int trial = 0; trial < TRIALS; trial++)
      {
         Result sync = runTrial(MEASURED_OPS, true);
         Result atomic = runTrial(MEASURED_OPS, false);

         syncWriterNanos[trial] = sync.writerElapsedNanos;
         atomicWriterNanos[trial] = atomic.writerElapsedNanos;
         syncReaderOps[trial] = sync.readerOpsCompleted;
         atomicReaderOps[trial] = atomic.readerOpsCompleted;

         System.out.printf("Trial %d: synchronized writer = %.1f ms (%.1f Mops/s), reader ops = %d%n",
                            trial + 1,
                            sync.writerElapsedNanos / 1e6,
                            MEASURED_OPS / (sync.writerElapsedNanos / 1e3),
                            sync.readerOpsCompleted);
         System.out.printf("Trial %d: atomic        writer = %.1f ms (%.1f Mops/s), reader ops = %d%n",
                            trial + 1,
                            atomic.writerElapsedNanos / 1e6,
                            MEASURED_OPS / (atomic.writerElapsedNanos / 1e3),
                            atomic.readerOpsCompleted);
      }

      System.out.println();
      System.out.println("=== Summary over " + TRIALS + " trials (" + MEASURED_OPS + " writer ops each) ===");
      summarize("synchronized writer", syncWriterNanos);
      summarize("atomic writer      ", atomicWriterNanos);
      summarizeCount("synchronized reader ops completed", syncReaderOps);
      summarizeCount("atomic reader ops completed       ", atomicReaderOps);

      double meanSync = mean(syncWriterNanos);
      double meanAtomic = mean(atomicWriterNanos);
      System.out.printf("%nWriter throughput ratio (synchronized time / atomic time): %.2fx%n", meanSync / meanAtomic);
   }

   private static Result runTrial(int writerOps, boolean useSynchronized) throws InterruptedException
   {
      YoRegistry registry = new YoRegistry("benchmark");
      YoDouble variable = new YoDouble("value", registry);
      YoBufferVariableEntry syncEntry = useSynchronized ? new YoBufferVariableEntry(variable, BUFFER_SIZE) : null;
      AtomicBufferVariableEntryPrototype atomicEntry = useSynchronized ? null : new AtomicBufferVariableEntryPrototype(BUFFER_SIZE);

      CountDownLatch start = new CountDownLatch(1);
      AtomicBoolean stopReader = new AtomicBoolean(false);
      AtomicLong readerOpsCompleted = new AtomicLong();
      long[] writerElapsedNanosHolder = new long[1];

      Thread reader = new Thread(() ->
      {
         awaitUninterruptibly(start);
         java.util.Random random = new java.util.Random(1);
         while (!stopReader.get())
         {
            int index = random.nextInt(BUFFER_SIZE);
            if (useSynchronized)
            {
               syncEntry.haveBoundsChanged();
               syncEntry.resetBoundsChangedFlag();
               syncEntry.readBufferAt(index);
            }
            else
            {
               atomicEntry.haveBoundsChanged();
               atomicEntry.resetBoundsChangedFlag();
               atomicEntry.readValueAt(index);
            }
            readerOpsCompleted.incrementAndGet();
         }
      }, "reader");
      reader.setDaemon(true);
      reader.start();

      Thread writer = new Thread(() ->
      {
         awaitUninterruptibly(start);
         java.util.Random random = new java.util.Random(2);
         long startTime = System.nanoTime();
         for (int i = 0; i < writerOps; i++)
         {
            double value = random.nextDouble() * 1000.0;
            int index = i % BUFFER_SIZE;
            if (useSynchronized)
               syncEntry.writeBufferAt(value, index);
            else
               atomicEntry.writeValueAt(value, index);
         }
         writerElapsedNanosHolder[0] = System.nanoTime() - startTime;
      }, "writer");
      writer.setDaemon(true);
      writer.start();

      start.countDown();
      writer.join();
      stopReader.set(true);
      reader.join();

      return new Result(writerElapsedNanosHolder[0], readerOpsCompleted.get());
   }

   private static void awaitUninterruptibly(CountDownLatch latch)
   {
      try
      {
         latch.await();
      }
      catch (InterruptedException e)
      {
         Thread.currentThread().interrupt();
      }
   }

   private static void summarize(String label, long[] nanos)
   {
      double meanMs = mean(nanos) / 1e6;
      double minMs = min(nanos) / 1e6;
      double maxMs = max(nanos) / 1e6;
      System.out.printf("%s: mean = %.1f ms, min = %.1f ms, max = %.1f ms%n", label, meanMs, minMs, maxMs);
   }

   private static void summarizeCount(String label, long[] counts)
   {
      System.out.printf("%s: mean = %.0f, min = %d, max = %d%n", label, mean(counts), min(counts), max(counts));
   }

   private static double mean(long[] values)
   {
      long sum = 0;
      for (long value : values)
         sum += value;
      return (double) sum / values.length;
   }

   private static long min(long[] values)
   {
      long min = Long.MAX_VALUE;
      for (long value : values)
         min = Math.min(min, value);
      return min;
   }

   private static long max(long[] values)
   {
      long max = Long.MIN_VALUE;
      for (long value : values)
         max = Math.max(max, value);
      return max;
   }

   private static final class Result
   {
      private final long writerElapsedNanos;
      private final long readerOpsCompleted;

      private Result(long writerElapsedNanos, long readerOpsCompleted)
      {
         this.writerElapsedNanos = writerElapsedNanos;
         this.readerOpsCompleted = readerOpsCompleted;
      }
   }
}
