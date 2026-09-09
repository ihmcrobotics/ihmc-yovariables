package us.ihmc.yoVariables.buffer;

import java.util.Random;
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
 * Simulates the confirmed real usage pattern for this class - one writer thread ticking the buffer
 * (e.g. a simulation/timer thread) concurrently with reader thread(s) polling bounds/data (e.g. a
 * render thread) - across a couple of reader counts (see {@link #READER_COUNTS}), matching
 * {@code RDXLoggingDevelopmentUI}'s {@code timer.scheduleAtFixedRate(...) ->
 * yoBuffer.tickAndWriteIntoBuffer()} alongside its {@code render()} method.
 * </p>
 * <p>
 * Each trial runs for a fixed wall-clock duration rather than a fixed op count, with every writer
 * and reader thread counting how many operations it completed in that window. This is what makes
 * writer and reader throughput comparable to each other and across variants/reader-counts: an
 * earlier version of this benchmark gated trial duration on the writer finishing a fixed op count,
 * which meant a slower variant's reader(s) simply got more wall-clock time to accumulate ops in - a
 * confound that would be actively misleading for a reader-scaling comparison like this one.
 * </p>
 */
public class SynchronizedVsAtomicBenchmark
{
   private static final int BUFFER_SIZE = 10_000;
   private static final long WARMUP_DURATION_MILLIS = 4_000;
   private static final long TRIAL_DURATION_MILLIS = 1_000;
   private static final int TRIALS = 25;
   private static final int[] READER_COUNTS = {1, 4};

   public static void main(String[] args) throws InterruptedException
   {
      for (int readerCount : READER_COUNTS)
      {
         System.out.println();
         System.out.println("############ readers = " + readerCount + " ############");

         System.out.println("Warming up (synchronized)...");
         runTrial(WARMUP_DURATION_MILLIS, true, readerCount);
         System.out.println("Warming up (atomic)...");
         runTrial(WARMUP_DURATION_MILLIS, false, readerCount);

         long[] syncWriterOps = new long[TRIALS];
         long[] atomicWriterOps = new long[TRIALS];
         long[] syncReaderOpsTotal = new long[TRIALS];
         long[] atomicReaderOpsTotal = new long[TRIALS];

         for (int trial = 0; trial < TRIALS; trial++)
         {
            Result sync = runTrial(TRIAL_DURATION_MILLIS, true, readerCount);
            Result atomic = runTrial(TRIAL_DURATION_MILLIS, false, readerCount);

            syncWriterOps[trial] = sync.writerOpsCompleted;
            atomicWriterOps[trial] = atomic.writerOpsCompleted;
            syncReaderOpsTotal[trial] = sync.totalReaderOpsCompleted();
            atomicReaderOpsTotal[trial] = atomic.totalReaderOpsCompleted();

            System.out.printf("Trial %d: synchronized writer = %6.2f Mops/s, readers total = %6.2f Mops/s (%.2f Mops/s/reader)%n",
                               trial + 1,
                               opsPerSec(sync.writerOpsCompleted) / 1e6,
                               opsPerSec(sync.totalReaderOpsCompleted()) / 1e6,
                               opsPerSec(sync.totalReaderOpsCompleted()) / 1e6 / readerCount);
            System.out.printf("Trial %d: atomic        writer = %6.2f Mops/s, readers total = %6.2f Mops/s (%.2f Mops/s/reader)%n",
                               trial + 1,
                               opsPerSec(atomic.writerOpsCompleted) / 1e6,
                               opsPerSec(atomic.totalReaderOpsCompleted()) / 1e6,
                               opsPerSec(atomic.totalReaderOpsCompleted()) / 1e6 / readerCount);
         }

         System.out.println();
         System.out.println("=== Summary: readers = " + readerCount + ", " + TRIALS + " trials of " + TRIAL_DURATION_MILLIS + " ms each ===");
         summarizeThroughput("synchronized writer     ", syncWriterOps);
         summarizeThroughput("atomic writer            ", atomicWriterOps);
         summarizeThroughput("synchronized readers total", syncReaderOpsTotal);
         summarizeThroughput("atomic readers total      ", atomicReaderOpsTotal);

         double writerRatio = mean(atomicWriterOps) / mean(syncWriterOps);
         double readerRatio = mean(atomicReaderOpsTotal) / mean(syncReaderOpsTotal);
         System.out.printf("%nAtomic/synchronized throughput ratio - writer: %.2fx, readers (total): %.2fx%n", writerRatio, readerRatio);
      }
   }

   private static double opsPerSec(long ops)
   {
      return ops / (TRIAL_DURATION_MILLIS / 1000.0);
   }

   private static Result runTrial(long durationMillis, boolean useSynchronized, int readerCount) throws InterruptedException
   {
      YoRegistry registry = new YoRegistry("benchmark");
      YoDouble variable = new YoDouble("value", registry);
      YoBufferVariableEntry syncEntry = useSynchronized ? new YoBufferVariableEntry(variable, BUFFER_SIZE) : null;
      AtomicBufferVariableEntryPrototype atomicEntry = useSynchronized ? null : new AtomicBufferVariableEntryPrototype(BUFFER_SIZE);

      CountDownLatch start = new CountDownLatch(1);
      AtomicBoolean stop = new AtomicBoolean(false);
      AtomicLong writerOpsCompleted = new AtomicLong();
      AtomicLong[] readerOpsCompleted = new AtomicLong[readerCount];
      for (int i = 0; i < readerCount; i++)
         readerOpsCompleted[i] = new AtomicLong();

      Thread[] readers = new Thread[readerCount];
      for (int r = 0; r < readerCount; r++)
      {
         int readerIndex = r;
         readers[r] = new Thread(() ->
         {
            awaitUninterruptibly(start);
            Random random = new Random(100 + readerIndex);
            long ops = 0;
            while (!stop.get())
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
               ops++;
            }
            readerOpsCompleted[readerIndex].set(ops);
         }, "reader-" + readerIndex);
         readers[r].setDaemon(true);
      }

      Thread writer = new Thread(() ->
      {
         awaitUninterruptibly(start);
         Random random = new Random(2);
         long ops = 0;
         while (!stop.get())
         {
            double value = random.nextDouble() * 1000.0;
            int index = (int) (ops % BUFFER_SIZE);
            if (useSynchronized)
               syncEntry.writeBufferAt(value, index);
            else
               atomicEntry.writeValueAt(value, index);
            ops++;
         }
         writerOpsCompleted.set(ops);
      }, "writer");
      writer.setDaemon(true);

      for (Thread reader : readers)
         reader.start();
      writer.start();

      start.countDown();
      Thread.sleep(durationMillis);
      stop.set(true);

      writer.join();
      for (Thread reader : readers)
         reader.join();

      long[] perReaderOps = new long[readerCount];
      for (int i = 0; i < readerCount; i++)
         perReaderOps[i] = readerOpsCompleted[i].get();

      return new Result(writerOpsCompleted.get(), perReaderOps);
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

   private static void summarizeThroughput(String label, long[] ops)
   {
      double meanOps = mean(ops);
      double coefficientOfVariation = meanOps == 0.0 ? 0.0 : stddev(ops, meanOps) / meanOps * 100.0;
      System.out.printf("%s: mean = %6.2f Mops/s, stddev = %5.2f Mops/s (%.1f%%), min = %6.2f Mops/s, max = %6.2f Mops/s%n",
                         label,
                         opsPerSec((long) meanOps) / 1e6,
                         opsPerSec((long) stddev(ops, meanOps)) / 1e6,
                         coefficientOfVariation,
                         opsPerSec(min(ops)) / 1e6,
                         opsPerSec(max(ops)) / 1e6);
   }

   private static double mean(long[] values)
   {
      long sum = 0;
      for (long value : values)
         sum += value;
      return (double) sum / values.length;
   }

   /** Sample standard deviation (n-1 denominator), given a precomputed mean. */
   private static double stddev(long[] values, double meanValue)
   {
      if (values.length < 2)
         return 0.0;

      double sumOfSquaredDeviations = 0.0;
      for (long value : values)
      {
         double deviation = value - meanValue;
         sumOfSquaredDeviations += deviation * deviation;
      }
      return Math.sqrt(sumOfSquaredDeviations / (values.length - 1));
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
      private final long writerOpsCompleted;
      private final long[] perReaderOpsCompleted;

      private Result(long writerOpsCompleted, long[] perReaderOpsCompleted)
      {
         this.writerOpsCompleted = writerOpsCompleted;
         this.perReaderOpsCompleted = perReaderOpsCompleted;
      }

      private long totalReaderOpsCompleted()
      {
         long total = 0;
         for (long ops : perReaderOpsCompleted)
            total += ops;
         return total;
      }
   }
}
