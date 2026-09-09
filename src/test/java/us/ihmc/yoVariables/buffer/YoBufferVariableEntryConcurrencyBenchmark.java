package us.ihmc.yoVariables.buffer;

import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Manual timing harness for {@link YoBufferVariableEntry}'s write/bounds-tracking concurrency
 * strategy, whatever that strategy currently is (as of this writing: lock-free, via
 * {@link java.util.concurrent.atomic.AtomicLongArray}/{@link java.util.concurrent.atomic.AtomicReference}/
 * {@link java.util.concurrent.atomic.AtomicBoolean} - it was previously {@code synchronized}).
 * <p>
 * Deliberately benchmarks only the real class rather than maintaining a second implementation
 * in-source for comparison: to see the effect of a concurrency-strategy change, run this benchmark
 * once on the branch/commit before the change and once after, and compare the two printed reports.
 * Keeping this in source means that if the strategy is ever changed again (including reverted), the
 * same benchmark immediately shows what changed, without needing to keep a synthetic alternative
 * implementation around forever just to have something to diff against.
 * </p>
 * <p>
 * This is deliberately <b>not</b> a JUnit test: timing results are inherently noisy (JIT warmup, GC,
 * whatever else is running on the machine), so this prints a report for a human to read rather than
 * asserting on specific numbers, which would make for a flaky test. It is not picked up by
 * {@code gradle test} (no {@code @Test} annotations) - run it directly, e.g.:
 * </p>
 * <pre>
 * gradle :ihmc-yovariables-test:run -PmainClass=us.ihmc.yoVariables.buffer.YoBufferVariableEntryConcurrencyBenchmark
 * </pre>
 * <p>
 * or, if that project property isn't wired up in this version of the build, extract the resolved
 * classpath from {@code gradle :ihmc-yovariables-test:run --info} (look for the "Command:" line) and
 * run {@code java -cp <that classpath> us.ihmc.yoVariables.buffer.YoBufferVariableEntryConcurrencyBenchmark}
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
 * Each trial runs for a fixed wall-clock duration rather than a fixed op count, with every writer and
 * reader thread counting how many operations it completed in that window, so writer and reader
 * throughput are directly comparable to each other. Reports mean/stddev/min/max (as coefficient of
 * variation alongside stddev) rather than just a mean: in practice, low trial counts have produced
 * summary numbers that didn't hold up once the trial count was increased, so the noise needs to be
 * visible, not just the average.
 * </p>
 */
public class YoBufferVariableEntryConcurrencyBenchmark
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

         System.out.println("Warming up...");
         runTrial(WARMUP_DURATION_MILLIS, readerCount);

         long[] writerOps = new long[TRIALS];
         long[] readerOpsTotal = new long[TRIALS];

         for (int trial = 0; trial < TRIALS; trial++)
         {
            Result result = runTrial(TRIAL_DURATION_MILLIS, readerCount);

            writerOps[trial] = result.writerOpsCompleted;
            readerOpsTotal[trial] = result.totalReaderOpsCompleted();

            System.out.printf("Trial %2d: writer = %6.2f Mops/s, readers total = %6.2f Mops/s (%.2f Mops/s/reader)%n",
                               trial + 1,
                               opsPerSec(result.writerOpsCompleted) / 1e6,
                               opsPerSec(result.totalReaderOpsCompleted()) / 1e6,
                               opsPerSec(result.totalReaderOpsCompleted()) / 1e6 / readerCount);
         }

         System.out.println();
         System.out.println("=== Summary: readers = " + readerCount + ", " + TRIALS + " trials of " + TRIAL_DURATION_MILLIS + " ms each ===");
         summarizeThroughput("writer      ", writerOps);
         summarizeThroughput("readers total", readerOpsTotal);
      }
   }

   private static double opsPerSec(long ops)
   {
      return ops / (TRIAL_DURATION_MILLIS / 1000.0);
   }

   private static Result runTrial(long durationMillis, int readerCount) throws InterruptedException
   {
      YoRegistry registry = new YoRegistry("benchmark");
      YoDouble variable = new YoDouble("value", registry);
      YoBufferVariableEntry entry = new YoBufferVariableEntry(variable, BUFFER_SIZE);

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
               entry.haveBoundsChanged();
               entry.resetBoundsChangedFlag();
               entry.readBufferAt(index);
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
            entry.writeBufferAt(value, index);
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
