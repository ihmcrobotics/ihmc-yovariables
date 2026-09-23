package us.ihmc.yoVariables.buffer;

import java.util.Arrays;
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
 * once on the branch/commit before the change and once after, and compare the two "FINAL SUMMARY"
 * tables. Keeping this in source means that if the strategy is ever changed again (including
 * reverted), the same benchmark immediately shows what changed, without needing to keep a synthetic
 * alternative implementation around forever just to have something to diff against.
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
 * directly. Pass {@code --verbose} to also print every individual trial.
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

   /** Coefficient of variation (stddev as % of mean) at or below which a measurement is called stable. */
   private static final double STABLE_CV_PERCENT = 5.0;
   /** Coefficient of variation above which a measurement is called noisy. */
   private static final double NOISY_CV_PERCENT = 15.0;

   public static void main(String[] args) throws InterruptedException
   {
      boolean verbose = args.length > 0 && args[0].equals("--verbose");

      printIntroduction();

      Summary[] summaries = new Summary[READER_COUNTS.length];

      for (int i = 0; i < READER_COUNTS.length; i++)
      {
         int readerCount = READER_COUNTS[i];

         System.out.println();
         System.out.println("==================== Scenario " + (i + 1) + " of " + READER_COUNTS.length + ": 1 writer + " + readerCount + " "
               + plural(readerCount, "reader") + " ====================");
         System.out.printf("Warming up for %.0f s (lets the JIT compile the hot code; not measured)...%n", WARMUP_DURATION_MILLIS / 1000.0);
         runTrial(WARMUP_DURATION_MILLIS, readerCount);

         double[] writerOpsPerSec = new double[TRIALS];
         double[] perReaderOpsPerSec = new double[TRIALS];
         double[] readersTotalOpsPerSec = new double[TRIALS];

         System.out.printf("Running %d trials of %.0f s each", TRIALS, TRIAL_DURATION_MILLIS / 1000.0);
         System.out.println(verbose ? ":" : " (pass --verbose to see each trial)...");

         for (int trial = 0; trial < TRIALS; trial++)
         {
            Result result = runTrial(TRIAL_DURATION_MILLIS, readerCount);

            writerOpsPerSec[trial] = result.writerOpsPerSec();
            readersTotalOpsPerSec[trial] = result.totalReaderOpsPerSec();
            perReaderOpsPerSec[trial] = readersTotalOpsPerSec[trial] / readerCount;

            if (verbose)
            {
               System.out.printf("  Trial %2d: writer %7.2f M ops/s | per reader %7.2f M ops/s | all readers %7.2f M ops/s%n",
                                 trial + 1,
                                 writerOpsPerSec[trial] / 1e6,
                                 perReaderOpsPerSec[trial] / 1e6,
                                 readersTotalOpsPerSec[trial] / 1e6);
            }
         }

         Summary summary = new Summary(readerCount,
                                       new Stats(writerOpsPerSec),
                                       new Stats(perReaderOpsPerSec),
                                       new Stats(readersTotalOpsPerSec));
         summaries[i] = summary;

         System.out.println();
         System.out.println("Results (millions of operations per second, higher is better):");
         System.out.println("                   median       mean  +/- stddev  noise (CV)                  min       max");
         printStatsRow("  writer       ", summary.writer);
         printStatsRow("  per reader   ", summary.perReader);
         if (readerCount > 1)
            printStatsRow("  all readers  ", summary.readersTotal);
      }

      printFinalSummary(summaries);
   }

   private static void printIntroduction()
   {
      System.out.println("YoBufferVariableEntry concurrency benchmark");
      System.out.println("-------------------------------------------");
      System.out.println("One writer thread and one or more reader threads use the same YoBufferVariableEntry at the");
      System.out.println("same time, the way a sim/timer thread and a render thread do. Each thread counts how many");
      System.out.println("operations it completes, and the counts are converted to rates.");
      System.out.println();
      System.out.println("  writer op = one writeBufferAt(value, index) call");
      System.out.println("  reader op = one haveBoundsChanged() + resetBoundsChangedFlag() + readBufferAt(index) cycle");
      System.out.println();
      System.out.println("How to read the results:");
      System.out.println("  * All rates are in millions of operations per second (M ops/s). Higher is better.");
      System.out.println("  * 'median' is the typical trial. If it is far from 'mean', a few unusually fast or slow");
      System.out.println("    trials are pulling the mean around; trust the median more.");
      System.out.println("  * 'noise (CV)' is the stddev as a percentage of the mean:");
      System.out.printf("      <= %.0f%% stable, %.0f-%.0f%% some noise, > %.0f%% noisy (close other programs and rerun).%n",
                        STABLE_CV_PERCENT,
                        STABLE_CV_PERCENT,
                        NOISY_CV_PERCENT,
                        NOISY_CV_PERCENT);
      System.out.println("  * Absolute numbers depend on the machine. Only compare runs made on the same machine.");
      System.out.println("  * To judge a code change, run this once before and once after, then compare the");
      System.out.println("    FINAL SUMMARY tables. Differences smaller than about 2x the stddev are just noise.");
   }

   private static void printStatsRow(String label, Stats stats)
   {
      System.out.printf("%s %9.2f  %9.2f  +/- %7.2f   %5.1f%% %-12s %9.2f %9.2f%n",
                        label,
                        stats.median / 1e6,
                        stats.mean / 1e6,
                        stats.stddev / 1e6,
                        stats.coefficientOfVariationPercent(),
                        "(" + stats.noiseLabel() + ")",
                        stats.min / 1e6,
                        stats.max / 1e6);
   }

   private static void printFinalSummary(Summary[] summaries)
   {
      System.out.println();
      System.out.println("==================================== FINAL SUMMARY ====================================");
      System.out.println("Mean throughput in millions of operations per second (higher is better), +/- stddev.");
      System.out.println();
      System.out.println("  readers |       writer        |      per reader     |     all readers");
      System.out.println("  --------+---------------------+---------------------+---------------------");
      for (Summary summary : summaries)
      {
         System.out.printf("  %7d | %8.2f +/- %6.2f | %8.2f +/- %6.2f | %8.2f +/- %6.2f%n",
                           summary.readerCount,
                           summary.writer.mean / 1e6,
                           summary.writer.stddev / 1e6,
                           summary.perReader.mean / 1e6,
                           summary.perReader.stddev / 1e6,
                           summary.readersTotal.mean / 1e6,
                           summary.readersTotal.stddev / 1e6);
      }

      System.out.println();
      System.out.println("What this means:");

      Summary first = summaries[0];
      for (int i = 1; i < summaries.length; i++)
      {
         Summary other = summaries[i];
         System.out.printf("  * Going from %d to %d %s, the writer %s.%n",
                           first.readerCount,
                           other.readerCount,
                           plural(other.readerCount, "reader"),
                           describeChange(first.writer, other.writer));
         System.out.printf("  * Going from %d to %d %s, each reader individually %s.%n",
                           first.readerCount,
                           other.readerCount,
                           plural(other.readerCount, "reader"),
                           describeChange(first.perReader, other.perReader));
      }

      boolean anyNoisy = false;
      for (Summary summary : summaries)
         anyNoisy |= summary.writer.isNoisy() || summary.perReader.isNoisy() || summary.readersTotal.isNoisy();
      if (anyNoisy)
         System.out.println("  * WARNING: some measurements were noisy. Close other programs and rerun before trusting small differences.");
      else
         System.out.println("  * All measurements had acceptable noise.");
      System.out.println("=======================================================================================");
   }

   /**
    * Describes how a throughput changed from {@code before} to {@code after}, treating changes smaller
    * than twice the combined stddev as no real change.
    */
   private static String describeChange(Stats before, Stats after)
   {
      double percentChange = (after.mean - before.mean) / before.mean * 100.0;
      double combinedStddev = Math.sqrt(before.stddev * before.stddev + after.stddev * after.stddev);

      if (Math.abs(after.mean - before.mean) < 2.0 * combinedStddev)
      {
         if (before.isNoisy() || after.isNoisy())
            return String.format("changed by %+.1f%%, but the runs were too noisy to tell whether that is real", percentChange);
         return String.format("stayed about the same (%+.1f%%, within noise)", percentChange);
      }
      else if (percentChange < 0.0)
         return String.format("slowed down by %.1f%% (%.2f -> %.2f M ops/s)", -percentChange, before.mean / 1e6, after.mean / 1e6);
      else
         return String.format("sped up by %.1f%% (%.2f -> %.2f M ops/s)", percentChange, before.mean / 1e6, after.mean / 1e6);
   }

   private static String plural(int count, String word)
   {
      return count == 1 ? word : word + "s";
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

      // Measure the actual window rather than trusting Thread.sleep to be exact.
      long startNanos = System.nanoTime();
      start.countDown();
      Thread.sleep(durationMillis);
      stop.set(true);
      long elapsedNanos = System.nanoTime() - startNanos;

      writer.join();
      for (Thread reader : readers)
         reader.join();

      long[] perReaderOps = new long[readerCount];
      for (int i = 0; i < readerCount; i++)
         perReaderOps[i] = readerOpsCompleted[i].get();

      return new Result(writerOpsCompleted.get(), perReaderOps, elapsedNanos / 1e9);
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

   private static final class Stats
   {
      private final double mean;
      private final double median;
      private final double stddev;
      private final double min;
      private final double max;

      private Stats(double[] values)
      {
         double sum = 0.0;
         double min = Double.POSITIVE_INFINITY;
         double max = Double.NEGATIVE_INFINITY;
         for (double value : values)
         {
            sum += value;
            min = Math.min(min, value);
            max = Math.max(max, value);
         }
         this.mean = sum / values.length;
         this.min = min;

         double[] sorted = values.clone();
         Arrays.sort(sorted);
         int middle = sorted.length / 2;
         this.median = sorted.length % 2 == 1 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) / 2.0;
         this.max = max;

         // Sample standard deviation (n-1 denominator).
         if (values.length < 2)
         {
            this.stddev = 0.0;
         }
         else
         {
            double sumOfSquaredDeviations = 0.0;
            for (double value : values)
            {
               double deviation = value - mean;
               sumOfSquaredDeviations += deviation * deviation;
            }
            this.stddev = Math.sqrt(sumOfSquaredDeviations / (values.length - 1));
         }
      }

      private double coefficientOfVariationPercent()
      {
         return mean == 0.0 ? 0.0 : stddev / mean * 100.0;
      }

      private boolean isNoisy()
      {
         return coefficientOfVariationPercent() > NOISY_CV_PERCENT;
      }

      private String noiseLabel()
      {
         double cv = coefficientOfVariationPercent();
         if (cv <= STABLE_CV_PERCENT)
            return "stable";
         else if (cv <= NOISY_CV_PERCENT)
            return "some noise";
         else
            return "NOISY";
      }
   }

   private static final class Summary
   {
      private final int readerCount;
      private final Stats writer;
      private final Stats perReader;
      private final Stats readersTotal;

      private Summary(int readerCount, Stats writer, Stats perReader, Stats readersTotal)
      {
         this.readerCount = readerCount;
         this.writer = writer;
         this.perReader = perReader;
         this.readersTotal = readersTotal;
      }
   }

   private static final class Result
   {
      private final long writerOpsCompleted;
      private final long[] perReaderOpsCompleted;
      private final double elapsedSeconds;

      private Result(long writerOpsCompleted, long[] perReaderOpsCompleted, double elapsedSeconds)
      {
         this.writerOpsCompleted = writerOpsCompleted;
         this.perReaderOpsCompleted = perReaderOpsCompleted;
         this.elapsedSeconds = elapsedSeconds;
      }

      private double writerOpsPerSec()
      {
         return writerOpsCompleted / elapsedSeconds;
      }

      private double totalReaderOpsPerSec()
      {
         long total = 0;
         for (long ops : perReaderOpsCompleted)
            total += ops;
         return total / elapsedSeconds;
      }
   }
}
