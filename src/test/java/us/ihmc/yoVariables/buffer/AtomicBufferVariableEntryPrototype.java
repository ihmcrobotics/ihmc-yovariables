package us.ihmc.yoVariables.buffer;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLongArray;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Lock-free prototype of the concurrency-relevant subset of {@link YoBufferVariableEntry}'s
 * behavior: writing a value into an index of the buffer, tracking the min/max bounds seen, and the
 * one-shot "have bounds changed" flag.
 * <p>
 * This exists only to benchmark against {@code YoBufferVariableEntry}'s {@code synchronized}
 * implementation - see {@link SynchronizedVsAtomicBenchmark}. It is not a full replacement: no
 * backing {@code YoVariable}, no buffer-resizing/cropping/shifting operations, nothing beyond what's
 * needed to compare the two concurrency strategies on equal footing.
 * </p>
 * <p>
 * Correctness is meant to be unconditional, i.e. equivalent to {@code synchronized} under any number
 * of concurrent writers, not just the single-writer pattern observed in real usage:
 * </p>
 * <ul>
 * <li>{@code bufferData} is an {@link AtomicLongArray} with values bit-reinterpreted via
 * {@link Double#doubleToLongBits(double)}/{@link Double#longBitsToDouble(long)} (the same trick
 * {@code YoVariable.getValueAsLongBits()} uses elsewhere in this codebase), since Java has no
 * {@code AtomicDoubleArray}. Per-slot reads/writes are individually atomic and torn-free.
 * <li>{@code currentBounds} is an immutable {@link Bounds} pair behind a single
 * {@link AtomicReference}, updated via a compare-and-swap retry loop. This is deliberate: making
 * {@code lower}/{@code upper} two separate atomic fields would let a reader observe a torn pair
 * (new lower, stale upper); swapping one immutable object atomically cannot produce that.
 * <li>{@code boundsChanged} is a plain {@link AtomicBoolean}.
 * </ul>
 */
class AtomicBufferVariableEntryPrototype
{
   private final AtomicLongArray bufferData;
   private final AtomicReference<Bounds> currentBounds = new AtomicReference<>(Bounds.EMPTY);
   private final AtomicBoolean boundsChanged = new AtomicBoolean(false);

   AtomicBufferVariableEntryPrototype(int bufferSize)
   {
      bufferData = new AtomicLongArray(bufferSize);
   }

   /**
    * Lock-free equivalent of {@code YoBufferVariableEntry.writeBufferAt(double, int)}: writes the
    * value into the buffer at the given index and widens the tracked bounds if needed.
    */
   void writeValueAt(double value, int index)
   {
      double previous = Double.longBitsToDouble(bufferData.get(index));
      if (previous == value)
         return;

      bufferData.set(index, Double.doubleToLongBits(value));

      Bounds current;
      Bounds widened;
      do
      {
         current = currentBounds.get();
         widened = current.widenedToInclude(value);
         if (widened == current)
            return; // Already inside bounds - no update, no flag change, matches YoBufferBounds.update().
      }
      while (!currentBounds.compareAndSet(current, widened));

      boundsChanged.set(true);
   }

   /** Lock-free equivalent of {@code YoBufferVariableEntry.readBufferAt(int)}. */
   double readValueAt(int index)
   {
      return Double.longBitsToDouble(bufferData.get(index));
   }

   /** Lock-free equivalent of {@code YoBufferVariableEntry.resetBoundsChangedFlag()}. */
   void resetBoundsChangedFlag()
   {
      boundsChanged.set(false);
   }

   /** Lock-free equivalent of {@code YoBufferVariableEntry.haveBoundsChanged()}. */
   boolean haveBoundsChanged()
   {
      return boundsChanged.get();
   }

   /** Lock-free equivalent of {@code YoBufferVariableEntry.getBounds()} (the already-computed case). */
   Bounds getBounds()
   {
      return currentBounds.get();
   }

   /**
    * Immutable lower/upper bound pair. The whole point of immutability here is that
    * {@link AtomicReference#compareAndSet} can swap the pair as one atomic unit - a reader's
    * {@link #getBounds()} always sees a self-consistent {@code (lower, upper)}, never a torn one.
    */
   static final class Bounds
   {
      static final Bounds EMPTY = new Bounds(Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);

      final double lower;
      final double upper;

      private Bounds(double lower, double upper)
      {
         this.lower = lower;
         this.upper = upper;
      }

      Bounds widenedToInclude(double value)
      {
         if (value >= lower && value <= upper)
            return this;
         return new Bounds(Math.min(lower, value), Math.max(upper, value));
      }
   }
}
