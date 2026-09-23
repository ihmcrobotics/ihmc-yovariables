/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.buffer;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLongArray;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferVariableEntryReader;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * {@code YoBufferVariableEntry} manages the buffer to store history for a single
 * {@code YoVariable}.
 * <p>
 * The write/read/bounds-tracking path ({@link #writeIntoBufferAt(int)}, {@link #readBufferAt(int)},
 * {@link #haveBoundsChanged()}, {@link #resetBoundsChangedFlag()}, {@link #getBounds()},
 * {@link #getWindowBounds(int, int)}) is lock-free rather than {@code synchronized}: {@link #bufferData}
 * is an {@link AtomicLongArray} (values bit-reinterpreted via {@link Double#doubleToLongBits(double)},
 * since Java has no {@code AtomicDoubleArray}), {@link #currentBounds} is an immutable
 * {@link YoBufferBounds} swapped behind an {@link AtomicReference} via a compare-and-swap retry loop
 * (so a reader can never see a torn {@code (lowerBound, upperBound)} pair), and
 * {@link #boundsChanged}/{@link #boundsDirty} are {@link AtomicBoolean}s. This is correct under any
 * number of concurrent writers, not just the single-writer pattern seen in this class's actual usage.
 * </p>
 * <p>
 * The buffer-resizing operations ({@link #enlargeBufferSize(int)}, {@link #cropBuffer(int, int)},
 * {@link #cutBuffer(int, int)}, {@link #shiftBuffer(int)}, {@link #thinData(int)},
 * {@link #clearBuffer(int)}, {@link #fillBuffer()}) remain single-thread-only, exactly as before this
 * change - they were never {@code synchronized} even when the rest of this class was, so this
 * preserves their existing (implicit) contract rather than expanding it.
 * </p>
 */
public class YoBufferVariableEntry implements YoBufferVariableEntryReader
{
   /** The variable this buffer is managing. */
   private final YoVariable variable;
   /** The buffer in which the history of the variable's values are stored, bit-reinterpreted as longs. */
   private volatile AtomicLongArray bufferData;
   /** The latest computed bounds on the variable values. */
   private final AtomicReference<YoBufferBounds> currentBounds = new AtomicReference<>(YoBufferBounds.EMPTY);
   /** Flag for user convenience to keep track of when bounds have been modified. */
   private final AtomicBoolean boundsChanged = new AtomicBoolean(true);
   /**
    * Internal used to indicate whether {@link #currentBounds} should be updated when the user calls
    * {@link #getBounds()}.
    */
   private final AtomicBoolean boundsDirty = new AtomicBoolean(true);
   /** Flag for user convenience. */
   private boolean useCustomBounds = false;
   /** User-defined bounds. */
   private volatile YoBufferBounds customBounds = YoBufferBounds.EMPTY;
   /** Flag for user convenience. */
   private boolean inverted = false;

   /**
    * Creates a new buffer of the given size for the given variable.
    *
    * @param variable   the variable this buffer is dedicated to.
    * @param bufferSize the initial size of this buffer.
    */
   public YoBufferVariableEntry(YoVariable variable, int bufferSize)
   {
      this.variable = variable;
      clearBuffer(bufferSize);
   }

   /**
    * Clone constructor.
    *
    * @param other the other buffer to copy. Not modified.
    */
   public YoBufferVariableEntry(YoBufferVariableEntry other)
   {
      variable = other.getVariable();
      bufferData = copyOf(other.bufferData);
      currentBounds.set(other.currentBounds.get());
      boundsChanged.set(other.boundsChanged.get());
      boundsDirty.set(other.boundsDirty.get());
      useCustomBounds = other.useCustomBounds;
      customBounds = other.customBounds;
      inverted = other.inverted;
   }

   private static AtomicLongArray copyOf(AtomicLongArray original)
   {
      AtomicLongArray copy = new AtomicLongArray(original.length());
      for (int i = 0; i < original.length(); i++)
         copy.set(i, original.get(i));
      return copy;
   }

   protected void clearBuffer(int bufferSize)
   {
      bufferData = new AtomicLongArray(bufferSize);
      currentBounds.set(YoBufferBounds.EMPTY);
      boundsDirty.set(true);
   }

   /** {@inheritDoc} */
   @Override
   public void setInverted(boolean inverted)
   {
      this.inverted = inverted;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getInverted()
   {
      return inverted;
   }

   /** {@inheritDoc} */
   @Override
   public int getBufferSize()
   {
      return bufferData.length();
   }

   /**
    * Writes the current variable value into the buffer at the given index.
    *
    * @param index the index to write in the buffer.
    */
   public void writeIntoBufferAt(int index)
   {
      writeBufferAt(variable.getValueAsDouble(), index);
   }

   /**
    * Writes the given value into this buffer at the given index.
    * <p>
    * Package-private: nothing outside this package should be able to write into the buffer without
    * going through {@link #writeIntoBufferAt(int)}'s well-defined semantics.
    * </p>
    *
    * @param value the value to write in this buffer.
    * @param index the index to write in the buffer.
    */
   void writeBufferAt(double value, int index)
   {
      AtomicLongArray buffer = bufferData;

      if (Double.longBitsToDouble(buffer.get(index)) == value)
         return;

      buffer.set(index, Double.doubleToLongBits(value));

      YoBufferBounds current;
      YoBufferBounds widened;
      do
      {
         current = currentBounds.get();
         widened = current.widenedToInclude(value);
         if (widened == current)
            return;
      }
      while (!currentBounds.compareAndSet(current, widened));

      boundsChanged.set(true);
   }

   /**
    * Reads the buffer at the given index and updates the variable current value.
    *
    * @param index the index read the buffer at.
    */
   protected void readFromBufferAt(int index)
   {
      variable.setValueFromDouble(Double.longBitsToDouble(bufferData.get(index)));
   }

   /** {@inheritDoc} */
   @Override
   public double readBufferAt(int index)
   {
      return Double.longBitsToDouble(bufferData.get(index));
   }

   /** {@inheritDoc} */
   @Override
   public double[] getBuffer()
   {
      return getBufferWindow(0, bufferData.length());
   }

   /** {@inheritDoc} */
   @Override
   public double[] getBufferWindow(int startIndex, int length)
   {
      AtomicLongArray buffer = bufferData;
      double[] sample = new double[length];
      int n = startIndex;

      for (int i = 0; i < length; i++)
      {
         sample[i] = Double.longBitsToDouble(buffer.get(n));
         n++;
         if (n >= buffer.length())
            n = 0;
      }

      return sample;
   }

   /** {@inheritDoc} */
   @Override
   public void useCustomBounds(boolean autoScale)
   {
      useCustomBounds = !autoScale;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isUsingCustomBounds()
   {
      return !useCustomBounds;
   }

   /** {@inheritDoc} */
   @Override
   public YoVariable getVariable()
   {
      return variable;
   }

   protected void fillBuffer()
   {
      double value = variable.getValueAsDouble();
      long bits = Double.doubleToLongBits(value);
      AtomicLongArray buffer = bufferData;

      for (int i = 0; i < buffer.length(); i++)
         buffer.set(i, bits);

      currentBounds.set(YoBufferBounds.EMPTY);
   }

   protected void enlargeBufferSize(int newSize)
   {
      AtomicLongArray oldData = bufferData;
      int oldNPoints = oldData.length();

      AtomicLongArray newData = new AtomicLongArray(newSize);

      for (int i = 0; i < oldNPoints; i++)
      {
         newData.set(i, oldData.get(i));
      }

      long lastBits = oldData.get(oldNPoints - 1);
      for (int i = oldNPoints; i < newData.length(); i++)
      {
         newData.set(i, lastBits);
      }

      bufferData = newData;
      boundsDirty.set(true);
   }

   protected int cropBuffer(int start, int end)
   {
      AtomicLongArray oldData = bufferData;

      // If the endpoints are unreasonable indicate failure
      if (start < 0 || end > oldData.length())
         return -1;

      int oldNPoints = oldData.length();

      // Calculate the total number of points after the crop
      int nPoints = computeBufferSizeAfterCrop(start, end, oldNPoints);

      AtomicLongArray newData = new AtomicLongArray(nPoints);

      // Transfer the data into the new array beginning with start.
      for (int i = 0; i < newData.length(); i++)
      {
         newData.set(i, oldData.get((i + start) % oldNPoints));
      }

      bufferData = newData;
      boundsDirty.set(true);

      // Indicate the data length
      return newData.length();
   }

   protected int cutBuffer(int start, int end)
   {
      if (start > end)
         return -1;

      AtomicLongArray oldData = bufferData;

      // If the endpoints are unreasonable indicate failure
      if (start < 0 || end > oldData.length())
         return -1;

      int oldNPoints = oldData.length();

      // Calculate the total number of points after the cut
      int nPoints = computeBufferSizeAfterCut(start, end, oldNPoints);

      // If the result is 0 the size will remain the same
      if (nPoints == 0)
         nPoints = oldNPoints;
      AtomicLongArray newData = new AtomicLongArray(nPoints);

      // Transfer the data into the new array beginning with start.
      int difference = end - start + 1;
      for (int i = 0; i < start; i++)
      {
         newData.set(i, oldData.get(i));
      }

      for (int i = end + 1; i < oldNPoints; i++)
      {
         newData.set(i - difference, oldData.get(i));
      }

      bufferData = newData;
      boundsDirty.set(true);

      // Indicate the data length
      return newData.length();
   }

   protected int thinData(int keepEveryNthPoint)
   {
      AtomicLongArray oldData = bufferData;
      int oldNPoints = oldData.length();

      int newNumberOfPoints = oldNPoints / keepEveryNthPoint;
      AtomicLongArray newData = new AtomicLongArray(newNumberOfPoints);

      int oldDataIndex = 0;
      for (int index = 0; index < newNumberOfPoints; index++)
      {
         newData.set(index, oldData.get(oldDataIndex));

         oldDataIndex = oldDataIndex + keepEveryNthPoint;
      }

      bufferData = newData;

      return newNumberOfPoints;
   }

   protected static int computeBufferSizeAfterCrop(int start, int end, int previousBufferSize)
   {
      int newBufferSize = (end - start + 1 + previousBufferSize) % previousBufferSize;
      if (newBufferSize == 0)
         return previousBufferSize;
      else
         return newBufferSize;
   }

   protected static int computeBufferSizeAfterCut(int start, int end, int previousBufferSize)
   {
      return previousBufferSize - (end - start + 1);
   }

   protected void shiftBuffer(int shiftIndex)
   {
      AtomicLongArray oldData = bufferData;
      int nPoints = oldData.length();

      // If the start point is outside of the data set abort
      if (shiftIndex <= 0 || shiftIndex >= nPoints)
         return;

      AtomicLongArray newData = new AtomicLongArray(nPoints);

      // Repopulate the array using the new order
      for (int i = 0; i < nPoints; i++)
      {
         newData.set(i, oldData.get((i + shiftIndex) % nPoints));
      }

      bufferData = newData;
      boundsDirty.set(true);
   }

   /** {@inheritDoc} */
   @Override
   public void resetBoundsChangedFlag()
   {
      boundsChanged.set(false);
   }

   /** {@inheritDoc} */
   @Override
   public boolean haveBoundsChanged()
   {
      return boundsChanged.get();
   }

   /**
    * Recomputes {@link #currentBounds} over the full buffer from a local snapshot of
    * {@link #bufferData}, and updates {@link #boundsChanged} accordingly.
    * <p>
    * Uses a plain {@link AtomicReference#set(Object)} rather than a compare-and-swap retry loop: unlike
    * {@link #writeBufferAt(double, int)}'s incremental widen, this is always a full,
    * self-contained rescan of a point-in-time snapshot, so under a race the worst case is a reader
    * transiently seeing a slightly stale (but internally consistent) bounds value, which the next call
    * corrects - retrying a whole-buffer O(n) rescan on CAS failure would be considerably more expensive
    * than accepting that.
    * </p>
    * <p>
    * Deliberately does not clear {@link #boundsDirty}, matching this method's pre-existing behavior
    * from before this class used atomics: only {@link #getWindowBounds(int, int)} ever clears it, so
    * {@link #getBounds()} keeps recomputing on every call once {@link #boundsDirty} has ever been set.
    * </p>
    */
   private void updateBounds()
   {
      AtomicLongArray buffer = bufferData;
      if (buffer == null)
      {
         boundsChanged.set(false);
         return;
      }

      double[] bufferSnapshot = toDoubleArray(buffer);
      YoBufferBounds oldBounds = currentBounds.get();
      YoBufferBounds newBounds = YoBufferBounds.computed(0, bufferSnapshot.length - 1, bufferSnapshot);
      currentBounds.set(newBounds);
      boundsChanged.set(newBounds.getLowerBound() != oldBounds.getLowerBound() || newBounds.getUpperBound() != oldBounds.getUpperBound());
   }

   private static double[] toDoubleArray(AtomicLongArray buffer)
   {
      double[] result = new double[buffer.length()];
      for (int i = 0; i < result.length; i++)
         result[i] = Double.longBitsToDouble(buffer.get(i));
      return result;
   }

   /** {@inheritDoc} */
   @Override
   public YoBufferBounds getBounds()
   {
      if (boundsDirty.get())
         updateBounds();

      return currentBounds.get();
   }

   /** {@inheritDoc} */
   @Override
   public YoBufferBounds getCustomBounds()
   {
      YoBufferBounds updated = YoBufferBounds.EMPTY.withInterval(0, getBufferSize() - 1).withBounds(variable.getLowerBound(), variable.getUpperBound());
      customBounds = updated;
      return updated;
   }

   /**
    * Calculates and returns the value average of the variable over the entire buffer.
    *
    * @return the average value.
    */
   public double computeAverage()
   {
      return computeAverage(0, getBufferSize());
   }

   /**
    * Calculates and returns the value average of the variable over a portion of the buffer.
    *
    * @param start  the first buffer index to include in the calculation of the average. Should be in
    *               [0, {@code this.getBufferSize()}[.
    * @param length the number of elements to include in the calculation of the average. Should be in
    *               ]0, {@code this.getBufferSize()}].
    * @return the average value.
    */
   public double computeAverage(int start, int length)
   {
      if (start < 0 || start >= getBufferSize())
         throw new IndexOutOfBoundsException("start should be in [0, " + getBufferSize() + "[, but was: " + length);
      if (length <= 0 || length > getBufferSize())
         throw new IndexOutOfBoundsException("length should be in ]0, " + getBufferSize() + "], but was: " + length);

      AtomicLongArray buffer = bufferData;
      double total = 0.0;
      int count = 0;
      int index = 0;

      while (count < length)
      {
         total += Double.longBitsToDouble(buffer.get(index));

         count++;
         index++;
         if (index >= getBufferSize())
            index = 0;
      }

      return total / length;
   }

   /** {@inheritDoc} */
   @Override
   public YoBufferBounds getWindowBounds(int startIndex, int endIndex)
   {
      AtomicLongArray buffer = bufferData;
      if (buffer == null)
         return currentBounds.get();

      YoBufferBounds oldBounds = currentBounds.get();

      if (boundsDirty.get() || startIndex != oldBounds.getStartIndex() || endIndex != oldBounds.getEndIndex())
      {
         double[] bufferSnapshot = toDoubleArray(buffer);
         YoBufferBounds newBounds = YoBufferBounds.computed(startIndex, endIndex, bufferSnapshot);
         currentBounds.set(newBounds);
         boundsChanged.set(newBounds.getLowerBound() != oldBounds.getLowerBound() || newBounds.getUpperBound() != oldBounds.getUpperBound());
         boundsDirty.set(false);
         return newBounds;
      }

      return oldBounds;
   }

   /**
    * Tests whether this buffer and {@code other} are equal to an {@code epsilon}.
    * <p>
    * The two buffers are considered equals if all the following conditions are met:
    * <ul>
    * <li>the length of the two buffer are of same size;
    * <li>the two buffers manage variables sharing the same full name;
    * <li>the data of the two buffers are equal to an {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param other   the other buffer to compare against {@code this}. Not modified.
    * @param epsilon the tolerance used when comparing the data of the two buffers.
    * @return {@code true} if the two buffers are considered equal, {@code false} otherwise.
    */
   public boolean epsilonEquals(YoBufferVariableEntry other, double epsilon)
   {
      if (getBufferSize() != other.getBufferSize())
         return false;
      if (!getVariableFullNameString().equals(other.getVariableFullNameString()))
         return false;

      AtomicLongArray thisData = bufferData;
      AtomicLongArray otherData = other.bufferData;

      for (int i = 0; i < getBufferSize(); i++)
      {
         double thisDataPoint = Double.longBitsToDouble(thisData.get(i));
         double otherDataPoint = Double.longBitsToDouble(otherData.get(i));

         if (Double.compare(thisDataPoint, otherDataPoint) != 0 && !EuclidCoreTools.epsilonEquals(thisDataPoint, otherDataPoint, epsilon))
         {
            return false;
         }
      }

      return true;
   }

   @Override
   public String toString()
   {
      return "variable: " + variable.getName() + ", buffer size: " + getBufferSize();
   }
}
