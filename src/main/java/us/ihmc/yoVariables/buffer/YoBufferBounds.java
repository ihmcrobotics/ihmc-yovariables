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

import java.util.Objects;

/**
 * Immutable value type storing the lower and upper bounds to a series of double values typically
 * from an indexed buffer.
 * <p>
 * It is part of the {@link YoBuffer} framework.
 * </p>
 * <p>
 * Immutable specifically so {@link YoBufferVariableEntry} can hold it behind a
 * {@link java.util.concurrent.atomic.AtomicReference} and swap one instance for another as a single
 * atomic operation - a reader can then never observe a torn {@code (lowerBound, upperBound)} pair
 * while a writer thread is concurrently widening the bounds, which two separate mutable fields could
 * not guarantee.
 * </p>
 */
public class YoBufferBounds
{
   /**
    * A cleared instance: indices {@code -1}, lower bound {@link Double#POSITIVE_INFINITY}, upper bound
    * {@link Double#NEGATIVE_INFINITY}.
    */
   public static final YoBufferBounds EMPTY = new YoBufferBounds(-1, -1, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);

   /**
    * The bounds on the data are typically computed from the buffer in the range [{@code startIndex},
    * {@code endIndex}].
    */
   private final int startIndex;
   /**
    * The bounds on the data are typically computed from the buffer in the range [{@code startIndex},
    * {@code endIndex}].
    */
   private final int endIndex;
   /** The minimum value in the window defined by [{@code startIndex}, {@code endIndex}]. */
   private final double lowerBound;
   /** The maximum value in the window defined by [{@code startIndex}, {@code endIndex}]. */
   private final double upperBound;

   private YoBufferBounds(int startIndex, int endIndex, double lowerBound, double upperBound)
   {
      this.startIndex = startIndex;
      this.endIndex = endIndex;
      this.lowerBound = lowerBound;
      this.upperBound = upperBound;
   }

   /**
    * Returns a new bounds with the given index window, keeping this instance's lower/upper bound.
    *
    * @param startIndex first index (inclusive) of the interval the bounds represent.
    * @param endIndex   last index (inclusive) of the interval the bounds represent.
    * @return the new bounds.
    */
   public YoBufferBounds withInterval(int startIndex, int endIndex)
   {
      return new YoBufferBounds(startIndex, endIndex, lowerBound, upperBound);
   }

   /**
    * Returns a new bounds with the given lower/upper bound, keeping this instance's index window.
    *
    * @param lowerBound the minimum value.
    * @param upperBound the maximum value.
    * @return the new bounds.
    */
   public YoBufferBounds withBounds(double lowerBound, double upperBound)
   {
      return new YoBufferBounds(startIndex, endIndex, lowerBound, upperBound);
   }

   /**
    * Computes the bounds of the given {@code buffer} within the interval [{@code startIndex},
    * {@code endIndex}].
    *
    * @param startIndex first index (inclusive) of the interval to compute the bounds of.
    * @param endIndex   last index (inclusive) of the interval to compute the bounds of.
    * @param buffer     the series of values to compute the lower and upper bounds of. Not modified.
    * @return the newly computed bounds.
    */
   public static YoBufferBounds computed(int startIndex, int endIndex, double[] buffer)
   {
      double newLowerBound = Double.POSITIVE_INFINITY;
      double newUpperBound = Double.NEGATIVE_INFINITY;

      if (startIndex < endIndex)
      {
         for (int i = startIndex; i < endIndex; i++)
         {
            double value = buffer[i];
            if (value < newLowerBound)
               newLowerBound = value;

            if (value > newUpperBound)
               newUpperBound = value;
         }
      }
      else
      {
         for (int i = startIndex; i < buffer.length; i++)
         {
            double value = buffer[i];
            if (value < newLowerBound)
               newLowerBound = value;

            if (value > newUpperBound)
               newUpperBound = value;
         }

         for (int i = 0; i < endIndex; i++)
         {
            double value = buffer[i];
            if (value < newLowerBound)
               newLowerBound = value;

            if (value > newUpperBound)
               newUpperBound = value;
         }
      }

      return new YoBufferBounds(startIndex, endIndex, newLowerBound, newUpperBound);
   }

   /**
    * Returns a new bounds widened to include {@code value}, keeping this instance's index window - or
    * this same instance if {@code value} is already inside the current bounds.
    * <p>
    * Returning {@code this} unchanged (rather than an equal-valued new instance) is deliberate: it lets
    * a caller doing a compare-and-swap update detect "no change needed" via reference equality without
    * an extra value comparison.
    * </p>
    *
    * @param value the new value that is ensured to be inside the returned bounds.
    * @return the widened bounds, or {@code this} if {@code value} was already inside bounds.
    */
   public YoBufferBounds widenedToInclude(double value)
   {
      if (value >= lowerBound && value <= upperBound)
         return this;
      return new YoBufferBounds(startIndex, endIndex, Math.min(lowerBound, value), Math.max(upperBound, value));
   }

   /**
    * Tests if the given {@code value} is inside (inclusive) the current bounds.
    *
    * @param value the query.
    * @return {@code true} if <tt>value &in; [lowerBounds; upperBound]</tt>, {@code false} otherwise.
    */
   public boolean isInsideBounds(double value)
   {
      return value >= lowerBound && value <= upperBound;
   }

   /**
    * Returns the start of the interval for which the bounds were computed.
    * <p>
    * The bounds on the data are typically computed from the buffer in the range [{@code startIndex},
    * {@code endIndex}].
    * </p>
    *
    * @return first index (inclusive) of the interval the bounds represent.
    */
   public int getStartIndex()
   {
      return startIndex;
   }

   /**
    * Returns the end of the interval for which the bounds were computed.
    * <p>
    * The bounds on the data are typically computed from the buffer in the range [{@code startIndex},
    * {@code endIndex}].
    * </p>
    *
    * @return last index (inclusive) of the interval the bounds represent.
    */
   public int getEndIndex()
   {
      return endIndex;
   }

   /**
    * Returns the current value for the lower bound, i.e. the minimum value in the interval
    * [{@code startIndex}, {@code endIndex}].
    *
    * @return the value of the lower bound.
    */
   public double getLowerBound()
   {
      return lowerBound;
   }

   /**
    * Returns the current value for the upper bound, i.e. the maximum value in the interval
    * [{@code startIndex}, {@code endIndex}].
    *
    * @return the value of the upper bound.
    */
   public double getUpperBound()
   {
      return upperBound;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!(object instanceof YoBufferBounds))
         return false;
      YoBufferBounds other = (YoBufferBounds) object;
      return startIndex == other.startIndex && endIndex == other.endIndex && Double.compare(lowerBound, other.lowerBound) == 0
            && Double.compare(upperBound, other.upperBound) == 0;
   }

   @Override
   public int hashCode()
   {
      return Objects.hash(startIndex, endIndex, lowerBound, upperBound);
   }

   @Override
   public String toString()
   {
      return "[" + startIndex + ", " + endIndex + "] -> [" + lowerBound + ", " + upperBound + "]";
   }
}
