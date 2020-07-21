package us.ihmc.yoVariables.buffer;

/**
 * This class is used to store the lower and upper bounds to a series of double values typically
 * from an indexed buffer.
 * <p>
 * It is part of the {@link YoBuffer} framework.
 * </p>
 */
public class YoBufferBounds
{
   /**
    * The bounds on the data are typically computed from the buffer in the range [{@code startIndex},
    * {@code endIndex}].
    */
   private int startIndex;
   /**
    * The bounds on the data are typically computed from the buffer in the range [{@code startIndex},
    * {@code endIndex}].
    */
   private int endIndex;
   /** The minimum value in the window defined by [{@code startIndex}, {@code endIndex}]. */
   private double lowerBound;
   /** The maximum value in the window defined by [{@code startIndex}, {@code endIndex}]. */
   private double upperBound;

   /**
    * Creates a new {@code DataBounds} that is cleared.
    * 
    * @see #clear()
    */
   public YoBufferBounds()
   {
      clear();
   }

   /**
    * Clears the internal data.
    * <p>
    * The indices are set to {@code -1}, the lower bound to {@link Double#POSITIVE_INFINITY}, and the
    * upper bound to {@link Double#NEGATIVE_INFINITY}.
    * </p>
    */
   public void clear()
   {
      this.startIndex = -1;
      this.endIndex = -1;
      lowerBound = Double.POSITIVE_INFINITY;
      upperBound = Double.NEGATIVE_INFINITY;
   }

   /**
    * Sets the index window for which the bounds represent.
    * <p>
    * This method does not modify the actual bounds.
    * </p>
    * 
    * @param startIndex first index (inclusive) of the interval the bounds represent.
    * @param endIndex   last index (inclusive) of the interval the bounds represent.
    */
   public void setInterval(int startIndex, int endIndex)
   {
      this.startIndex = startIndex;
      this.endIndex = endIndex;
   }

   /**
    * Sets the bounds.
    * <p>
    * This method does not modify the indices for the interval.
    * </p>
    * 
    * @param lowerBound the minimum value.
    * @param upperBound the maximum value.
    */
   public void setBounds(double lowerBound, double upperBound)
   {
      this.lowerBound = lowerBound;
      this.upperBound = upperBound;
   }

   /**
    * Copies the values from {@code other}.
    * 
    * @param other the other bounds to copy the values from. Not modified.
    */
   public void set(YoBufferBounds other)
   {
      startIndex = other.startIndex;
      endIndex = other.endIndex;
      lowerBound = other.lowerBound;
      upperBound = other.upperBound;
   }

   /**
    * Computes and update the bounds of the given {@code buffer} within the interval
    * [{@code startIndex}, {@code endIndex}].
    * <p>
    * The interval in which the bounds are to be computed can be set via
    * {@link #setInterval(int, int)}.
    * </p>
    * 
    * @param buffer the series of values to compute the lower and upper bounds of. Not modified.
    * @return {@code true} if the bounds have changed, {@code false} otherwise.
    */
   public boolean compute(double[] buffer)
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

      boolean changed = false;

      if (newLowerBound != lowerBound || newUpperBound != upperBound)
      {
         lowerBound = newLowerBound;
         upperBound = newUpperBound;
         changed = true;
      }

      return changed;
   }

   /**
    * Updates the current bounds to contain the given {@code value}.
    * 
    * @param value the new value that is ensured to be inside the bounds after calling this method.
    * @return {@code true} if the bounds have changed, {@code false} otherwise.
    */
   public boolean update(double value)
   {
      boolean changed = false;

      if (value < lowerBound)
      {
         lowerBound = value;
         changed = true;
      }

      if (value > upperBound)
      {
         upperBound = value;
         changed = true;
      }

      return changed;
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
}
