package us.ihmc.yoVariables.buffer.interfaces;

import us.ihmc.yoVariables.buffer.YoBufferBounds;
import us.ihmc.yoVariables.buffer.YoBufferVariableEntry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Base interface for reading from a {@link YoBufferVariableEntry}.
 * <p>
 * A {@link YoBufferVariableEntryReader} manages the buffer for a single {@code YoVariable}.
 * </p>
 */
public interface YoBufferVariableEntryReader
{
   /**
    * The variable this buffer is for.
    * 
    * @return this buffer's variable.
    */
   YoVariable getVariable();

   /**
    * Convenience method for getting this buffer's variable name.
    * 
    * @return the name of this buffer's variable.
    * @see YoVariable#getName()
    */
   default String getVariableName()
   {
      return getVariable().getName();
   }

   /**
    * Convenience method for getting this buffer's variable full name.
    * 
    * @return the full name of this buffer's variable.
    * @see YoVariable#getFullNameString()
    */
   default String getVariableFullNameString()
   {
      return getVariable().getFullNameString();
   }

   /**
    * Returns the current size of this buffer.
    * 
    * @return the buffer size.
    */
   int getBufferSize();

   /**
    * Reads and returns the buffer value at the given index.
    * 
    * @param index the index to read the buffer at.
    * @return the value stored in the buffer at the given index.
    */
   double readBufferAt(int index);

   /**
    * Returns a copy of this entire buffer.
    * 
    * @return copy of this buffer.
    */
   double[] getBuffer();

   /**
    * Returns a sample of this buffer.
    * 
    * @param startIndex the first index to be stored in the returned sample.
    * @param length     the sample length.
    * @return the sample.
    */
   double[] getBufferWindow(int startIndex, int length);

   /**
    * Marks an internal flag, next call to {@link #haveBoundsChanged()} is guaranteed to return
    * {@code false}.
    */
   void resetBoundsChangedFlag();

   /**
    * Returns whether the data bounds have changed.
    * <p>
    * The data bounds can be obtained via {@link #getBounds()} and are updated when the buffer data has
    * been changed. When {@code true}, the flag indicating the bounds have changed can only be manually
    * reset via {@link #resetBoundsChangedFlag()}.
    * </p>
    * 
    * @return {@code true} if the bounds have been updated, {@code false} otherwise.
    */
   boolean haveBoundsChanged();

   /**
    * Returns the current bounds for the data in this buffer.
    * <p>
    * This method will trigger an update of the bounds if necessary.
    * </p>
    * <p>
    * The data bounds can be used to get the minimum and maximum values current stored in this buffer.
    * </p>
    * 
    * @return the current bounds for the entire buffer.
    */
   YoBufferBounds getBounds();

   /**
    * Returns the current lower bound for the data in this buffer.
    * <p>
    * This method will trigger an update of the bounds if necessary.
    * </p>
    * 
    * @return the current lower bound for the entire buffer, i.e. the minimum value contained in this
    *         buffer.
    */
   default double getLowerBound()
   {
      return getBounds().getLowerBound();
   }

   /**
    * Returns the current upper bound for the data in this buffer.
    * <p>
    * This method will trigger an update of the bounds if necessary.
    * </p>
    * 
    * @return the current upper bound for the entire buffer, i.e. the maximum value contained in this
    *         buffer.
    */
   default double getUpperBound()
   {
      return getBounds().getUpperBound();
   }

   /**
    * Computes and returns the bounds on the buffer values for the index interval [{@code startIndex},
    * {@code endIndex}].
    * 
    * @param startIndex first index (inclusive) of the interval the bounds are to be computed.
    * @param endIndex   last index (inclusive) of the interval the bounds are to be computed.
    * @return the bounds on the buffer values for the index interval [{@code startIndex},
    *         {@code endIndex}].
    */
   YoBufferBounds getWindowBounds(int startIndex, int endIndex);

   /**
    * Computes and returns the lower bound on the buffer values for the index interval
    * [{@code startIndex}, {@code endIndex}].
    * 
    * @param startIndex first index (inclusive) of the interval the lower is to be computed.
    * @param endIndex   last index (inclusive) of the interval the lower is to be computed.
    * @return the lower bound on the buffer values for the index interval [{@code startIndex},
    *         {@code endIndex}].
    */
   default double getWindowLowerBound(int startIndex, int endIndex)
   {
      return getWindowBounds(startIndex, endIndex).getLowerBound();
   }

   /**
    * Computes and returns the upper bound on the buffer values for the index interval
    * [{@code startIndex}, {@code endIndex}].
    * 
    * @param startIndex first index (inclusive) of the interval the upper is to be computed.
    * @param endIndex   last index (inclusive) of the interval the upper is to be computed.
    * @return the upper bound on the buffer values for the index interval [{@code startIndex},
    *         {@code endIndex}].
    */
   default double getWindowUpperBound(int startIndex, int endIndex)
   {
      return getWindowBounds(startIndex, endIndex).getUpperBound();
   }

   /**
    * Sets the internal flag "useCustomBounds" associated to this entry.
    * <p>
    * The flag is meant for convenience and does not affect the rest of this entry's internal state.
    * </p>
    * 
    * @param useCustomBounds the new value for the flag.
    */
   void useCustomBounds(boolean useCustomBounds);

   /**
    * Returns the current value for the flag "useCustomBounds" associated to this entry.
    * <p>
    * The flag is meant for convenience and does not affect the rest of this entry's internal state.
    * </p>
    * 
    * @return the flag's current value.
    */
   boolean isUsingCustomBounds();

   /**
    * Sets custom bounds to be associated with this entry.
    * <p>
    * Custom bounds do not affect the rest of this entry's internal state.
    * </p>
    * <p>
    * The custom bounds are saved as the variable's bounds.
    * </p>
    * 
    * @param customLowerBound the value for the custom lower bound.
    * @param customUpperBound the value for the custom upper bound.
    * @see YoVariable#setVariableBounds(double, double)
    */
   default void setCustomBounds(double customLowerBound, double customUpperBound)
   {
      getVariable().setVariableBounds(customLowerBound, customUpperBound);
   }

   /**
    * Gets the current custom bounds associated with this entry.
    * 
    * @return the custom bounds.
    */
   YoBufferBounds getCustomBounds();

   /**
    * Gets the current custom lower bound associated with this entry.
    * 
    * @return the custom lower bound.
    */
   default double getCustomLowerBound()
   {
      return getCustomBounds().getLowerBound();
   }

   /**
    * Gets the current custom upper bound associated with this entry.
    * 
    * @return the custom upper bound.
    */
   default double getCustomUpperBound()
   {
      return getCustomBounds().getUpperBound();
   }

   /**
    * Sets the internal flag "inverted" associated to this entry.
    * <p>
    * The flag is meant for convenience and does not affect the rest of this entry's internal state.
    * </p>
    * 
    * @param inverted the new value for the flag.
    */
   void setInverted(boolean inverted);

   /**
    * Returns the current value for the flag "inverted" associated to this entry.
    * <p>
    * The flag is meant for convenience and does not affect the rest of this entry's internal state.
    * </p>
    * 
    * @return the flag's current value.
    */
   boolean getInverted();
}
