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
package us.ihmc.yoVariables.buffer.interfaces;

import us.ihmc.yoVariables.buffer.YoBuffer;

/**
 * Base interface for reading from a {@link YoBuffer}.
 * <p>
 * A {@link YoBufferReader} manages the buffers for a collection of {@code YoVariable}s.
 * </p>
 */
public interface YoBufferReader
{
   /**
    * Returns the index of the current buffer's in-point.
    * <p>
    * The buffer has a sub-interval to highlight part of interest in the buffer, or the part were
    * actual data was written. This sub-interval is defined by [{@code inPoint}, {@code outPoint}].
    * When filled up, the buffer automatically wraps the reading/writing index, such that it is
    * possible that {@code inPoint > outPoint} indicating that the sub-interval starts towards the end
    * of the buffer to end at the beginning.
    * </p>
    * 
    * @return the in-point current index value.
    */
   int getInPoint();

   /**
    * Returns the index of the current buffer's out-point.
    * <p>
    * The buffer has a sub-interval to highlight part of interest in the buffer, or the part were
    * actual data was written. This sub-interval is defined by [{@code inPoint}, {@code outPoint}].
    * When filled up, the buffer automatically wraps the reading/writing index, such that it is
    * possible that {@code inPoint > outPoint} indicating that the sub-interval starts towards the end
    * of the buffer to end at the beginning.
    * </p>
    * 
    * @return the out-point current index value.
    */
   int getOutPoint();

   /**
    * Returns the value of the current index.
    * <p>
    * The current index represents the current reading/writing position in the buffer.
    * </p>
    * 
    * @return the current index value.
    */
   int getCurrentIndex();

   /**
    * Returns the current buffer size.
    * 
    * @return the buffer size.
    */
   int getBufferSize();

   /**
    * Sets the current buffer index and reads the buffer at the new index and updates the variables'
    * value.
    * 
    * @param index the new index value.
    */
   void setCurrentIndex(int index);

   /**
    * Increments/decrements the current buffer index by the given step size and reads the buffer at the
    * new index and updates the variables' value.
    * 
    * @param stepSize the size of the increment (if positive) or decrement (if negative).
    * @return {@code true} if the current index rolled over to the beginning of the buffer,
    *         {@code false} otherwise.
    */
   boolean tickAndReadFromBuffer(int stepSize);

   /**
    * Computes and returns the length of [{@code inPoint}, {@code outPoint}].
    * 
    * @return the length of the sub-interval [{@code inPoint}, {@code outPoint}].
    */
   default int getBufferInOutLength()
   {
      if (getOutPoint() > getInPoint())
         return getOutPoint() - getInPoint() + 1;
      else
         return getBufferSize() - (getInPoint() - getOutPoint()) + 1;
   }

   /**
    * Tests whether the given index is located in the sub-interval [{@code inPoint}, {@code outPoint}].
    * <p>
    * This method accounts for the possibility of the sub-interval rolling over the end of the buffer,
    * i.e. {@code inPoint > outPoint}.
    * </p>
    * 
    * @param indexToCheck the query.
    * @return {@code true} if the query is located in the sub-interval, {@code false} otherwise.
    */
   default boolean isIndexBetweenBounds(int indexToCheck)
   {
      if (indexToCheck < 0 || indexToCheck >= getBufferSize())
         return false;
      else if (getInPoint() <= getOutPoint())
         return indexToCheck >= getInPoint() && indexToCheck <= getOutPoint();
      else
         return indexToCheck <= getOutPoint() || indexToCheck > getInPoint();
   }
}
