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

import java.util.Arrays;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferVariableEntryReader;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * {@code YoBufferVariableEntry} manages the buffer to store history for a single
 * {@code YoVariable}.
 */
public class YoBufferVariableEntry implements YoBufferVariableEntryReader
{
   /** The variable this buffer is managing. */
   private final YoVariable variable;
   /** The buffer in which the history of the variable's values are stored. */
   private double[] bufferData;
   /** The latest computed bounds on the variable values. */
   private final YoBufferBounds currentBounds = new YoBufferBounds();
   /** Flag for user convenience to keep track of when bounds have been modified. */
   private boolean boundsChanged = true;
   /**
    * Internal used to indicate whether {@link #currentBounds} should be updated when the user calls
    * {@link #getBounds()}.
    */
   private boolean boundsDirty = true;
   /** Flag for user convenience. */
   private boolean useCustomBounds = false;
   /** User-defined bounds. */
   private final YoBufferBounds customBounds = new YoBufferBounds();
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
      bufferData = Arrays.copyOf(other.bufferData, other.getBufferSize());
      currentBounds.set(other.currentBounds);
      boundsChanged = other.boundsChanged;
      boundsDirty = other.boundsDirty;
      useCustomBounds = other.useCustomBounds;
      customBounds.set(other.customBounds);
      inverted = other.inverted;
   }

   protected void clearBuffer(int bufferSize)
   {
      bufferData = new double[bufferSize];
      currentBounds.clear();
      boundsDirty = true;
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
      return bufferData.length;
   }

   /**
    * Writes the current variable value into the buffer at the given index.
    * 
    * @param index the index to write in the buffer.
    */
   public synchronized void writeIntoBufferAt(int index)
   {
      writeBufferAt(variable.getValueAsDouble(), index);
   }

   /**
    * Writes the given value into this buffer at the given index.
    * 
    * @param value the value to write in this buffer.
    * @param index the index to write in the buffer.
    */
   public void writeBufferAt(double value, int index)
   {
      if (bufferData[index] == value)
         return;

      bufferData[index] = value;

      if (currentBounds.update(value))
         boundsChanged = true;
   }

   /**
    * Reads the buffer at the given index and updates the variable current value.
    * 
    * @param index the index read the buffer at.
    */
   protected void readFromBufferAt(int index)
   {
      variable.setValueFromDouble(bufferData[index]);
   }

   /** {@inheritDoc} */
   @Override
   public double readBufferAt(int index)
   {
      return bufferData[index];
   }

   /** {@inheritDoc} */
   @Override
   public double[] getBuffer()
   {
      return getBufferWindow(0, bufferData.length);
   }

   /** {@inheritDoc} */
   @Override
   public double[] getBufferWindow(int startIndex, int length)
   {
      double[] sample = new double[length];
      int n = startIndex;

      for (int i = 0; i < length; i++)
      {
         sample[i] = bufferData[n];
         n++;
         if (n >= bufferData.length)
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
      for (int i = 0; i < bufferData.length; i++)
         bufferData[i] = value;
      currentBounds.clear();
   }

   protected void enlargeBufferSize(int newSize)
   {
      double[] oldData = bufferData;
      int oldNPoints = oldData.length;

      bufferData = new double[newSize];

      for (int i = 0; i < oldNPoints; i++)
      {
         bufferData[i] = oldData[i];
      }

      for (int i = oldNPoints; i < bufferData.length; i++)
      {
         bufferData[i] = oldData[oldNPoints - 1];
      }

      boundsDirty = true;
   }

   protected int cropBuffer(int start, int end)
   {
      // If the endpoints are unreasonable indicate failure
      if (start < 0 || end > bufferData.length)
         return -1;

      // Create a temporary variable to hold the old data
      double[] oldData = bufferData;
      int oldNPoints = oldData.length;

      // Calculate the total number of points after the crop
      int nPoints = computeBufferSizeAfterCrop(start, end, oldNPoints);

      bufferData = new double[nPoints];

      // Transfer the data into the new array beginning with start.
      for (int i = 0; i < bufferData.length; i++)
      {
         bufferData[i] = oldData[(i + start) % oldNPoints];
      }

      boundsDirty = true;

      // Indicate the data length
      return bufferData.length;
   }

   protected int cutBuffer(int start, int end)
   {
      if (start > end)
         return -1;

      // If the endpoints are unreasonable indicate failure
      if (start < 0 || end > bufferData.length)
         return -1;

      // Create a temporary variable to hold the old data
      double[] oldData = bufferData;
      int oldNPoints = oldData.length;

      // Calculate the total number of points after the cut
      int nPoints = computeBufferSizeAfterCut(start, end, oldNPoints);

      // If the result is 0 the size will remain the same
      if (nPoints == 0)
         nPoints = oldNPoints;
      bufferData = new double[nPoints];

      // Transfer the data into the new array beginning with start.
      int difference = end - start + 1;
      for (int i = 0; i < start; i++)
      {
         bufferData[i] = oldData[i];
      }

      for (int i = end + 1; i < oldData.length; i++)
      {
         bufferData[i - difference] = oldData[i];
      }

      boundsDirty = true;

      // Indicate the data length
      return bufferData.length;
   }

   protected int thinData(int keepEveryNthPoint)
   {
      double[] oldData = bufferData;
      int oldNPoints = oldData.length;

      int newNumberOfPoints = oldNPoints / keepEveryNthPoint;
      bufferData = new double[newNumberOfPoints];

      int oldDataIndex = 0;
      for (int index = 0; index < newNumberOfPoints; index++)
      {
         bufferData[index] = oldData[oldDataIndex];

         oldDataIndex = oldDataIndex + keepEveryNthPoint;
      }

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
      // If the start point is outside of the data set abort
      if (shiftIndex <= 0 || shiftIndex >= bufferData.length)
         return;

      // Create a temporary array to carry out the shift
      double[] oldData = bufferData;
      int nPoints = bufferData.length;
      bufferData = new double[nPoints];

      // Repopulate the array using the new order
      for (int i = 0; i < nPoints; i++)
      {
         bufferData[i] = oldData[(i + shiftIndex) % nPoints];
      }

      boundsDirty = true;
   }

   /** {@inheritDoc} */
   @Override
   public synchronized void resetBoundsChangedFlag()
   {
      boundsChanged = false;
   }

   /** {@inheritDoc} */
   @Override
   public synchronized boolean haveBoundsChanged()
   {
      return boundsChanged;
   }

   private synchronized boolean updateBounds()
   {
      boundsChanged = false;

      if (bufferData == null)
         return false;

      currentBounds.setInterval(0, getBufferSize() - 1);
      boundsChanged = currentBounds.compute(bufferData);

      return boundsChanged;
   }

   /** {@inheritDoc} */
   @Override
   public YoBufferBounds getBounds()
   {
      if (boundsDirty)
         updateBounds();

      return currentBounds;
   }

   /** {@inheritDoc} */
   @Override
   public YoBufferBounds getCustomBounds()
   {
      customBounds.setInterval(0, getBufferSize() - 1);
      customBounds.setBounds(variable.getLowerBound(), variable.getUpperBound());
      return customBounds;
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

      double total = 0.0;
      int count = 0;
      int index = 0;

      while (count < length)
      {
         total += bufferData[index];

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
      if (bufferData != null)
      {
         if (boundsDirty || startIndex != currentBounds.getStartIndex() || endIndex != currentBounds.getEndIndex())
         {
            currentBounds.setInterval(startIndex, endIndex);
            boundsChanged = currentBounds.compute(bufferData);
            boundsDirty = false;
         }
      }
      return currentBounds;
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

      for (int i = 0; i < getBufferSize(); i++)
      {
         double thisDataPoint = bufferData[i];
         double otherDataPoint = other.bufferData[i];

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
