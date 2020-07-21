package us.ihmc.yoVariables.buffer;

import java.util.Arrays;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferVariableEntryReader;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoBufferVariableEntry implements YoBufferVariableEntryReader
{
   private final YoVariable variable;
   private double[] bufferData;

   private final YoBufferBounds currentBounds = new YoBufferBounds();
   private boolean boundsChanged = true;
   private boolean boundsDirty = true;
   private boolean useCustomBounds = false;
   private final YoBufferBounds customBounds = new YoBufferBounds();

   private boolean inverted = false;

   public YoBufferVariableEntry(YoVariable variable, int bufferSize)
   {
      this.variable = variable;
      clearBuffer(bufferSize);
   }

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

   @Override
   public void setInverted(boolean inverted)
   {
      this.inverted = inverted;
   }

   @Override
   public boolean getInverted()
   {
      return inverted;
   }

   @Override
   public int getBufferSize()
   {
      return bufferData.length;
   }

   public synchronized void writeIntoBufferAt(int index)
   {
      setBufferValueAt(variable.getValueAsDouble(), index);
   }

   public void setBufferValueAt(double data, int index)
   {
      if (bufferData[index] == data)
         return;

      bufferData[index] = data;

      if (currentBounds.update(data))
         boundsChanged = true;
   }

   protected void readFromBufferAt(int index)
   {
      variable.setValueFromDouble(bufferData[index]);
   }

   @Override
   public double getBufferValueAt(int index)
   {
      return bufferData[index];
   }

   @Override
   public double[] getBuffer()
   {
      return getBufferWindow(0, bufferData.length);
   }

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

   @Override
   public void useCustomBounds(boolean autoScale)
   {
      useCustomBounds = !autoScale;
   }

   @Override
   public boolean isUsingCustomBounds()
   {
      return !useCustomBounds;
   }

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

   /**
    * Crop the data stored for this variable using the two specified endpoints. Cropping must reduce or
    * maintain the size of the data set, it cannot be increased.
    *
    * @param start Index of the new start point in the current data
    * @param end   Index of the new end point in the current data
    * @return Overall length of the new data set.
    */
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

      // If the result is 0 the size will remain the same
      if (nPoints == 0)
         nPoints = oldNPoints;
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

   public int cutData(int start, int end)
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

   public int thinData(int keepEveryNthPoint)
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
      return (end - start + 1 + previousBufferSize) % previousBufferSize;
   }

   protected static int computeBufferSizeAfterCut(int start, int end, int previousBufferSize)
   {
      return previousBufferSize - (end - start + 1);
   }

   /**
    * Packs the data based on a new start point. Data is shifted in the array such that the index start
    * is the beginning. Once the data is shifted the min and max values are relocated.
    *
    * @param shiftIndex Index of the data point to become the beginning.
    */
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

   @Override
   public synchronized void resetBoundsChangedFlag()
   {
      boundsChanged = false;
   }

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

   @Override
   public YoBufferBounds getBounds()
   {
      if (boundsDirty)
         updateBounds();

      return currentBounds;
   }

   @Override
   public YoBufferBounds getCustomBounds()
   {
      customBounds.setInterval(0, getBufferSize() - 1);
      customBounds.setBounds(variable.getLowerBound(), variable.getUpperBound());
      return customBounds;
   }

   public double computeAverage()
   {
      double total = 0.0;

      int length = bufferData.length;
      for (int i = 0; i < length; i++)
      {
         total = total + bufferData[i];
      }

      return total / length;
   }

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

   public boolean epsilonEquals(YoBufferVariableEntry other, double epsilon)
   {
      return epsilonEquals(other, 0, getBufferSize() - 1, epsilon);
   }

   public boolean epsilonEquals(YoBufferVariableEntry other, int inPoint, int outPoint, double epsilon)
   {
      if (inPoint >= getBufferSize() || inPoint >= other.getBufferSize())
         return false;
      if (outPoint >= getBufferSize() || outPoint >= other.getBufferSize())
         return false;

      if (inPoint <= outPoint)
      {
         for (int i = inPoint; i < outPoint; i++)
         {
            if (!EuclidCoreTools.epsilonEquals(bufferData[i], other.bufferData[i], epsilon))
            {
               return false;
            }
         }
      }
      else
      {
         if (getBufferSize() != other.getBufferSize())
            return false;

         for (int i = inPoint; i < getBufferSize(); i++)
         {
            if (!EuclidCoreTools.epsilonEquals(bufferData[i], other.bufferData[i], epsilon))
            {
               return false;
            }
         }

         for (int i = 0; i < outPoint; i++)
         {
            if (!EuclidCoreTools.epsilonEquals(bufferData[i], other.bufferData[i], epsilon))
            {
               return false;
            }
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
