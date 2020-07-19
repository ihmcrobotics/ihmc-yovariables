package us.ihmc.yoVariables.dataBuffer;

import java.util.Arrays;

import us.ihmc.yoVariables.variable.YoVariable;

public class DataBufferEntry implements DataEntry
{
   private final YoVariable variable;
   private double[] bufferData;

   private final DataBounds currentBounds = new DataBounds();
   private boolean boundsChanged = true;
   private boolean boundsDirty = true;
   private boolean useCustomBounds = false;
   private final DataBounds customBounds = new DataBounds();

   private boolean inverted = false;

   public DataBufferEntry(YoVariable variable, int bufferSize)
   {
      this.variable = variable;
      clear(bufferSize);
   }

   public DataBufferEntry(DataBufferEntry other)
   {
      this.variable = other.getVariable();
      bufferData = Arrays.copyOf(other.bufferData, other.getDataLength());
      currentBounds.set(other.currentBounds);
      boundsChanged = other.boundsChanged;
      boundsDirty = other.boundsDirty;
      useCustomBounds = other.useCustomBounds;
      customBounds.set(other.customBounds);
      inverted = other.inverted;
   }

   public void clear(int bufferSize)
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

   public int getDataLength()
   {
      return bufferData.length;
   }

   public synchronized void setDataAtIndexToYoVariableValue(int index)
   {
      setDataAt(variable.getValueAsDouble(), index);
   }

   public void setDataAt(double data, int index)
   {
      if (bufferData[index] == data)
         return;

      this.bufferData[index] = data;

      if (currentBounds.update(data))
         boundsChanged = true;
   }

   protected void setYoVariableValueToDataAtIndex(int index)
   {
      variable.setValueFromDouble(bufferData[index]);
   }

   @Override
   public double getValueAt(int index)
   {
      return bufferData[index];
   }

   @Override
   public double[] getBuffer()
   {
      return this.getBufferWindow(0, bufferData.length);
   }

   @Override
   public double[] getBufferWindow(int startIndex, int endIndex)
   {
      return Arrays.copyOfRange(bufferData, startIndex, endIndex);
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

   protected void copyValueThrough()
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

      updateBounds();
   }

   /**
    * Crop the data stored for this variable using the two specified endpoints. Cropping must reduce or
    * maintain the size of the data set, it cannot be increased.
    *
    * @param start Index of the new start point in the current data
    * @param end   Index of the new end point in the current data
    * @return Overall length of the new data set.
    */
   protected int cropData(int start, int end)
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

      // Calculate the Min and Max values for the new set
      updateBounds();

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

      // Calculate the Min and Max values for the new set
      updateBounds();

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
    * @param start Index of the data point to become the beginning.
    */
   protected void packData(int start)
   {
      // If the start point is outside of the data set abort
      if (start <= 0 || start >= bufferData.length)
         return;

      // Create a temporary array to carry out the shift
      double[] oldData = bufferData;
      int nPoints = bufferData.length;
      bufferData = new double[nPoints];

      // Repopulate the array using the new order
      for (int i = 0; i < nPoints; i++)
      {
         bufferData[i] = oldData[(i + start) % nPoints];
      }

      // Recalculate the new min and max values
      updateBounds();
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

      currentBounds.setWindow(0, getDataLength() - 1);
      boundsChanged = currentBounds.compute(bufferData);

      return boundsChanged;
   }

   @Override
   public DataBounds getBounds()
   {
      if (boundsDirty)
         updateBounds();

      return currentBounds;
   }

   @Override
   public DataBounds getCustomBounds()
   {
      customBounds.setWindow(0, getDataLength() - 1);
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

   public double[] getWindowedData(int in, /* int out, */int bufferLength)
   {
      double[] ret = new double[bufferLength];
      int n = in;

      for (int i = 0; i < bufferLength; i++)
      {
         ret[i] = bufferData[n];
         n++;
         if (n >= bufferData.length)
            n = 0;
      }

      return ret;
   }

   @Override
   public DataBounds getWindowBounds(int startIndex, int endIndex)
   {
      if (bufferData == null)
      {
         currentBounds.setWindow(startIndex, endIndex);
         boundsChanged = currentBounds.compute(bufferData);
      }
      return currentBounds;
   }

   public boolean checkIfDataIsEqual(DataBufferEntry entry2, int inPoint, int outPoint, double epsilon)
   {
      //      System.out.println(this.variable.getName() + ": InPoint = " + inPoint + ", outPoint = " + outPoint);

      if (inPoint >= bufferData.length)
         return false;
      if (inPoint >= entry2.bufferData.length)
         return false;
      if (outPoint >= bufferData.length)
         return false;
      if (outPoint >= entry2.bufferData.length)
         return false;

      if (inPoint > outPoint)
         throw new RuntimeException("Sorry, but we assume that inPoint is not greater than outPoint in this method!");

      boolean ret = true;
      for (int i = inPoint; i < outPoint; i++)
      {
         double dataOne = bufferData[i];
         double dataTwo = entry2.bufferData[i];

         //         System.out.println(this.variable.getName() + ": dataOne = " + dataOne + ", dataTwo = " + dataTwo);

         if (Math.abs(dataOne - dataTwo) > epsilon)
         {
            ret = false;
         }
      }

      return ret;
   }
}
