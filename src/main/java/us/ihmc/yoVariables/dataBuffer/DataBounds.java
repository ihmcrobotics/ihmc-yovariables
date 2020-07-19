package us.ihmc.yoVariables.dataBuffer;

public class DataBounds
{
   private int startIndex;
   private int endIndex;
   private double lowerBound;
   private double upperBound;

   public DataBounds()
   {
   }

   void clear()
   {
      this.startIndex = -1;
      this.endIndex = -1;
      lowerBound = Double.POSITIVE_INFINITY;
      upperBound = Double.NEGATIVE_INFINITY;
   }

   void setWindow(int startIndex, int endIndex)
   {
      this.startIndex = startIndex;
      this.endIndex = endIndex;
   }

   void setBounds(double lowerBound, double upperBound)
   {
      this.lowerBound = lowerBound;
      this.upperBound = upperBound;
   }

   void set(DataBounds other)
   {
      startIndex = other.startIndex;
      endIndex = other.endIndex;
      lowerBound = other.lowerBound;
      upperBound = other.upperBound;
   }

   boolean compute(double[] dataBuffer)
   {
      double newLowerBound = Double.POSITIVE_INFINITY;
      double newUpperBound = Double.NEGATIVE_INFINITY;

      for (int i = startIndex; i < endIndex; i++)
      {
         double value = dataBuffer[i];
         if (value < newLowerBound)
            newLowerBound = value;

         if (value > newUpperBound)
            newUpperBound = value;
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

   boolean update(double value)
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

   public boolean isInsideBounds(double value)
   {
      return value >= lowerBound && value <= upperBound;
   }

   public int getStartIndex()
   {
      return startIndex;
   }

   public int getEndIndex()
   {
      return endIndex;
   }

   public double getLowerBound()
   {
      return lowerBound;
   }

   public double getUpperBound()
   {
      return upperBound;
   }
}
