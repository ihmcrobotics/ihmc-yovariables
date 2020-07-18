package us.ihmc.yoVariables.dataBuffer;

import us.ihmc.yoVariables.variable.YoVariable;

public interface DataEntry
{
   YoVariable getVariable();

   default String getVariableName()
   {
      return getVariable().getName();
   }

   default String getFullVariableNameWithNameSpace()
   {
      return getVariable().getFullNameString();
   }

   double getValueAt(int index);

   double[] getData();

   double[] getData(int startIndex, int endIndex);
   
   double getMax();
   
   double getMin();
   
   double getMax(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex);
   
   double getMin(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex);

   boolean isAutoScaleEnabled();

   double getManualMinScaling();

   double getManualMaxScaling();

   void resetMinMaxChanged();

   boolean hasMinMaxChanged();

   void setManualScaling(double newMinVal, double newMaxVal);

   void enableAutoScale(boolean b);

   void setInverted(boolean selected);

   boolean getInverted();
}
