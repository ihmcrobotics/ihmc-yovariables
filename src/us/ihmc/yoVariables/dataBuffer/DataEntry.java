package us.ihmc.yoVariables.dataBuffer;

import us.ihmc.yoVariables.variable.YoVariable;

public interface DataEntry
{
   public abstract String getVariableName();

   public abstract String getFullVariableNameWithNameSpace();

   public abstract void getVariableNameAndValue(StringBuffer stringBuffer);
   
   public abstract void getVariableNameAndValueAtIndex(StringBuffer stringBuffer, int index);

   public abstract double[] getData();

   public abstract double[] getData(int startIndex, int endIndex);

   public abstract double getMax();

   public abstract double getMin();

   public abstract boolean isAutoScaleEnabled();

   public abstract double getManualMinScaling();

   public abstract double getManualMaxScaling();

   public abstract void resetMinMaxChanged();

   public abstract boolean hasMinMaxChanged();

   public abstract double getMax(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex);

   public abstract double getMin(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex);

   public abstract void setManualScaling(double newMinVal, double newMaxVal);

   public abstract void enableAutoScale(boolean b);

   public abstract YoVariable<?> getVariable();

   public abstract void setInverted(boolean selected);

   public abstract boolean getInverted();

   public abstract void attachDataEntryChangeListener(DataEntryChangeListener listener);

   public abstract void detachDataEntryChangeListener(DataEntryChangeListener listener);

   public abstract void notifyDataEntryChangeListeners(int index);
}
