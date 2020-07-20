package us.ihmc.yoVariables.dataBuffer;

import us.ihmc.yoVariables.variable.YoVariable;

public interface YoBufferVariableEntryReader
{
   YoVariable getVariable();

   default String getVariableName()
   {
      return getVariable().getName();
   }

   default String getVariableFullNameString()
   {
      return getVariable().getFullNameString();
   }

   double getBufferValueAt(int index);

   double[] getBuffer();

   double[] getBufferWindow(int startIndex, int endIndex);

   void resetBoundsChangedFlag();

   boolean haveBoundsChanged();

   YoBufferBounds getBounds();

   default double getLowerBound()
   {
      return getBounds().getLowerBound();
   }

   default double getUpperBound()
   {
      return getBounds().getUpperBound();
   }

   YoBufferBounds getWindowBounds(int startIndex, int endIndex);

   default double getWindowLowerBound(int startIndex, int endIndex)
   {
      return getWindowBounds(startIndex, endIndex).getLowerBound();
   }

   default double getWindowUpperBound(int startIndex, int endIndex)
   {
      return getWindowBounds(startIndex, endIndex).getUpperBound();
   }

   void useCustomBounds(boolean useCustomBounds);

   boolean isUsingCustomBounds();

   default void setCustomBounds(double customLowerBound, double customUpperBound)
   {
      getVariable().setVariableBounds(customLowerBound, customUpperBound);
   }

   YoBufferBounds getCustomBounds();

   default double getCustomLowerBound()
   {
      return getCustomBounds().getLowerBound();
   }

   default double getCustomUpperBound()
   {
      return getCustomBounds().getUpperBound();
   }

   void setInverted(boolean selected);

   boolean getInverted();
}
