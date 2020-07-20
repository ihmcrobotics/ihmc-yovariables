package us.ihmc.yoVariables.buffer.interfaces;

public interface YoBufferReader
{
   int getInPoint();
   
   int getOutPoint();
   
   int getCurrentIndex();

   int getBufferSize();

   void setCurrentIndex(int index);

   boolean tickAndReadFromBuffer(int stepSize);

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
