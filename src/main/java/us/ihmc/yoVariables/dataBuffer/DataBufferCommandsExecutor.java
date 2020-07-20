package us.ihmc.yoVariables.dataBuffer;

public interface DataBufferCommandsExecutor
{
   int getInPoint();
   
   int getOutPoint();
   
   int getCurrentIndex();

   int getBufferSize();

   void setCurrentIndex(int index);

   boolean tickAndReadFromBuffer(int ticks);

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
