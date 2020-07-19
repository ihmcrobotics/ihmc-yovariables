package us.ihmc.yoVariables.dataBuffer;

public interface DataBufferCommandsExecutor extends GotoInPointCommandExecutor, GotoOutPointCommandExecutor
{
   public abstract int getInPoint();

   public abstract void setIndex(int index);

   public abstract boolean tick(int ticks);

   public abstract int getIndex();

   public abstract int getOutPoint();

   public abstract boolean isIndexBetweenInAndOutPoint(int indexToCheck);
}
