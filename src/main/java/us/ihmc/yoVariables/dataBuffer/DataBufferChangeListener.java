package us.ihmc.yoVariables.dataBuffer;

public interface DataBufferChangeListener
{
   public abstract void notifyOfBufferChange();

   public abstract void notifyOfManualEndChange(int inPoint, int outPoint);
}
