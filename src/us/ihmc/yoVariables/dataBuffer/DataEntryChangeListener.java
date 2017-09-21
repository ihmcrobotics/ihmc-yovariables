package us.ihmc.yoVariables.dataBuffer;

public interface DataEntryChangeListener
{
   public abstract void notifyOfDataChange(DataEntry entry, int index);
}
