package us.ihmc.yoVariables.dataBuffer;

public interface KeyPointsHolder
{
   void toggleKeyPoints();
   
   boolean areKeyPointsEnabled();

   void addListener(KeyPointsChangedListener listener);
}
