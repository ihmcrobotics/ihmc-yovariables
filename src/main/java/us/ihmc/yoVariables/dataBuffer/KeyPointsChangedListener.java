package us.ihmc.yoVariables.dataBuffer;

import java.util.List;

public interface KeyPointsChangedListener
{
   void changed(Change change);

   public static interface Change
   {
      boolean wasToggled();

      boolean areKeyPointsEnabled();

      List<Integer> getAddedKeyPoints();

      List<Integer> getRemovedKeyPoints();
   }
}
