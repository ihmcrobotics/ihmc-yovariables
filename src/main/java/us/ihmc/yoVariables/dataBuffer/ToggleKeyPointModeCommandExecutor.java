package us.ihmc.yoVariables.dataBuffer;

public interface ToggleKeyPointModeCommandExecutor
{
   public abstract boolean isKeyPointModeToggled();

   public abstract void toggleKeyPointMode();

   public abstract void registerToggleKeyPointModeCommandListener(ToggleKeyPointModeCommandListener commandListener);
}
