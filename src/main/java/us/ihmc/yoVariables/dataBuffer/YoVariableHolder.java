package us.ihmc.yoVariables.dataBuffer;

import java.util.List;

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

public interface YoVariableHolder
{
   /**
    * Returns all the YoVariables in this YoVariableHolder
    *
    * @return ArrayList
    */
   List<YoVariable> getVariables();

   /**
    * Gets a YoVariable with the given name if it is in this YoVariableHolder, otherwise returns null.
    * If name contains a ".", then the YoVariable's nameSpace ending must match that of name. If there
    * is more than one YoVariable that matches, then throws a RuntimeException.
    *
    * @param name String Name of YoVariable to get. If contains a ".", then YoVariable's nameSpace
    *             ending must match that of name.
    * @return YoVariable matching the given name.
    */
   default YoVariable findVariable(String name)
   {
      int separatorIndex = name.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return findVariable(null, name);
      else
         return findVariable(name.substring(0, separatorIndex), name.substring(separatorIndex + 1));
   }

   /**
    * Gets a YoVariable with the given nameSpace and name if it is in this YoVariableHolder, otherwise
    * returns null. If name contains a ".", then throws a RuntimeException. If there is more than one
    * YoVariable that matches, then throws a RuntimeException.
    *
    * @param nameSpaceEnding String nameSpaceEnding ending of YoVariable to get. The YoVariable's
    *                        nameSpace ending must match that of nameSpace.
    * @param name            String Name of YoVariable to get. If contains a ".", then throws a
    *                        RuntimeException.
    * @return YoVariable matching the given nameSpace and name.
    */
   YoVariable findVariable(String nameSpaceEnding, String name);

   /**
    * Checks if this YoVariableHolder holds exactly one YoVariable with the given name. If so, returns
    * true, otherwise returns false. If name contains a ".", then the YoVariable's nameSpace ending
    * must match that of name. If there is more than one YoVariable that matches, returns false.
    *
    * @param name String Name of YoVariable to check for. If contains a ".", then YoVariable's
    *             nameSpace ending must match that of name.
    * @return boolean Whether or not this YoVariableHolder holds exactly one Variable of the given
    *         name.
    */
   default boolean hasUniqueVariable(String name)
   {
      int separatorIndex = name.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return hasUniqueVariable(null, name);
      else
         return hasUniqueVariable(name.substring(0, separatorIndex), name.substring(separatorIndex + 1));
   }

   /**
    * Checks if this YoVariableHolder holds exactly one YoVariable with the given nameSpace and name.
    * If so, returns true, otherwise returns false. If name contains a ".", then throws a
    * RuntimeException. If there is more than one YoVariable that matches, returns false.
    *
    * @param nameSpaceEnding String NameSpace ending of YoVariable to check for. The YoVariable's
    *                        nameSpace ending must match that of nameSpace.
    * @param name            String Name of YoVariable to check for. If contains a ".", then throws a
    *                        RuntimeException.
    * @return boolean Whether or not this YoVariableHolder holds exactly one Variable that matches the
    *         given nameSpace and name.
    */
   boolean hasUniqueVariable(String nameSpaceEnding, String name);

   /**
    * Returns all the YoVariables with the given name that are in this YoVariableHolder, empty if there
    * are none. If name contains a ".", then the YoVariable's nameSpace ending must match that of name.
    *
    * @param name String Name of YoVariable to get. If name contains a ".", then the YoVariable's
    *             nameSpace ending must match that of name.
    * @return ArrayList<YoVariable> matching the given name.
    */
   default List<YoVariable> findVariables(String name)
   {
      int separatorIndex = name.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return findVariables(null, name);
      else
         return findVariables(name.substring(0, separatorIndex), name.substring(separatorIndex + 1));
   }

   /**
    * Returns all the YoVariables with the given nameSpace and name that are in this YoVariableHolder,
    * empty if there are none. If name contains a ".", then throws a RuntimeException.
    *
    * @param nameSpaceEnding String NameSpace ending of YoVariables to get. The YoVariable's nameSpace
    *                        ending must match that of nameSpace.
    * @param name            String Name of YoVariable to get. If contains a ".", then throws a
    *                        RuntimeException.
    * @return ArrayList<YoVariable> matching the given nameSpace and name.
    */
   List<YoVariable> findVariables(String nameSpaceEnding, String name);

   /**
    * Returns all the YoVariables with the given nameSpace that are in this YoVariableHolder, empty if
    * there are none.
    *
    * @param nameSpace NameSpace to match.
    * @return ArrayList<YoVariable> matching YoVariables.
    */
   List<YoVariable> findVariables(NameSpace nameSpace);
}
