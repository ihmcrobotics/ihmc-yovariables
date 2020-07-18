package us.ihmc.yoVariables.registry;

import java.util.List;

import us.ihmc.yoVariables.exceptions.IllegalNameException;
import us.ihmc.yoVariables.tools.YoTools;
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
    * Returns the first discovered instance of a variable matching the given name.
    *
    * @param name the name of the variable to retrieve. If the name contains
    *             {@link YoTools#NAMESPACE_SEPERATOR_STRING}, it is split at the last occurrence to
    *             extract a namespace and the actual variable name.
    * @return the variable corresponding to the search criteria, or {@code null} if it could not be
    *         found.
    * @see #findVariable(String, String)
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
    * Returns the first discovered instance of a variable matching the given name.
    *
    * @param nameSpaceEnding (optional) the namespace of the registry in which the variable was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search is for the variable name only.
    * @param name            the name of the variable to retrieve.
    * @return the variable corresponding to the search criteria, or {@code null} if it could not be
    *         found.
    * @throws IllegalNameException if {@code name} contains "{@value YoTools#NAMESPACE_SEPERATOR}".
    */
   YoVariable findVariable(String nameSpaceEnding, String name);

   /**
    * Returns the all the variables matching the given name.
    *
    * @param name the name of the variable to retrieve. If the name contains
    *             {@link YoTools#NAMESPACE_SEPERATOR_STRING}, it is split at the last occurrence to
    *             extract a namespace and the actual variable name.
    * @return list of all the variables corresponding to the search criteria.
    * @throws IllegalNameException if {@code name} contains "{@value YoTools#NAMESPACE_SEPERATOR}".
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
    * Returns the all the variables matching the given name and namespace.
    *
    * @param nameSpaceEnding (optional) the namespace of the registry in which the variable was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search for the variable name only.
    * @param name            the name of the variable to retrieve.
    * @return list of all the variables corresponding to the search criteria.
    * @throws IllegalNameException if {@code name} contains "{@value YoTools#NAMESPACE_SEPERATOR}".
    */
   List<YoVariable> findVariables(String nameSpaceEnding, String name);

   /**
    * Searches for all the variables which namespace match the given one.
    *
    * @param nameSpace the full namespace of the registry of interest.
    * @return the variables that were registered at the given namespace.
    */
   List<YoVariable> findVariables(NameSpace nameSpace);

   /**
    * Searches this variable holder and tests if there is exactly one variable that matches the search
    * criteria.
    *
    * @param name the name of the variable of interest. If the name contains
    *             {@link YoTools#NAMESPACE_SEPERATOR_STRING}, it is split at the last occurrence to
    *             extract a namespace and the actual variable name.
    * @return {@code true} if there is exactly one variable that matches the search criteria,
    *         {@code false} otherwise.
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
    * Searches this variable holder and tests if there is exactly one variable that matches the search
    * criteria.
    *
    * @param nameSpaceEnding (optional) the namespace of the registry in which the variable was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search for the variable name only.
    * @param name            the name of the variable.
    * @return {@code true} if there is exactly one variable that matches the search criteria,
    *         {@code false} otherwise.
    * @throws IllegalNameException if {@code name} contains "{@value YoTools#NAMESPACE_SEPERATOR}".
    */
   boolean hasUniqueVariable(String nameSpaceEnding, String name);

   /**
    * Searches this variable holder and tests if there is at least one variable that matches the search
    * criteria.
    *
    * @param name the name of the variable(s) of interest. If the name contains
    *             {@link YoTools#NAMESPACE_SEPERATOR_STRING}, it is split at the last occurrence to
    *             extract a namespace and the actual variable name.
    * @return {@code true} if there is exactly one variable that matches the search criteria,
    *         {@code false} otherwise.
    */
   default boolean hasVariable(String name)
   {
      return findVariable(name) != null;
   }

   /**
    * Searches this variable holder and tests if there is at least one variable that matches the search
    * criteria.
    *
    * @param nameSpaceEnding (optional) the namespace of the registry in which the variable(s) was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search for the variable name only.
    * @param name            the name of the variable(s).
    * @return {@code true} if there is exactly one variable that matches the search criteria,
    *         {@code false} otherwise.
    * @throws IllegalNameException if {@code name} contains "{@value YoTools#NAMESPACE_SEPERATOR}".
    */
   default boolean hasVariable(String nameSpaceEnding, String name)
   {
      return findVariable(nameSpaceEnding, name) != null;
   }
}