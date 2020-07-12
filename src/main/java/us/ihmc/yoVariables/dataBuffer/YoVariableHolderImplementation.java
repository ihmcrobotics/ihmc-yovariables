package us.ihmc.yoVariables.dataBuffer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * <p>
 * Description: An implementation of a YoVariableHolder.
 * </p>
 */
public class YoVariableHolderImplementation implements YoVariableHolder
{
   private final Map<String, List<YoVariable>> yoVariableSet = new LinkedHashMap<>();

   public YoVariableHolderImplementation()
   {
   }

   @Override
   public List<YoVariable> getVariables()
   {
      List<YoVariable> ret = new ArrayList<>();

      Collection<List<YoVariable>> variableLists = yoVariableSet.values();

      for (List<YoVariable> list : variableLists)
      {
         for (YoVariable variable : list)
         {
            ret.add(variable);
         }
      }

      return ret;
   }

   /**
    * Adds the given YoVariables to this YoVariableHolder. If any Variable is not unique, throws a
    * RuntimeException.
    *
    * @param variables YoVariables to add to this YoVariableHolder
    */
   public void addVariablesToHolder(List<YoVariable> variables)
   {
      for (YoVariable variable : variables)
      {
         addVariableToHolder(variable);
      }
   }

   /**
    * Adds the given YoVariable to this YoVariableHolder. If this Variable is not unique, throws a
    * RuntimeException.
    *
    * @param variable YoVariable to add to this YoVariableHolder
    */
   public void addVariableToHolder(YoVariable variable)
   {
      String lowerCaseName = variable.getName();
      lowerCaseName = lowerCaseName.toLowerCase();

      List<YoVariable> variablesWithThisName = yoVariableSet.get(lowerCaseName);
      if (variablesWithThisName == null)
      {
         variablesWithThisName = new ArrayList<>();
         yoVariableSet.put(lowerCaseName, variablesWithThisName);
      }

      // Make sure the variable is unique:
      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         if (variablesWithThisName.get(i).getFullNameString().equals(variable.getFullNameString()))
         {
            System.err.println("Not a unique variable! " + variable.getFullNameString()
                  + " has already been added to this YoVariableHolder!. FullNames are \n");

            for (YoVariable variableToPrint : variablesWithThisName)
            {
               System.err.println(variableToPrint.getFullNameString());
            }

            throw new RuntimeException("Not a unique variable! " + variable.getFullNameString() + " has already been added to this YoVariableHolder!. ");
         }
      }

      variablesWithThisName.add(variable);
   }

   public YoVariable getVariableUsingFullNamespace(String fullname)
   {
      for (YoVariable yoVariable : getVariables())
      {
         if (yoVariable.getFullNameString().equals(fullname))
            return yoVariable;
      }

      // not found
      String error = "Warning: " + fullname + " not found. (YoVariableHolderImplementation.getVariable)";
      System.err.println(error);

      return null;
   }

   @Override
   public YoVariable findVariable(String fullname)
   {
      String name = YoTools.toShortName(fullname);
      List<YoVariable> variablesWithThisName = yoVariableSet.get(name.toLowerCase());

      if (variablesWithThisName == null)
      {
         //         String error = "Warning: " + fullname + " not found. (YoVariableHolderImplementation.getVariable)";
         //         System.err.println(error);

         return null;
      }

      YoVariable foundVariable = null;

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable yoVariable = variablesWithThisName.get(i);
         if (YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, fullname))
         {
            if (foundVariable != null)
            {
               LogTools.error("Called getVariable with " + fullname + ". That is insufficient name information to distinguish a unique variable! "
                     + "Please include more of the name space! Already found " + foundVariable.getFullNameString() + ". Looking for variable "
                     + yoVariable.getFullNameString());
               // new Throwable().printStackTrace(); // Use to find callers.
            }
            else
               foundVariable = yoVariable;
         }
      }

      return foundVariable;
   }

   @Override
   public YoVariable findVariable(String nameSpaceEnding, String name)
   {
      if (name.contains("."))
      {
         throw new RuntimeException(name + " contains a dot. It must not when calling getVariable(String nameSpace, String name)");
      }

      List<YoVariable> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return null;
      }

      YoVariable foundVariable = null;

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable yoVariable = variablesWithThisName.get(i);

         if (yoVariable.getYoRegistry().getNameSpace().endsWith(nameSpaceEnding))
         {
            if (foundVariable != null)
            {
               throw new RuntimeException("Called getVariable with " + nameSpaceEnding + ", " + name
                     + ". That is insufficient name information to distinguish a unique variable! Please include more of the name space!");
            }

            foundVariable = yoVariable;
         }
      }

      return foundVariable;
   }

   @Override
   public boolean hasUniqueVariable(String fullname)
   {
      String name = YoTools.toShortName(fullname);

      List<YoVariable> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return false;
      }

      boolean foundVariable = false;

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable yoVariable = variablesWithThisName.get(i);

         if (YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, fullname))
         {
            if (foundVariable)
            {
               return false;
            }

            foundVariable = true;
         }
      }

      return foundVariable;
   }

   @Override
   public boolean hasUniqueVariable(String nameSpaceEnding, String name)
   {
      if (name.contains("."))
      {
         throw new RuntimeException(name + " contains a dot. It must not when calling hasVariable(String nameSpace, String name)");
      }

      List<YoVariable> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return false;
      }

      boolean foundVariable = false;

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable yoVariable = variablesWithThisName.get(i);

         if (yoVariable.getYoRegistry().getNameSpace().endsWith(nameSpaceEnding))
         {
            if (foundVariable)
            {
               return false;
            }

            foundVariable = true;
         }
      }

      return foundVariable;
   }

   @Override
   public List<YoVariable> findVariables(String nameSpaceEnding, String name)
   {
      if (name.contains("."))
      {
         throw new RuntimeException(name + " contains a dot. It must not when calling getVariables(String nameSpace, String name)");
      }

      List<YoVariable> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return new ArrayList<>(0);
      }

      List<YoVariable> ret = new ArrayList<>();

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable yoVariable = variablesWithThisName.get(i);

         if (yoVariable.getYoRegistry().getNameSpace().endsWith(nameSpaceEnding))
         {
            ret.add(yoVariable);
         }
      }

      return ret;
   }

   @Override
   public List<YoVariable> findVariables(String fullname)
   {
      String name = YoTools.toShortName(fullname);

      List<YoVariable> variablesWithThisName = yoVariableSet.get(name.toLowerCase());
      if (variablesWithThisName == null)
      {
         return new ArrayList<>(0);
      }

      List<YoVariable> ret = new ArrayList<>();

      for (int i = 0; i < variablesWithThisName.size(); i++)
      {
         YoVariable yoVariable = variablesWithThisName.get(i);

         if (YoTools.matchFullNameEndsWithCaseInsensitive(yoVariable, fullname))
         {
            ret.add(yoVariable);
         }
      }

      return ret;
   }

   @Override
   public List<YoVariable> findVariables(NameSpace nameSpace)
   {
      List<YoVariable> ret = new ArrayList<>();

      Collection<List<YoVariable>> variableLists = yoVariableSet.values();

      for (List<YoVariable> list : variableLists)
      {
         for (YoVariable variable : list)
         {
            if (variable.getYoRegistry().getNameSpace().equals(nameSpace))
            {
               ret.add(variable);
            }
         }
      }

      return ret;
   }
}
