package us.ihmc.yoVariables.registry;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.dataBuffer.YoVariableHolder;
import us.ihmc.yoVariables.listener.YoVariableRegistryChangedListener;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableRegistry implements YoVariableHolder
{
   private final String name;
   private NameSpace nameSpace;

   private List<YoVariable<?>> variables = new ArrayList<>();
   private Map<String, YoVariable<?>> nameToVariableMap = new LinkedHashMap<>(); // From name to the variable with that name.
   private List<YoParameter<?>> parameters = new ArrayList<>();
   private Map<String, YoParameter<?>> nameToParameterMap = new LinkedHashMap<>();

   private YoVariableRegistry parent;
   private List<YoVariableRegistry> children = new ArrayList<>();

   private List<YoVariableRegistryChangedListener> yoVariableRegistryChangedListeners;

   public YoVariableRegistry(String name)
   {
      YoVariableTools.checkForIllegalCharacters(name);

      this.name = name;

      if (name != null && name.length() > 0)
      {
         nameSpace = new NameSpace(name);
      }
   }

   public String getName()
   {
      return name;
   }

   public NameSpace getNameSpace()
   {
      return nameSpace;
   }

   public void attachYoVariableRegistryChangedListener(YoVariableRegistryChangedListener listener)
   {
      if (yoVariableRegistryChangedListeners == null)
      {
         yoVariableRegistryChangedListeners = new ArrayList<>();
      }

      yoVariableRegistryChangedListeners.add(listener);

      verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();
   }

   public void registerVariable(YoVariable<?> variable)
   {
      String variableName = variable.getName();
      variableName = variableName.toLowerCase(); // Make everything case insensitive! Cannot have two YoVariables with same names except case.

      if (nameToVariableMap.containsKey(variableName))
      {
         System.err.println("Error:  " + variable.getName() + " has already been registered in this registry! YoVariableRegistry NameSpace = " + nameSpace);
         // Sometimes RuntimeExceptions are not displayed, but the error out is still visible.
         throw new RuntimeException("Error:  " + variable.getName() + " has already been registered in this registry! YoVariableRegistry NameSpace = "
               + nameSpace);
      }

      nameToVariableMap.put(variableName, variable);
      variables.add(variable);

      if (variable.isParameter())
      {
         YoParameter<?> parameter = variable.getParameter();
         parameters.add(parameter);
         nameToParameterMap.put(variableName, parameter);
      }

      notifyListenersYoVariableWasRegistered(variable);
   }

   /**
    * Returns the variables that have been registered to this registry.
    *
    * @return unmodifiable list of this registry's variables.
    */
   @Override
   public List<YoVariable<?>> getYoVariables()
   {
      return Collections.unmodifiableList(variables);
   }

   /**
    * Returns the parameters that have been registered to this registry.
    *
    * @return unmodifiable list of this registry's parameters.
    */
   public List<YoParameter<?>> getYoParameters()
   {
      return Collections.unmodifiableList(parameters);
   }

   /**
    * Returns the parent registry for this registry or {@code null} if {@code this} is the root
    * registry.
    *
    * @return the parent or {@code null} if {@code this} is the root registry.
    */
   public YoVariableRegistry getParent()
   {
      return parent;
   }

   /**
    * Returns the child registries contained in this registry.
    *
    * @return this registry direct children.
    */
   public List<YoVariableRegistry> getChildren()
   {
      return children;
   }

   /**
    * Collects recursively and returns all the variables contained in this registry and in its
    * descendants.
    *
    * @return list of all variables registered to this registry and its child registries.
    */
   public List<YoVariable<?>> getSubtreeYoVariables()
   {
      List<YoVariable<?>> yoVariables = new ArrayList<>();
      getSubtreeYoVariables(yoVariables);
      return yoVariables;
   }

   /**
    * Collects recursively and packs all the variables contained in this registry and in its
    * descendants into {@code variablesToPack}.
    *
    * @param variablesToPack list used to store all variables registered to this registry and its child
    *                        registries.
    */
   private void getSubtreeYoVariables(List<YoVariable<?>> variablesToPack)
   {
      // Add ours:
      variablesToPack.addAll(variables);

      // Add children's recursively:
      for (YoVariableRegistry registry : children)
      {
         registry.getSubtreeYoVariables(variablesToPack);
      }
   }

   /**
    * Collects recursively and returns all the parameters contained in this registry and in its
    * descendants.
    *
    * @return list of all parameters registered to this registry and its child registries.
    */
   public List<YoParameter<?>> getSubtreeYoParameters()
   {
      List<YoParameter<?>> yoParameters = new ArrayList<>();
      getSubtreeYoParameters(yoParameters);
      return yoParameters;
   }

   /**
    * Collects recursively and packs all the parameters contained in this registry and in its
    * descendants into {@code parametersToPack}.
    *
    * @param parametersToPack list used to store all parameters registered to this registry and its
    *                         child registries.
    */
   private void getSubtreeYoParameters(List<YoParameter<?>> parametersToPack)
   {
      // Add ours:
      parametersToPack.addAll(parameters);

      // Add children's recursively:
      for (YoVariableRegistry registry : children)
      {
         registry.getSubtreeYoParameters(parametersToPack);
      }
   }

   /**
    * Collects recursively and returns the subtree of registries starting with this registry.
    *
    * @return list of all the registries composing the subtree starting at {@code this}.
    */
   public List<YoVariableRegistry> getSubtreeYoVariableRegistries()
   {
      List<YoVariableRegistry> yoVariableRegistries = new ArrayList<>();
      getSubtreeYoVariableRegistries(yoVariableRegistries);

      return yoVariableRegistries;
   }

   /**
    * Collects recursively and packs the subtree of registries starting with this registry into
    * {@code yoVariableRegistriesToPack}.
    *
    * @param yoVariableRegistriesToPack list used to store all the registries composing the subtree
    *                                   starting at {@code this}.
    */
   public void getSubtreeYoVariableRegistries(List<YoVariableRegistry> yoVariableRegistriesToPack)
   {
      // Add mine:
      yoVariableRegistriesToPack.add(this);

      // Add all the children recursively:
      for (YoVariableRegistry child : children)
      {
         child.getSubtreeYoVariableRegistries(yoVariableRegistriesToPack);
      }
   }

   /**
    * Returns the first discovered instance of a variable matching the given name.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    * <p>
    * The given {@code name} can either be:
    * <ul>
    * <li>the variable name, for instance {@code "aVariable"},
    * <li>the variable name and its namespace, for instance:
    * {@code "aRegistry.anotherRegistry.aVariable"}.
    * </ul>
    * </p>
    * 
    * @param name the name of the variables to retrieve. Can contain the namespace of its parent to
    *             narrow down the search.
    */
   @Override
   public YoVariable<?> getYoVariable(String name)
   {
      int separatorIndex = name.lastIndexOf(YoVariableTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return getYoVariable(null, name);
      else
         return getYoVariable(name.substring(0, separatorIndex + 1), name.substring(separatorIndex));
   }

   /**
    * Returns the first discovered instance of a variable matching the given name.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    * 
    * @param parentNameSpace (optional) the namespace of the registry in which the variable was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search for the variable name only.
    * @param name            the name of the variable to retrieve.
    * @return the variable corresponding to the search criteria, or {@code null} if it could not be
    *         found.
    * @throws IllegalNameException if {@code name} contains
    *                              "{@value YoVariableTools#NAMESPACE_SEPERATOR}".
    */
   @Override
   public YoVariable<?> getYoVariable(String parentNameSpace, String name)
   {
      if (name.contains(YoVariableTools.NAMESPACE_SEPERATOR_STRING))
         throw new IllegalNameException(name + " cannot contain '" + YoVariableTools.NAMESPACE_SEPERATOR_STRING + "'.");

      if (parentNameSpace == null || nameSpace.endsWith(parentNameSpace))
      {
         YoVariable<?> variable = nameToVariableMap.get(name);
         if (variable != null)
            return variable;
      }

      for (YoVariableRegistry child : children)
      {
         YoVariable<?> variable = child.getYoVariable(parentNameSpace, name);
         if (variable != null)
            return variable;
      }

      return null;
   }

   /**
    * Returns the all of the variables matching the given name.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    * <p>
    * The given {@code name} can either be:
    * <ul>
    * <li>the variable name, for instance {@code "aVariable"},
    * <li>the variable name and its namespace, for instance:
    * {@code "aRegistry.anotherRegistry.aVariable"}.
    * </ul>
    * </p>
    * 
    * @param name the name of the variables to retrieve. Can contain the namespace of its parent to
    *             narrow down the search.
    * @return list of all the variables corresponding to the search criteria.
    */
   @Override
   public List<YoVariable<?>> getYoVariables(String name)
   {
      List<YoVariable<?>> matchedVariables = new ArrayList<>();
      getYoVariables(name, matchedVariables);
      return matchedVariables;
   }

   /**
    * Returns the all of the variables matching the given name and namespace.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    * 
    * @param parentNameSpace (optional) the namespace of the registry in which the variable was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search for the variable name only.
    * @param name            the name of the variable to retrieve.
    * @return list of all the variables corresponding to the search criteria.
    * @throws IllegalNameException if {@code name} contains
    *                              "{@value YoVariableTools#NAMESPACE_SEPERATOR}".
    */
   @Override
   public List<YoVariable<?>> getYoVariables(String parentNameSpace, String name)
   {
      List<YoVariable<?>> matchedVariables = new ArrayList<>();
      getYoVariables(parentNameSpace, name, matchedVariables);
      return matchedVariables;
   }

   /**
    * Collects and stores the all of the variables matching the given name into
    * {@code matchedVariablesToPack}.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    * <p>
    * The given {@code name} can either be:
    * <ul>
    * <li>the variable name, for instance {@code "aVariable"},
    * <li>the variable name and its namespace, for instance:
    * {@code "aRegistry.anotherRegistry.aVariable"}.
    * </ul>
    * </p>
    * 
    * @param name                   the name of the variables to retrieve. Can contain the namespace of
    *                               its parent to narrow down the search.
    * @param matchedVariablesToPack list used to store all the variables corresponding to the search
    *                               criteria.
    */
   public void getYoVariables(String name, List<YoVariable<?>> matchedVariablesToPack)
   {
      int separatorIndex = name.lastIndexOf(YoVariableTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         getYoVariables(null, name, matchedVariablesToPack);
      else
         getYoVariables(name.substring(0, separatorIndex + 1), name.substring(separatorIndex), matchedVariablesToPack);
   }

   /**
    * Collects and stores the all of the variables matching the given name and namespace into
    * {@code matchedVariablesToPack}.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    * 
    * @param parentNameSpace        (optional) the namespace of the registry in which the variable was
    *                               registered. The namespace does not need to be complete, i.e. it
    *                               does not need to contain the name of the registries closest to the
    *                               root registry. If {@code null}, the search for the variable name
    *                               only.
    * @param name                   the name of the variables to retrieve. Can contain the namespace of
    *                               its parent to narrow down the search.
    * @param matchedVariablesToPack list used to store all the variables corresponding to the search
    *                               criteria.
    */
   public void getYoVariables(String parentNameSpace, String name, List<YoVariable<?>> matchedVariablesToPack)
   {
      if (name.contains(YoVariableTools.NAMESPACE_SEPERATOR_STRING))
         throw new IllegalNameException(name + " cannot contain '" + YoVariableTools.NAMESPACE_SEPERATOR_STRING + "'.");

      if (parentNameSpace == null || nameSpace.endsWith(parentNameSpace))
      {
         YoVariable<?> variable = nameToVariableMap.get(name);
         if (variable != null)
            matchedVariablesToPack.add(variable);
      }

      for (YoVariableRegistry child : children)
      {
         child.getYoVariables(parentNameSpace, name, matchedVariablesToPack);
      }
   }

   /**
    * Search in the subtree starting at this registry and tests if there is exactly one variable that
    * matches the search criteria.
    * 
    * @param parentNameSpace (optional) the namespace of the registry in which the variable was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search for the variable name only.
    * @param name            the name of the variable to retrieve.
    * @return {@code true} if there is exactly one variable that matches the search criteria,
    *         {@code false} otherwise.
    */
   @Override
   public boolean hasUniqueYoVariable(String parentNameSpace, String name)
   {
      if (name.contains(YoVariableTools.NAMESPACE_SEPERATOR_STRING))
         throw new IllegalNameException(name + " cannot contain '" + YoVariableTools.NAMESPACE_SEPERATOR_STRING + "'.");

      return countNumberOfYoVariables(parentNameSpace, name) == 1;
   }

   /**
    * Search in the subtree starting at this registry and tests if there is exactly one variable that
    * matches the search criteria.
    * <p>
    * The given {@code name} can either be:
    * <ul>
    * <li>the variable name, for instance {@code "aVariable"},
    * <li>the variable name and its namespace, for instance:
    * {@code "aRegistry.anotherRegistry.aVariable"}.
    * </ul>
    * </p>
    * 
    * @param name the name of the variables to retrieve. Can contain the namespace of its parent to
    *             narrow down the search.
    * @return {@code true} if there is exactly one variable that matches the search criteria,
    *         {@code false} otherwise.
    */
   @Override
   public boolean hasUniqueYoVariable(String name)
   {
      int separatorIndex = name.lastIndexOf(YoVariableTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return hasUniqueYoVariable(null, name);
      else
         return hasUniqueYoVariable(name.substring(0, separatorIndex + 1), name.substring(separatorIndex));
   }

   private int countNumberOfYoVariables(String parentNameSpace, String name)
   {
      int count = 0;

      if (parentNameSpace == null || nameSpace.endsWith(parentNameSpace))
      {
         if (nameToVariableMap.containsKey(name))
            count++;
      }

      for (YoVariableRegistry child : children)
      {
         count += child.countNumberOfYoVariables(parentNameSpace, name);
      }

      return count;
   }

   public void addChild(YoVariableRegistry child)
   {
      addChild(child, true);
   }

   public void addChild(YoVariableRegistry child, boolean notifyListeners)
   {
      // Prepend the parents nameSpace to the child. NameSpace will figure out if it's valid or not.
      // This then requires that the child only has it's portion of the NameSpace that the parent does not.

      if (child == null)
         return;

      // Make sure no children with this name already:
      for (YoVariableRegistry childToCheck : children)
      {
         String childToCheckShortName = childToCheck.getNameSpace().getShortName();
         String childShortName = child.getNameSpace().getShortName();
         if (childToCheckShortName.equals(childShortName))
         {
            throw new RuntimeException("Adding a child to a YoVariableRegistry that has the same name as a previous one: "
                  + childToCheck.getNameSpace().getName() + ". Parent name space = " + getNameSpace().getName());
         }
      }

      NameSpace parentNameSpace = getNameSpace();

      child.prependNameSpace(parentNameSpace);

      //    System.err.println("Child: " + child.getNameSpace().getShortName() + " parent:" + this.getNameSpace().getName());

      child.setParent(this);
      children.add(child);

      if (notifyListeners)
         notifyListenersYoVariableRegistryWasAdded(child);
   }

   private void prependNameSpace(NameSpace parentNameSpace)
   {
      if (nameSpace == null)
         throw new RuntimeException("Cannot prepend a NameSpace. This NameSpace is null. Only root can have a null NameSpace");
      if (parentNameSpace == null)
         return;

      // Fix my name
      nameSpace = new NameSpace(parentNameSpace.getName() + "." + nameSpace.getName());

      // Fix my children
      for (YoVariableRegistry child : children)
      {
         child.prependNameSpace(parentNameSpace);
      }
   }

   private void setParent(YoVariableRegistry parent)
   {
      if (this.parent != null)
         throw new RuntimeException("Parent was already set!! It was " + this.parent + ". this = " + this);

      this.parent = parent;

      verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();
   }

   public void clear()
   {
      variables.clear();
      nameToVariableMap.clear();
      children.clear();

      notifyListenersYoVariableRegistryWasCleared(this);
   }

   @Override
   public String toString()
   {
      return nameSpace.getName();
   }

   public YoVariableRegistry getOrCreateAndAddRegistry(NameSpace fullNameSpace)
   {
      if (nameSpace == null && fullNameSpace == null)
      {
         return this;
      }

      if (nameSpace != null && nameSpace.equals(fullNameSpace))
      {
         return this;
      }

      if (nameSpace == null || fullNameSpace.startsWith(nameSpace.getName()))
      {
         for (YoVariableRegistry child : children)
         {
            YoVariableRegistry registry = child.getOrCreateAndAddRegistry(fullNameSpace);
            if (registry != null)
               return registry;
         }

         // If, after going through all the children, none of them match, then
         // create it here and return it.
         NameSpace nameSpaceToContinueWith = fullNameSpace.stripOffFromBeginning(nameSpace);
         YoVariableRegistry registry = createChainOfRegistries(nameSpaceToContinueWith);
         this.addChild(registry);

         return getToBottomRegistry(registry);
      }

      return null;
   }

   public YoVariableRegistry getRegistry(NameSpace fullNameSpace)
   {
      if (nameSpace == null && fullNameSpace == null)
      {
         return this;
      }

      if (nameSpace != null && nameSpace.equals(fullNameSpace))
      {
         return this;
      }

      if (nameSpace == null || fullNameSpace.startsWith(nameSpace.getName()))
      {
         for (YoVariableRegistry child : children)
         {
            YoVariableRegistry registry = child.getRegistry(fullNameSpace);
            if (registry != null)
               return registry;
         }

         throw new RuntimeException("Registry not found");
      }

      return null;
   }

   public int getNumberOfYoVariables()
   {
      return variables.size();
   }

   public YoVariable<?> getYoVariable(int index)
   {
      return variables.get(index);
   }

   private YoVariableRegistry createChainOfRegistries(NameSpace fullNameSpace)
   {
      String rootName = fullNameSpace.getRootName();
      YoVariableRegistry rootRegistry = new YoVariableRegistry(rootName);

      String subName = fullNameSpace.getNameWithRootStripped();
      if (subName == null)
         return rootRegistry;

      NameSpace subNameSpace = new NameSpace(subName);
      YoVariableRegistry subRegistry = createChainOfRegistries(subNameSpace);

      rootRegistry.addChild(subRegistry);

      return rootRegistry;
   }

   private static YoVariableRegistry getToBottomRegistry(YoVariableRegistry root)
   {
      if (root.children == null || root.children.size() == 0)
         return root;
      if (root.children.size() > 1)
         throw new RuntimeException("This should only be called with a new chain!!");

      return getToBottomRegistry(root.children.get(0));
   }

   public void printAllVariablesIncludingDescendants(PrintStream out)
   {
      for (YoVariable<?> var : variables)
      {
         out.print(var.getFullNameWithNameSpace() + "\n");
      }

      for (YoVariableRegistry child : children)
      {
         child.printAllVariablesIncludingDescendants(out);
      }
   }

   @Override
   public List<YoVariable<?>> getVariables(NameSpace nameSpace)
   {
      List<YoVariable<?>> ret = new ArrayList<>();

      List<YoVariable<?>> allVariables = getSubtreeYoVariables();

      for (YoVariable<?> variable : allVariables)
      {
         if (variable.getYoVariableRegistry().getNameSpace().equals(nameSpace))
         {
            ret.add(variable);
         }
      }

      return ret;
   }

   public synchronized List<YoVariable<?>> getMatchingVariables(String[] names, String[] regularExpressions)
   {
      List<YoVariable<?>> ret = new ArrayList<>();

      if (names != null)
      {
         for (int i = 0; i < names.length; i++)
         {
            if (names[i] != null)
            {
               String name = names[i];
               YoVariable<?> var = getYoVariable(name);

               if (var != null)
               {
                  ret.add(var);
               }
            }
         }
      }

      recursivelyGetMatchingVariables(ret, regularExpressions);

      return ret;
   }

   private void recursivelyGetMatchingVariables(List<YoVariable<?>> ret, String[] regularExpressions)
   {
      if (regularExpressions != null)
      {
         for (int i = 0; i < regularExpressions.length; i++)
         {
            Pattern pattern = Pattern.compile(regularExpressions[i]);

            for (int j = 0; j < variables.size(); j++)
            {
               YoVariable<?> var = variables.get(j);
               Matcher matcher = pattern.matcher(var.getName());

               if (matcher.matches())
               {
                  ret.add(var);
               }
            }
         }
      }

      for (YoVariableRegistry child : children)
      {
         child.recursivelyGetMatchingVariables(ret, regularExpressions);
      }
   }

   public boolean areEqual(YoVariableRegistry registry)
   {
      if (registry == null)
         return false;

      if (!getNameSpace().equals(registry.getNameSpace()))
         return false;

      for (YoVariable<?> variable : variables)
      {
         if (getVariableWithSameName(registry.variables, variable) == null)
         {
            return false;
         }
      }

      if (children.size() != registry.children.size())
         return false;

      for (YoVariableRegistry child : children)
      {
         YoVariableRegistry matchingChild = getRegistryWithSameNameSpace(registry.children, child);
         if (!child.areEqual(matchingChild))
         {
            return false;
         }
      }

      return true;
   }

   private static YoVariable<?> getVariableWithSameName(List<YoVariable<?>> variables, YoVariable<?> variableToMatch)
   {
      for (YoVariable<?> variable : variables)
      {
         if (variable.getFullNameWithNameSpace().equals(variableToMatch.getFullNameWithNameSpace()))
            return variable;
      }

      return null;
   }

   private static YoVariableRegistry getRegistryWithSameNameSpace(List<YoVariableRegistry> registries, YoVariableRegistry registryToMatch)
   {
      for (YoVariableRegistry registry : registries)
      {
         if (registryToMatch.getNameSpace().equals(registry.getNameSpace()))
         {
            return registry;
         }
      }

      return null;
   }

   private void verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners()
   {
      if (parent != null && yoVariableRegistryChangedListeners != null)
      {
         throw new RuntimeException("Only root YoVariableRegistries should have listeners. This registry does! YoVariableRegistry = " + this);
      }
   }

   private void notifyListenersYoVariableRegistryWasAdded(YoVariableRegistry child)
   {
      // Push it up the chain. Only the root will notify it's listeners. Non-roots shouldn't have listeners.
      verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();

      if (parent != null)
      {
         parent.notifyListenersYoVariableRegistryWasAdded(child);
      }
      else if (yoVariableRegistryChangedListeners != null)
      {
         for (YoVariableRegistryChangedListener listener : yoVariableRegistryChangedListeners)
         {
            listener.yoVariableRegistryWasAdded(child);
         }
      }
   }

   private void notifyListenersYoVariableRegistryWasCleared(YoVariableRegistry registry)
   {
      // Push it up the chain. Only the root will notify it's listeners. Non-roots shouldn't have listeners.
      verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();

      if (parent != null)
      {
         parent.notifyListenersYoVariableRegistryWasCleared(registry);
      }

      if (yoVariableRegistryChangedListeners != null)
      {
         for (YoVariableRegistryChangedListener listener : yoVariableRegistryChangedListeners)
         {
            listener.yoVariableRegistryWasCleared(registry);
         }
      }
   }

   private void notifyListenersYoVariableWasRegistered(YoVariable<?> variable)
   {
      // Push it up the chain. Only the root will notify it's listeners. Non-roots shouldn't have listeners.
      verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();

      if (parent != null)
      {
         parent.notifyListenersYoVariableWasRegistered(variable);
      }

      else if (yoVariableRegistryChangedListeners != null)
      {
         for (YoVariableRegistryChangedListener listener : yoVariableRegistryChangedListeners)
         {
            listener.yoVariableWasRegistered(this, variable);
         }
      }
   }

   /**
    * Checks if this registry or its children have parameters registered
    *
    * @return true if this registry or its children have parameters registered
    */
   public boolean getIfRegistryOrChildrenHaveParameters()
   {
      if (!parameters.isEmpty())
      {
         return true;
      }

      for (int i = 0; i < children.size(); i++)
      {
         if (children.get(i).getIfRegistryOrChildrenHaveParameters())
         {
            return true;
         }
      }

      return false;
   }

   public void closeAndDispose()
   {
      if (variables != null)
      {
         variables.clear();
         variables = null;
      }

      if (nameToVariableMap != null)
      {
         nameToVariableMap.clear();
         nameToVariableMap = null;
      }

      if (parameters != null)
      {
         parameters.clear();
         parameters = null;
      }

      if (nameToParameterMap != null)
      {
         nameToParameterMap.clear();
         nameToParameterMap = null;
      }

      if (children != null)
      {
         for (YoVariableRegistry child : children)
         {
            child.closeAndDispose();
         }
         children = null;
      }

      parent = null;

      if (yoVariableRegistryChangedListeners != null)
      {
         yoVariableRegistryChangedListeners.clear();
         yoVariableRegistryChangedListeners = null;
      }
   }

   public static void printSizeRecursively(int minVariablesToPrint, int minChildrenToPrint, YoVariableRegistry root)
   {
      List<YoVariableRegistry> registriesOfInterest = new ArrayList<>();
      int totalVariables = collectRegistries(minVariablesToPrint, minChildrenToPrint, root, registriesOfInterest);
      Collections.sort(registriesOfInterest, new Comparator<YoVariableRegistry>()
      {
         @Override
         public int compare(YoVariableRegistry o1, YoVariableRegistry o2)
         {
            if (o1.getNumberOfYoVariables() == o2.getNumberOfYoVariables())
               return 0;
            return o1.getNumberOfYoVariables() > o2.getNumberOfYoVariables() ? -1 : 1;
         }
      });

      System.out.println("");
      LogTools.info("Printing children of " + root.getName() + " registry.");
      System.out.println("Total Number of YoVariables: " + totalVariables);
      System.out.println("Listing registries with at least " + minVariablesToPrint + " variables or at least " + minChildrenToPrint + " children.");
      System.out.println("Sorting by number of variables.");

      for (int registryIdx = 0; registryIdx < registriesOfInterest.size(); registryIdx++)
         YoVariableRegistry.printInfo(registriesOfInterest.get(registryIdx));

      System.out.println("");
   }

   private static int collectRegistries(int minVariablesToPrint, int minChildrenToPrint, YoVariableRegistry registry,
                                        List<YoVariableRegistry> registriesOfInterest)
   {
      int variables = registry.getNumberOfYoVariables();
      int children = registry.getChildren().size();

      if (variables >= minVariablesToPrint || children >= minChildrenToPrint)
         registriesOfInterest.add(registry);

      int totalNumberOfVariables = variables;
      for (int childIdx = 0; childIdx < children; childIdx++)
      {
         YoVariableRegistry childRegistry = registry.getChildren().get(childIdx);
         totalNumberOfVariables += YoVariableRegistry.collectRegistries(minVariablesToPrint, minChildrenToPrint, childRegistry, registriesOfInterest);
      }

      return totalNumberOfVariables;
   }

   private static void printInfo(YoVariableRegistry registry)
   {
      int variables = registry.getNumberOfYoVariables();
      int children = registry.getChildren().size();

      int maxPropertyLength = 17;
      String variableString = trimStringToLength("Variables: " + variables, maxPropertyLength, "...");
      String childrenString = trimStringToLength("Children: " + children, maxPropertyLength, "...");

      int maxNameLength = 70;
      String name = registry.getClass().getSimpleName() + " " + registry.getNameSpace().getName();
      name = trimStringToLength(name, maxNameLength, "...");

      System.out.println(name + "\t" + variableString + "\t" + childrenString);
   }

   private static String trimStringToLength(String original, int length, String placeholder)
   {
      int chararcters = original.length();
      int placeholderLength = placeholder.length();

      if (chararcters > length)
         return original.substring(0, length - placeholderLength) + placeholder;
      else
         return StringUtils.rightPad(original, length, " ");
   }

}