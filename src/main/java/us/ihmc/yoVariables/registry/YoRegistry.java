package us.ihmc.yoVariables.registry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import us.ihmc.yoVariables.dataBuffer.YoVariableHolder;
import us.ihmc.yoVariables.listener.YoVariableRegistryChangedListener;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoTools.SearchQuery;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoRegistry implements YoVariableHolder
{
   private final String name;
   private NameSpace nameSpace;

   private final List<YoVariable<?>> variables = new ArrayList<>();
   private final Map<String, YoVariable<?>> nameToVariableMap = new LinkedHashMap<>(); // From name to the variable with that name.
   private final List<YoParameter<?>> parameters = new ArrayList<>();

   private YoRegistry parent;
   private final List<YoRegistry> children = new ArrayList<>();
   private final Map<String, YoRegistry> nameToChildMap = new LinkedHashMap<>();

   private List<YoVariableRegistryChangedListener> yoVariableRegistryChangedListeners;

   public YoRegistry(String name)
   {
      YoTools.checkForIllegalCharacters(name);

      this.name = name;
      nameSpace = new NameSpace(name);
   }

   public String getName()
   {
      return name;
   }

   public NameSpace getNameSpace()
   {
      return nameSpace;
   }

   /**
    * Registers a new {@code YoVariable} in this registry.
    * <p>
    * The variable will become available for search queries from this registry and any of its parent
    * registries.
    * </p>
    *
    * @param variable the new variable to register.
    * @throws NameCollisionException if the new variable's name collide with another variable
    *                                previously registered.
    */
   public void addYoVariable(YoVariable<?> variable)
   {
      // Make everything case insensitive! Cannot have two YoVariables with same names except case.
      String variableName = variable.getName().toLowerCase();

      if (nameToVariableMap.containsKey(variableName))
      {
         throw new NameCollisionException("Name collision for new variable: " + variableName + ". Parent name space = " + getNameSpace());
      }

      variables.add(variable);
      nameToVariableMap.put(variableName, variable);

      if (variable.isParameter())
      {
         parameters.add(variable.getParameter());
      }

      notifyListenersYoVariableWasRegistered(variable);
   }

   /**
    * Adds child registry to this registry.
    * <p>
    * The registry and all its subtree will become available for search queries from this registry and
    * any of its parent registries.
    * </p>
    *
    * @param child the new registry to add.
    * @throws NameCollisionException if adding the new registry would cause a name collision.
    */
   public void addChild(YoRegistry child)
   {
      addChild(child, true);
   }

   /**
    * Adds child registry to this registry.
    * <p>
    * The registry and all its subtree will become available for search queries from this registry and
    * any of its parent registries.
    * </p>
    *
    * @param child           the new registry to add.
    * @param notifyListeners indicates whether this operation should trigger the change listeners.
    *                        Default value is {@code true}.
    * @throws NameCollisionException if adding the new registry would cause a name collision.
    */
   public void addChild(YoRegistry child, boolean notifyListeners)
   {
      if (child == null)
         return;

      String childName = child.getName().toLowerCase();

      // Make sure no children with this name already:
      if (nameToChildMap.containsKey(childName))
         throw new NameCollisionException("Name collision for new child: " + childName + ". Parent name space = " + getNameSpace());

      child.setParent(this);
      child.prependNameSpace(nameSpace);
      children.add(child);
      nameToChildMap.put(childName, child);

      if (notifyListeners)
         notifyListenersYoVariableRegistryWasAdded(child);
   }

   /**
    * Tests for the presence of at least one parameter in this registry.
    *
    * @return {@code true} if this registry has at least one parameter, {@code false} otherwise.
    */
   public boolean hasParameters()
   {
      return !parameters.isEmpty();
   }

   /**
    * Tests for the presence of at least one parameter in the subtree starting at this registry.
    *
    * @return {@code true} if at least one parameter was found in either this registry or any of its
    *         descendants, {@code false} otherwise.
    */
   public boolean hasParametersDeep()
   {
      if (hasParameters())
         return true;

      for (int i = 0; i < children.size(); i++)
      {
         if (children.get(i).hasParametersDeep())
         {
            return true;
         }
      }

      return false;
   }

   /**
    * Returns whether this registry is the root registry, i.e. it has no parent.
    * 
    * @return {@code true} if this registry is the root, {@code false} otherwise.
    */
   public boolean isRoot()
   {
      return parent == null;
   }

   /**
    * Retrieves and returns the root registry.
    * 
    * @return the root registry or {@code this} if it is the root.
    */
   public YoRegistry getRoot()
   {
      if (isRoot())
         return this;
      else
         return parent.getRoot();
   }

   /**
    * Returns the parent registry for this registry or {@code null} if {@code this} is the root
    * registry.
    *
    * @return the parent or {@code null} if {@code this} is the root registry.
    */
   public YoRegistry getParent()
   {
      return parent;
   }

   /**
    * Returns the child registries contained in this registry.
    *
    * @return this registry direct children.
    */
   public List<YoRegistry> getChildren()
   {
      return children;
   }

   /**
    * Returns a variable previously registered in this registry.
    *
    * @param name the name of the variable to get.
    * @return the variable corresponding to the given {@code name}, or {@code null} if such variable
    *         has not been registered.
    */
   public YoVariable<?> getVariable(String name)
   {
      return nameToVariableMap.get(name);
   }

   /**
    * Returns the variables that have been registered to this registry.
    *
    * @return unmodifiable list of this registry's variables.
    */
   @Override
   public List<YoVariable<?>> getVariables()
   {
      return Collections.unmodifiableList(variables);
   }

   /**
    * Returns a parameter previously registered in this registry.
    *
    * @param name the name of the parameter to get.
    * @return the parameter corresponding to the given {@code name}, or {@code null} if such parameter
    *         has not been registered.
    */
   public YoParameter<?> getParameter(String name)
   {
      YoVariable<?> yoVariable = getVariable(name);
      if (yoVariable != null && yoVariable.isParameter())
         return yoVariable.getParameter();
      else
         return null;
   }

   /**
    * Returns the parameters that have been registered to this registry.
    *
    * @return unmodifiable list of this registry's parameters.
    */
   public List<YoParameter<?>> getParameters()
   {
      return Collections.unmodifiableList(parameters);
   }

   /**
    * Collects and returns all the variables contained in this registry and in its descendants.
    *
    * @return list of all variables registered to this registry and its child registries.
    */
   public List<YoVariable<?>> getSubtreeVariables()
   {
      List<YoVariable<?>> yoVariables = new ArrayList<>();
      getSubtreeVariables(yoVariables);
      return yoVariables;
   }

   /**
    * Collects and packs all the variables contained in this registry and in its descendants into
    * {@code variablesToPack}.
    *
    * @param variablesToPack list used to store all variables registered to this registry and its child
    *                        registries.
    */
   private void getSubtreeVariables(List<YoVariable<?>> variablesToPack)
   {
      // Add ours:
      variablesToPack.addAll(variables);

      // Add children's recursively:
      for (YoRegistry registry : children)
      {
         registry.getSubtreeVariables(variablesToPack);
      }
   }

   /**
    * Collects recursively and returns all the parameters contained in this registry and in its
    * descendants.
    *
    * @return list of all parameters registered to this registry and its child registries.
    */
   public List<YoParameter<?>> getSubtreeParameters()
   {
      List<YoParameter<?>> yoParameters = new ArrayList<>();
      getSubtreeParameters(yoParameters);
      return yoParameters;
   }

   /**
    * Collects recursively and packs all the parameters contained in this registry and in its
    * descendants into {@code parametersToPack}.
    *
    * @param parametersToPack list used to store all parameters registered to this registry and its
    *                         child registries.
    */
   private void getSubtreeParameters(List<YoParameter<?>> parametersToPack)
   {
      // Add ours:
      parametersToPack.addAll(parameters);

      // Add children's recursively:
      for (YoRegistry registry : children)
      {
         registry.getSubtreeParameters(parametersToPack);
      }
   }

   /**
    * Collects recursively and returns the subtree of registries starting with this registry.
    *
    * @return list of all the registries composing the subtree starting at {@code this}.
    */
   public List<YoRegistry> getSubtreeRegistries()
   {
      List<YoRegistry> yoVariableRegistries = new ArrayList<>();
      getSubtreeRegistries(yoVariableRegistries);
      return yoVariableRegistries;
   }

   /**
    * Collects recursively and packs the subtree of registries starting with this registry into
    * {@code yoVariableRegistriesToPack}.
    *
    * @param yoVariableRegistriesToPack list used to store all the registries composing the subtree
    *                                   starting at {@code this}.
    */
   public void getSubtreeRegistries(List<YoRegistry> yoVariableRegistriesToPack)
   {
      // Add mine:
      yoVariableRegistriesToPack.add(this);

      // Add all the children recursively:
      for (YoRegistry child : children)
      {
         child.getSubtreeRegistries(yoVariableRegistriesToPack);
      }
   }

   /**
    * Returns a variable previously registered in this registry.
    *
    * @param name the name of the variables to get.
    */
   @Override
   public YoVariable<?> findVariable(String name)
   {
      return YoTools.findYoVariable(new SearchQuery(name), null, this);
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
    * @throws IllegalNameException if {@code name} contains "{@value YoTools#NAMESPACE_SEPERATOR}".
    */
   @Override
   public YoVariable<?> findVariable(String parentNameSpace, String name)
   {
      return YoTools.findYoVariable(new SearchQuery(parentNameSpace, name), null, this);
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
   public List<YoVariable<?>> findVariables(String name)
   {
      return YoTools.findYoVariables(new SearchQuery(name), null, this, null);
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
    * @throws IllegalNameException if {@code name} contains "{@value YoTools#NAMESPACE_SEPERATOR}".
    */
   @Override
   public List<YoVariable<?>> findVariables(String parentNameSpace, String name)
   {
      return YoTools.findYoVariables(new SearchQuery(parentNameSpace, name), null, this, null);
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
   public boolean hasUniqueVariable(String name)
   {
      int separatorIndex = name.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return hasUniqueVariable(null, name);
      else
         return hasUniqueVariable(name.substring(0, separatorIndex + 1), name.substring(separatorIndex));
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
   public boolean hasUniqueVariable(String parentNameSpace, String name)
   {
      if (name.contains(YoTools.NAMESPACE_SEPERATOR_STRING))
         throw new IllegalNameException(name + " cannot contain '" + YoTools.NAMESPACE_SEPERATOR_STRING + "'.");

      return countNumberOfVariables(parentNameSpace, name) == 1;
   }

   private int countNumberOfVariables(String parentNameSpace, String name)
   {
      int count = 0;

      if (parentNameSpace == null || nameSpace.endsWith(parentNameSpace))
      {
         if (nameToVariableMap.containsKey(name))
            count++;
      }

      for (YoRegistry child : children)
      {
         count += child.countNumberOfVariables(parentNameSpace, name);
      }

      return count;
   }

   private void prependNameSpace(NameSpace parentNameSpace)
   {
      // Fix my name
      nameSpace = new NameSpace(parentNameSpace.getName() + "." + nameSpace.getName());

      // Fix my children
      for (YoRegistry child : children)
      {
         child.prependNameSpace(parentNameSpace);
      }
   }

   private void setParent(YoRegistry parent)
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

   public YoRegistry getRegistry(NameSpace fullNameSpace)
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
         for (YoRegistry child : children)
         {
            YoRegistry registry = child.getRegistry(fullNameSpace);
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

   @Override
   public List<YoVariable<?>> getVariables(NameSpace nameSpace)
   {
      List<YoVariable<?>> ret = new ArrayList<>();

      List<YoVariable<?>> allVariables = getSubtreeVariables();

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
               YoVariable<?> var = findVariable(name);

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

      for (YoRegistry child : children)
      {
         child.recursivelyGetMatchingVariables(ret, regularExpressions);
      }
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof YoRegistry)
         return equals((YoRegistry) object);
      else
         return false;
   }

   public boolean equals(YoRegistry other)
   {
      if (other == this)
         return true;
      if (other == null)
         return false;

      if (!getNameSpace().equals(other.getNameSpace()))
         return false;

      if (variables.size() != other.variables.size())
         return false;

      for (YoVariable<?> variable : variables)
      {
         if (!other.nameToVariableMap.containsKey(variable.getName()))
            return false;
      }

      if (children.size() != other.children.size())
         return false;

      for (YoRegistry child : children)
      {
         if (!other.nameToChildMap.containsKey(child.getName()))
            return false;
      }

      return true;
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

   private void verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners()
   {
      if (parent != null && yoVariableRegistryChangedListeners != null)
      {
         throw new RuntimeException("Only root YoVariableRegistries should have listeners. This registry does! YoVariableRegistry = " + this);
      }
   }

   private void notifyListenersYoVariableRegistryWasAdded(YoRegistry child)
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

   private void notifyListenersYoVariableRegistryWasCleared(YoRegistry registry)
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
}