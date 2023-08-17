/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.registry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.yoVariables.exceptions.IllegalNameException;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.exceptions.NameCollisionException;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener.Change;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Data structure for creating, managing, and interacting with a hierarchy of {@code YoVariable}.
 * <p>
 * Registries are organized into a tree structure with one parent, variables, and child registries.
 * Each {@code YoVariable} can only be registered to one registry at a time.
 * </p>
 */
public class YoRegistry implements YoVariableHolder
{
   /** The name of this registry. */
   private final String name;
   /**
    * The namespace of this registry, it is the collection of the registry names from the root to here.
    */
   private YoNamespace namespace;

   /** The list of variables that are currently registered in {@code this}. */
   private final List<YoVariable> variables = new ArrayList<>();
   /**
    * Mapping from the lower-case simple name of a variable to the instance of the variable.
    * Facilitates retrieval of registered variables from their name.
    */
   private final Map<String, YoVariable> nameToVariableMap = new LinkedHashMap<>();
   /**
    * The list of parameters that are currently registered in this {@code this}. This list is mostly
    * used for book keeping.
    */
   private final List<YoParameter> parameters = new ArrayList<>();

   /**
    * The registry to which this registry is currently a child of, or {@code null} if {@code this} is
    * the root.
    */
   private YoRegistry parent;
   /** The list of child registries that are currently registered in {@code this}. */
   private final List<YoRegistry> children = new ArrayList<>();
   /**
    * Mapping from the lower-case simple name of a registry to the instance of the registry.
    * Facilitates retrieval of registered children from their name.
    */
   private final Map<String, YoRegistry> nameToChildMap = new LinkedHashMap<>();

   /**
    * List of active listeners currently attached to this registry. Only instantiated when adding the
    * first listener.
    */
   private List<YoRegistryChangedListener> changedListeners;

   /**
    * Current level of restriction to apply to this registry, also indicates the minimum level of
    * restriction to be applied on the descendants of this registry.
    */
   private YoRegistryRestrictionLevel restrictionLevel = YoRegistryRestrictionLevel.FULLY_MUTABLE;

   /**
    * Creates a new registry.
    * <p>
    * Typically, the new registry will need to be registered as a child of another registry that is
    * accessible from variable visualizer, for instance simulation or server.
    * </p>
    *
    * @param name the name of the new registry.
    * @throws IllegalNameException if the name contains any of the following characters:
    *                              {@value YoTools#ILLEGAL_CHARACTERS_REGEX}.
    */
   public YoRegistry(String name)
   {
      if (name == null || name.isEmpty())
         throw new IllegalArgumentException("Cannot create a registry without a name.");

      YoTools.checkForIllegalCharacters(name);
      this.name = name;
      namespace = new YoNamespace(name);
   }

   /**
    * Gets the name of this registry.
    *
    * @return this registry's name.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Gets the current namespace for this registry.
    * <p>
    * The namespace contains in order the name of all the registries from the root ending with this
    * registry.
    * </p>
    *
    * @return this registry's namespace.
    */
   public YoNamespace getNamespace()
   {
      return namespace;
   }

   /**
    * Sets the desired level of restriction for this registry.
    * <p>
    * This allows to control restrictions to be applied on this registry and its subtree, such as
    * whether it is permitted to add and/or remove variables and registries. The level can only be
    * changed such that it is more restrictive. The level of restriction of the registries in the
    * subtree is automatically raised to match the new mode. Registries in the subtree that were
    * previously configured with a higher restriction level are not affected by this operation.
    * </p>
    *
    * @param restrictionLevel the new restriction level for this registry.
    */
   public void setRestrictionLevel(YoRegistryRestrictionLevel restrictionLevel)
   {
      if (this.restrictionLevel.ordinal() > restrictionLevel.ordinal())
         throw new IllegalArgumentException("Cannot reduce restriction level. Current mode: " + restrictionLevel + ", tried to set to: " + restrictionLevel);

      this.restrictionLevel = restrictionLevel;

      for (int i = 0; i < children.size(); i++)
      {
         YoRegistry child = children.get(i);
         if (child.restrictionLevel.ordinal() < restrictionLevel.ordinal())
            child.setRestrictionLevel(restrictionLevel);
      }
   }

   /**
    * Gets the current restriction level for this registry.
    *
    * @return this registry's restriction level.
    */
   public YoRegistryRestrictionLevel getRestrictionLevel()
   {
      return restrictionLevel;
   }

   /**
    * Clears the internal data of this registry and its children.
    * <p>
    * The root registry can be cleared regardless of its current restriction level, in which case the
    * restriction level is reverted to {@link YoRegistryRestrictionLevel#FULLY_MUTABLE} such that the
    * registry can be used. However, for any registry that is not the root, the restriction level needs
    * to allow for removing variables and registries.
    * </p>
    * <p>
    * After a registry has been clear, it contains no variables and no child registries. This registry
    * is detached from its parent and the variables detached from this registry.
    * </p>
    *
    * @throws IllegalOperationException if this registry is not fully mutable and is not the root.
    */
   public void clear()
   {
      if (!isRoot() && restrictionLevel != YoRegistryRestrictionLevel.FULLY_MUTABLE)
         throw new IllegalOperationException("Cannot clear a registry that is not the root and that does not have appropriate restriction level.");
      detachFromParent();
      clearInternal(false);
      notifyListeners(null, null, null, ChangeType.CLEARED);
      changedListeners = null;
   }

   private void clearInternal(boolean clearListeners)
   {
      variables.forEach(variable -> variable.clear());
      variables.clear();
      nameToVariableMap.clear();
      parameters.clear();
      children.forEach(child -> child.clearInternal(true));
      children.clear();
      nameToChildMap.clear();
      restrictionLevel = YoRegistryRestrictionLevel.FULLY_MUTABLE;
      if (clearListeners)
         changedListeners = null;
   }

   /**
    * Adds a listener to this registry.
    *
    * @param listener the listener for listening to changes done on this registry and its descendants.
    */
   public void addListener(YoRegistryChangedListener listener)
   {
      if (changedListeners == null)
         changedListeners = new ArrayList<>();
      changedListeners.add(listener);
   }

   /**
    * Removes all listeners previously added to this registry.
    */
   public void removeListeners()
   {
      changedListeners = null;
   }

   /**
    * Tries to remove a listener from this registry. If the listener could not be found and removed,
    * nothing happens.
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed, {@code false} if the listener was not found and
    *         nothing happened.
    */
   public boolean removeListener(YoRegistryChangedListener listener)
   {
      if (changedListeners == null)
         return false;
      return changedListeners.remove(listener);
   }

   /**
    * Registers a new {@code YoVariable} in this registry.
    * <p>
    * The variable will become available for search queries from this registry and any of its parent
    * registries.
    * </p>
    * <p>
    * If the variable was already registered to this registry, this method does nothing.
    * </p>
    *
    * @param variable the new variable to register.
    * @throws NameCollisionException    if the new variable's name collide with another variable
    *                                   previously registered.
    * @throws IllegalOperationException if the operation is not permitted.
    */
   public void addVariable(YoVariable variable)
   {
      if (!restrictionLevel.isAdditionAllowed())
         throw new IllegalOperationException("Cannot add variables to this registry: " + namespace);

      if (nameToVariableMap.containsValue(variable))
         return;

      // Make everything case insensitive! Cannot have two YoVariables with same names except case.
      String variableName = variable.getName().toLowerCase();

      if (nameToVariableMap.containsKey(variableName))
         throw new NameCollisionException("Name collision for new variable: " + variableName + ". Parent name space = " + getNamespace());

      if (variable.getRegistry() != null)
         variable.getRegistry().removeVariable(variable);

      variables.add(variable);
      nameToVariableMap.put(variableName, variable);
      variable.setRegistry(this);

      if (variable.isParameter())
      {
         parameters.add(variable.getParameter());
      }

      notifyListeners(this, null, variable, ChangeType.VARIABLE_ADDED);
   }

   /**
    * Removes a variable previously registered in this registry.
    *
    * @param variable the variable to remove.
    * @throws IllegalOperationException if the operation is not permitted.
    */
   public void removeVariable(YoVariable variable)
   {
      if (!nameToVariableMap.containsValue(variable))
         return;

      String variableName = variable.getName().toLowerCase();

      if (!restrictionLevel.isRemovalAllowed())
         throw new IllegalOperationException("Cannot remove variables from this registry: " + namespace);

      variable.setRegistry(null);
      variables.remove(variable);
      nameToVariableMap.remove(variableName);

      if (variable.isParameter())
         parameters.remove(variable.getParameter());

      notifyListeners(this, null, variable, ChangeType.VARIABLE_REMOVED);
   }

   /**
    * Adds child registry to this registry.
    * <p>
    * The registry and all its subtree will become available for search queries from this registry and
    * any of its parent registries.
    * </p>
    *
    * @param child the new registry to add.
    * @throws NameCollisionException    if adding the new registry would cause a name collision.
    * @throws IllegalOperationException if the operation is not permitted.
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
    * <p>
    * If the given registry is already a child of this registry, this method does nothing.
    * </p>
    *
    * @param child           the new registry to add.
    * @param notifyListeners indicates whether this operation should trigger the change listeners.
    *                        Default value is {@code true}.
    * @throws NameCollisionException    if adding the new registry would cause a name collision.
    * @throws IllegalOperationException if the operation is not permitted.
    */
   public void addChild(YoRegistry child, boolean notifyListeners)
   {
      if (child == null)
         return;
      if (child == this)
         throw new IllegalOperationException("Cannot register a registry as a child of itself, registry: " + namespace);

      if (!restrictionLevel.isAdditionAllowed())
         throw new IllegalOperationException("Cannot add children to this registry: " + namespace);

      if (nameToChildMap.containsValue(child))
         return;

      String childName = child.getName().toLowerCase();

      // Make sure no children with this name already:
      if (nameToChildMap.containsKey(childName))
         throw new NameCollisionException("Name collision for new child: " + childName + ". Parent name space = " + getNamespace());

      child.detachFromParent();
      child.parent = this;
      child.setParentNamespace(namespace);
      if (child.getRestrictionLevel().ordinal() < restrictionLevel.ordinal())
         child.setRestrictionLevel(restrictionLevel);

      children.add(child);
      nameToChildMap.put(childName, child);

      if (notifyListeners)
         notifyListeners(this, child, null, ChangeType.REGISTRY_ADDED);
   }

   /**
    * Removes a child registry that was previously added in this registry.
    *
    * @param child the registry to remove.
    * @throws IllegalOperationException if the operation is not permitted.
    */
   public void removeChild(YoRegistry child)
   {
      if (child == null || child.getParent() != this)
         return;

      if (!restrictionLevel.isRemovalAllowed())
         throw new IllegalOperationException("Cannot remove children from this registry: " + namespace);

      String childName = child.getName().toLowerCase();

      child.parent = null;
      child.setParentNamespace(null);

      children.remove(child);
      nameToChildMap.remove(childName);

      notifyListeners(this, child, null, ChangeType.REGISTRY_REMOVED);
   }

   /**
    * Detaches this registry from its parent if it has any.
    *
    * @throws IllegalOperationException if the operation is not permitted.
    */
   public void detachFromParent()
   {
      if (parent == null)
         return;
      parent.removeChild(this);
   }

   /**
    * Convenience method for updating the namespace of this registry when its parent is being updated.
    *
    * @param parentNamespace the new namespace of this registry's parent.
    * @throws IllegalNameException if the namespace sanity check fails.
    */
   private void setParentNamespace(YoNamespace parentNamespace)
   {
      if (parentNamespace == null)
         namespace = new YoNamespace(name);
      else
         namespace = parentNamespace.append(name);
      namespace.checkSanity();
      children.forEach(child -> child.setParentNamespace(namespace));
      variables.forEach(variable -> variable.resetFullName());
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
    * Returns a registry previously registered in this registry.
    *
    * @param name the name of the registry to get.
    * @return the registry corresponding to the given {@code name}, or {@code null} if such registry
    *         has not been registered.
    */
   public YoRegistry getChild(String name)
   {
      return nameToChildMap.get(name.toLowerCase());
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
   public YoVariable getVariable(String name)
   {
      return nameToVariableMap.get(name.toLowerCase());
   }

   /**
    * Returns the variables that have been registered to this registry.
    *
    * @return unmodifiable list of this registry's variables.
    */
   @Override
   public List<YoVariable> getVariables()
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
   public YoParameter getParameter(String name)
   {
      YoVariable yoVariable = getVariable(name);
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
   public List<YoParameter> getParameters()
   {
      return Collections.unmodifiableList(parameters);
   }

   /**
    * Collects and returns all the variables contained in this registry and in its descendants.
    *
    * @return list of all variables registered to this registry and its child registries.
    */
   public List<YoVariable> collectSubtreeVariables()
   {
      List<YoVariable> yoVariables = new ArrayList<>();
      collectSubtreeVariables(yoVariables);
      return yoVariables;
   }

   /**
    * Collects and packs all the variables contained in this registry and in its descendants into
    * {@code variablesToPack}.
    *
    * @param variablesToPack list used to store all variables registered to this registry and its child
    *                        registries.
    */
   private void collectSubtreeVariables(List<YoVariable> variablesToPack)
   {
      // Add ours:
      variablesToPack.addAll(variables);

      // Add children's recursively:
      for (YoRegistry child : children)
      {
         child.collectSubtreeVariables(variablesToPack);
      }
   }

   /**
    * Collects recursively and returns all the parameters contained in this registry and in its
    * descendants.
    *
    * @return list of all parameters registered to this registry and its child registries.
    */
   public List<YoParameter> collectSubtreeParameters()
   {
      List<YoParameter> yoParameters = new ArrayList<>();
      collectSubtreeParameters(yoParameters);
      return yoParameters;
   }

   /**
    * Collects recursively and packs all the parameters contained in this registry and in its
    * descendants into {@code parametersToPack}.
    *
    * @param parametersToPack list used to store all parameters registered to this registry and its
    *                         child registries.
    */
   private void collectSubtreeParameters(List<YoParameter> parametersToPack)
   {
      // Add ours:
      parametersToPack.addAll(parameters);

      // Add children's recursively:
      for (YoRegistry child : children)
      {
         child.collectSubtreeParameters(parametersToPack);
      }
   }

   /**
    * Collects recursively and returns the subtree of registries starting with this registry.
    *
    * @return list of all the registries composing the subtree starting at {@code this}.
    */
   public List<YoRegistry> collectSubtreeRegistries()
   {
      List<YoRegistry> yoVariableRegistries = new ArrayList<>();
      collectSubtreeRegistries(yoVariableRegistries);
      return yoVariableRegistries;
   }

   /**
    * Collects recursively and packs the subtree of registries starting with this registry into
    * {@code yoVariableRegistriesToPack}.
    *
    * @param yoVariableRegistriesToPack list used to store all the registries composing the subtree
    *                                   starting at {@code this}.
    */
   public void collectSubtreeRegistries(List<YoRegistry> yoVariableRegistriesToPack)
   {
      // Add mine:
      yoVariableRegistriesToPack.add(this);

      // Add all the children recursively:
      for (YoRegistry child : children)
      {
         child.collectSubtreeRegistries(yoVariableRegistriesToPack);
      }
   }

   /**
    * {@inheritDoc}
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    */
   @Override
   public YoVariable findVariable(String name)
   {
      return YoVariableHolder.super.findVariable(name);
   }

   /**
    * {@inheritDoc}
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    */
   @Override
   public YoVariable findVariable(String namespaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      return YoSearchTools.findFirstVariable(namespaceEnding, name, null, this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    */
   @Override
   public List<YoVariable> findVariables(String name)
   {
      return YoVariableHolder.super.findVariables(name);
   }

   /**
    * {@inheritDoc}
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    */
   @Override
   public List<YoVariable> findVariables(String namespaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      return YoSearchTools.findVariables(namespaceEnding, name, null, this, null);
   }

   /**
    * Finds the registry matching the given {@code namespace} and returns its variables.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    */
   @Override
   public List<YoVariable> findVariables(YoNamespace namespace)
   {
      YoRegistry registry = findRegistry(namespace);
      if (registry == null)
         return Collections.emptyList();
      else
         return registry.getVariables();
   }

   /**
    * Returns the first discovered instance of a registry matching the given name.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    *
    * @param name the name of the registry to retrieve. If the name contains
    *             {@link YoTools#NAMESPACE_SEPERATOR_STRING}, it is split at the last occurrence to
    *             extract a namespace and the actual registry name.
    * @return the registry corresponding to the search criteria, or {@code null} if it could not be
    *         found.
    * @see #findRegistry(String, String)
    */
   public YoRegistry findRegistry(String name)
   {
      int separatorIndex = name.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      if (separatorIndex == -1)
         return findRegistry(null, name);
      else
         return findRegistry(name.substring(0, separatorIndex), name.substring(separatorIndex + 1));
   }

   /**
    * Returns the first discovered instance of a registry matching the given name and namespace.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    *
    * @param namespaceEnding (optional) the namespace of the registry in which the registry was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search is for the registry name only.
    * @param name            the name of the registry to retrieve.
    * @return the registry corresponding to the search criteria, or {@code null} if it could not be
    *         found.
    * @throws IllegalNameException if {@code name} contains "{@value YoTools#NAMESPACE_SEPERATOR}".
    */
   public YoRegistry findRegistry(String namespaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      return YoSearchTools.findFirstRegistry(namespaceEnding, name, null, this);
   }

   /**
    * Finds and returns the registry with the given {@code namespace}.
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    *
    * @param namespaceEnding the namespace of the registry of interest. The namespace does not need to
    *                        be complete, i.e. it does not need to contain the name of the registries
    *                        closest to the root registry.
    * @return the registry which namespace matches the given one, or {@code null} if it could not be
    *         found.
    */
   public YoRegistry findRegistry(YoNamespace namespaceEnding)
   {
      if (this.namespace.endsWith(namespaceEnding))
         return this;

      // TODO Can likely speed up the search by performing smart comparison of the children namespace with the query.
      for (YoRegistry child : children)
      {
         YoRegistry registry = child.findRegistry(namespaceEnding);
         if (registry != null)
            return registry;
      }

      return null;
   }

   /**
    * {@inheritDoc}
    * <p>
    * The search is first conducted in this registry, then in its children in the order in which they
    * were added.
    * </p>
    */
   @Override
   public boolean hasUniqueVariable(String namespaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      return countNumberOfVariables(namespaceEnding, name) == 1;
   }

   private int countNumberOfVariables(String parentNamespace, String name)
   {
      int count = 0;

      if (parentNamespace == null || namespace.endsWith(parentNamespace))
      {
         if (nameToVariableMap.containsKey(name.toLowerCase()))
            count++;
      }

      for (YoRegistry child : children)
      {
         count += child.countNumberOfVariables(parentNamespace, name);
      }

      return count;
   }

   /**
    * Returns the number of variables this registry contains.
    *
    * @return the number of variables in this registry only.
    */
   public int getNumberOfVariables()
   {
      return variables.size();
   }

   /**
    * Returns the number of variables in the sub-tree starting at this registry contains.
    *
    * @return the number of variables in this registry and its descendants.
    */
   public int getNumberOfVariablesDeep()
   {
      int numberOfVariables = variables.size();

      for (int i = 0; i < children.size(); i++)
      {
         numberOfVariables += children.get(i).getNumberOfVariablesDeep();
      }

      return numberOfVariables;
   }

   /**
    * Return the variable at the given index, the variables are stored in the order they were
    * registered.
    *
    * @param index the index of the variable of interest.
    * @return the corresponding variable.
    */
   public YoVariable getVariable(int index)
   {
      return variables.get(index);
   }

   private void notifyListeners(YoRegistry targetParentRegistry, YoRegistry targetRegistry, YoVariable targetVariable, ChangeType type)
   {
      RegistryChange change;
      if (changedListeners != null)
      {
         // Making a copy ensures that the source of the event is 'this'.
         change = new RegistryChange(targetParentRegistry, targetRegistry, targetVariable, type);
         for (YoRegistryChangedListener listener : changedListeners)
         {
            listener.changed(change);
         }
      }

      if (parent != null)
      {
         parent.notifyListeners(targetParentRegistry, targetRegistry, targetVariable, type);
      }
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!(object instanceof YoRegistry))
         return false;

      YoRegistry other = (YoRegistry) object;

      if (!getNamespace().equals(other.getNamespace()))
         return false;

      if (variables.size() != other.variables.size())
         return false;

      for (YoVariable variable : variables)
      {
         if (!other.nameToVariableMap.containsKey(variable.getName().toLowerCase()))
            return false;
      }

      if (children.size() != other.children.size())
         return false;

      for (YoRegistry child : children)
      {
         if (!other.nameToChildMap.containsKey(child.getName().toLowerCase()))
            return false;
      }

      return true;
   }

   @Override
   public int hashCode()
   {
      return namespace.hashCode();
   }

   @Override
   public String toString()
   {
      return namespace.getName();
   }

   private enum ChangeType
   {
      REGISTRY_ADDED, REGISTRY_REMOVED, VARIABLE_ADDED, VARIABLE_REMOVED, CLEARED
   }

   private final class RegistryChange implements Change
   {
      private final YoRegistry targetParentRegistry;
      private final YoRegistry targetRegistry;
      private final YoVariable targetVariable;
      private final ChangeType type;

      public RegistryChange(YoRegistry targetParentRegistry, YoRegistry targetRegistry, YoVariable targetVariable, ChangeType type)
      {
         this.targetParentRegistry = targetParentRegistry;
         this.targetRegistry = targetRegistry;
         this.targetVariable = targetVariable;
         this.type = type;
      }

      @Override
      public boolean wasRegistryAdded()
      {
         return type == ChangeType.REGISTRY_ADDED;
      }

      @Override
      public boolean wasRegistryRemoved()
      {
         return type == ChangeType.REGISTRY_REMOVED;
      }

      @Override
      public boolean wasVariableAdded()
      {
         return type == ChangeType.VARIABLE_ADDED;
      }

      @Override
      public boolean wasVariableRemoved()
      {
         return type == ChangeType.VARIABLE_REMOVED;
      }

      @Override
      public boolean wasCleared()
      {
         return type == ChangeType.CLEARED;
      }

      @Override
      public YoRegistry getSource()
      {
         return YoRegistry.this;
      }

      @Override
      public YoRegistry getTargetParentRegistry()
      {
         return targetParentRegistry;
      }

      @Override
      public YoRegistry getTargetRegistry()
      {
         return targetRegistry;
      }

      @Override
      public YoVariable getTargetVariable()
      {
         return targetVariable;
      }

      @Override
      public String toString()
      {
         switch (type)
         {
            case REGISTRY_ADDED:
               return String.format("Added registry: %s. Child of: %s. Source of event: %s",
                                    targetRegistry.getName(),
                                    targetParentRegistry.getName(),
                                    getName());
            case REGISTRY_REMOVED:
               return String.format("Removed registry: %s. Was child of: %s. Source of event: %s",
                                    targetRegistry.getName(),
                                    targetParentRegistry.getName(),
                                    getName());
            case VARIABLE_ADDED:
               return String.format("Added variable: %s. Registered in: %s. Source of event: %s",
                                    targetVariable.getName(),
                                    targetParentRegistry.getName(),
                                    getName());
            case VARIABLE_REMOVED:
               return String.format("Removed variable: %s. Was registered in: %s. Source of event: %s",
                                    targetVariable.getName(),
                                    targetParentRegistry.getName(),
                                    getName());
            case CLEARED:
               return String.format("Cleared registry: %s.", getName());

            default:
               return "Unexpected event type: " + type;
         }
      }
   }
}